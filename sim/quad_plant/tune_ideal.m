function [gains, info] = tune_ideal(varargin)
%TUNE_IDEAL Tune the ideal controller's gains on analytic per-axis linear plants.
%   [gains, info] = tune_ideal()                 tune and print
%   [gains, info] = tune_ideal('apply', true)    also write gains into base `ideal`
%   [gains, info] = tune_ideal('verify', true)   apply + run the nonlinear step
%                                                scenarios (run_quad_scenarios)
%   Options: 'PM' rate-loop phase margin deg (default 45), 'GM' gain margin
%   dB (8), 'PMo' outer angle-loop PM deg (60), 'wi' integrator zero rad/s
%   (Ki = wi*Kp, default 6), 'Td' allowed Kd/Kp range s ([2 25] ms; the
%   board's nominal Kd/Kp is 8.4 ms), 'sep' min ratio of rate/angle
%   crossover (4), 'plot' (false).
%
%   Method (no Simulink Control Design needed, Control System Toolbox only):
%   1. Linearise the plant at hover by hand from `quad`:
%        motor  dw/dV_m = Km/(tau_m s + 1),  Km = (Kt/R)/(Kt^2/R + 2 kQ w0),
%               tau_m = J_r/(Kt^2/R + 2 kQ w0)            (~23 ms)
%        roll   p/u = (Vbus/1000)*Km*2kT w0*4*|y_arm| / ((tau_m s+1)(Jx s + k_rot_x))
%        pitch  same with |x_arm|, Jy
%        yaw    r/u = (Vbus/1000)*Km*4*(J_r s + 2 kQ w0) / ((tau_m s+1)(Jz s + k_rot_z))
%      u in us per motor, through the Quad-X mixer (|mix| = 1).
%      Loop delays: controller ZOH Ts/2 + ESC PWM frame/2 + ESC latency
%      + IMU (1st-order DLPF at its bandwidth + nd samples).
%   2. pidtune(G, 'PIDF') gives a starting point, then the board-structured
%      controller C = Kp + Ki/(s+wL) + Kd s/((1+s/w1)(1+s/w2)) (w1, w2 fixed
%      by the board's D network, Ki = wi*Kp, Kd = Td*Kp) is grid-searched
%      over (Kp, Td) for the highest crossover that meets PM and GM (margin()).
%   3. Outer angle P: L_o = Ka/s * T_rate, Ka chosen for PMo but at most
%      wc_rate/sep.
%   4. Optionally verify in quad_sim.slx (nonlinear).

o = struct('apply', false, 'verify', false, 'PM', 45, 'GM', 8, 'PMo', 60, 'wi', 6, ...
           'Td', [2e-3 25e-3], 'sep', 4, 'plot', false);
for k = 1:2:numel(varargin), o.(varargin{k}) = varargin{k+1}; end

here = fileparts(mfilename('fullpath'));
[quad, ideal] = local_params(here);

%% Operating point and plants
P = quad;
w0   = P.checks.w_hover;
Vbus = P.checks.Vbus_hover;
den  = P.motor.Kt^2/P.motor.R + 2*P.prop.kQ*w0;
Km   = (P.motor.Kt/P.motor.R)/den;
tau  = P.inertia.J_rotor/den;
kv   = Vbus/1000;                         % V per us of pulse
yarm = mean(abs(P.geom.r_motor(2,:)));
xarm = mean(abs(P.geom.r_motor(1,:)));
s = tf('s');
mot = kv*Km/(tau*s + 1);                  % rad/s rotor speed per us
G.roll  = mot*2*P.prop.kT*w0*4*yarm/(P.inertia.J(1,1)*s + P.drag.k_rot(1));
G.pitch = mot*2*P.prop.kT*w0*4*xarm/(P.inertia.J(2,2)*s + P.drag.k_rot(2));
G.yaw   = mot*4*(P.inertia.J_rotor*s + 2*P.prop.kQ*w0)/(P.inertia.J(3,3)*s + P.drag.k_rot(3));

% delays and IMU filter
qtmp.dt = 5e-4; qtmp.gnd = struct('k',2000,'c',50,'cxy',5,'klev',2,'clev',0.05);
qtmp.imu = struct('bias_g_pat',[0;0;0],'bias_a_pat',[0;0;0]); qtmp.start = 'air'; qtmp.alt0 = 1; qtmp.soc0 = 1;
qd = quad_sim_derive(qtmp, quad);
k = quad.imu.dlpf_cfg_used + 1;
Td = ideal.Ts/2 + qd.esc.Ts/2 + qd.esc.delay + qd.imu.nd_g*quad.imu.Ts;
Fimu = 1/(s/(2*pi*quad.imu.dlpf.gyro_bw(k)) + 1);
info.Td = Td; info.tau_m = tau; info.Km = Km; info.Vbus = Vbus;

axes_ = {'roll','pitch','yaw'};
gains = struct();
for a = 1:3
    ax = axes_{a};
    Gd = G.(ax)*Fimu; Gd.OutputDelay = Td;
    Gd = pade(Gd, 4);                         % rational, so isstable/feedback are exact
    % 1) pidtune seed
    [Cpid, ~] = pidtune(Gd, 'PIDF', pidtuneOptions('PhaseMargin', o.PM));
    w1 = ideal.(ax).w1; w2 = ideal.(ax).w2; wL = ideal.(ax).wL;
    % 2) grid over Kp (log) and Td = Kd/Kp: keep the highest crossover that
    %    meets PM >= o.PM and GM >= o.GM with a stable closed loop
    Cf = @(Kp, Tdv) Kp*(1 + o.wi/(s + wL) + Tdv*s/((1 + s/w1)*(1 + s/w2)));
    Tdg = linspace(o.Td(1), o.Td(2), 9);
    if strcmp(ax, 'yaw'), Tdg = 0; end
    best = struct('wc', 0, 'Kp', NaN, 'Td', NaN);
    for Tdv = Tdg
        for Kp = logspace(0.5, 2.7, 40)
            Lk = Cf(Kp, Tdv)*Gd;
            [Gm, Pm, ~, Wcp] = margin(Lk);
            if isempty(Wcp) || ~isfinite(Wcp) || Pm < o.PM || 20*log10(Gm) < o.GM, continue; end
            if ~isstable(feedback(Lk, 1)), continue; end
            if Wcp > best.wc, best = struct('wc', Wcp, 'Kp', Kp, 'Td', Tdv); end
        end
    end
    if isnan(best.Kp), error('tune_ideal: no %s gains meet PM %g / GM %g', ax, o.PM, o.GM); end
    C = Cf(best.Kp, best.Td);
    L = C*Gd;
    [Gm, Pm, ~, Wcp] = margin(L);
    g = struct();
    g.Kp = best.Kp; g.Ki = o.wi*g.Kp; g.Kd = best.Td*g.Kp;
    g.wc = Wcp; g.PM = Pm; g.GM_dB = 20*log10(Gm);
    g.pidtune = struct('Kp', Cpid.Kp, 'Ki', Cpid.Ki, 'Kd', Cpid.Kd, 'Tf', Cpid.Tf);
    % 3) outer angle loop
    g.Ka = 0; g.wc_outer = NaN; g.PM_outer = NaN;
    if a < 3
        T = feedback(L, 1);                       % rate_sp -> measured rate
        Ka = fzero(@(K) local_pm(K/s*T) - o.PMo, [0.5 200]);
        Ka = min(Ka, Wcp/o.sep);                  % keep the outer loop >= sep x slower
        [~, Pmo, ~, Wco] = margin(Ka/s*T);
        g.Ka = Ka; g.wc_outer = Wco; g.PM_outer = Pmo;
    end
    gains.(ax) = g;
    info.G.(ax) = Gd; info.C.(ax) = C;
end

%% report
fprintf('\nTUNE_IDEAL: hover op point w0 = %.0f rad/s, Vbus = %.2f V, tau_m = %.1f ms, loop delay %.2f ms + DLPF %d Hz\n', ...
    w0, Vbus, tau*1e3, Td*1e3, quad.imu.dlpf.gyro_bw(k));
fprintf('%-6s %8s %8s %8s %7s | %7s %6s %6s | %7s %7s\n', 'axis', 'Kp', 'Ki', 'Kd', 'Ka', 'wc', 'PM', 'GM', 'wc_ang', 'PM_ang');
for a = 1:3
    g = gains.(axes_{a});
    fprintf('%-6s %8.2f %8.1f %8.4f %7.2f | %7.1f %6.1f %6.1f | %7.2f %7.1f\n', axes_{a}, ...
        g.Kp, g.Ki, g.Kd, g.Ka, g.wc, g.PM, g.GM_dB, g.wc_outer, g.PM_outer);
end
fprintf('units: Kp us/(rad/s), Ki us/rad, Kd us/(rad/s^2), Ka 1/s, wc rad/s, PM deg, GM dB\n');

if o.plot
    figure('Name', 'tune_ideal loop gains');
    margin(info.C.roll*info.G.roll); hold on; margin(info.C.pitch*info.G.pitch); margin(info.C.yaw*info.G.yaw);
    legend('roll', 'pitch', 'yaw');
end

if o.apply || o.verify
    ideal_b = evalin('base', 'ideal');
    for a = 1:3
        ax = axes_{a};
        ideal_b.(ax).Kp = round(gains.(ax).Kp, 3, 'significant');
        ideal_b.(ax).Ki = round(gains.(ax).Ki, 3, 'significant');
        ideal_b.(ax).Kd = round(gains.(ax).Kd, 3, 'significant');
        ideal_b.(ax).Ka = round(gains.(ax).Ka, 3, 'significant');
    end
    assignin('base', 'ideal', ideal_b);
    fprintf('Gains written into base workspace struct `ideal`.\n');
end
if o.verify
    info.metrics = run_quad_scenarios({'hover','roll_step','pitch_step','yaw_step'});
end
end

%% ------------------------------------------------------------------------
function pm = local_pm(L)
[~, pm] = margin(L);
if isnan(pm) || isinf(pm), pm = 90; end
end

function [quad, ideal] = local_params(here)
run(fullfile(here, 'quad_params.m'));
run(fullfile(here, 'ctrl_ideal_params.m'));
end
