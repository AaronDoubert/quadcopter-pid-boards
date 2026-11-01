function FF = first_flight_gains(varargin)
%FIRST_FLIGHT_GAINS Conservative ("first flight") trimpot settings for the PID board.
%   FF = first_flight_gains()            design, print the table, write first_flight_gains.md
%   FF = first_flight_gains('apply', true)  also load the result into base `ideal` and `pcb`
%
%   The search runs in board units: per-axis mixer scale Kmix (us/V, a
%   control_io firmware constant, pcb.ctrl.K_rpy) and the trimpots RV_P,
%   RV_I, RV_D (and RV_A for the angle loop). Every candidate is turned into
%   the equivalent board-shaped controller
%       C = Kmix*Kw*[Kp_b + Ki_b/(s + wL) + Kd_b*s/((1+s/w1)(1+s/w2))]
%   with w2 set by RV_D, and checked on the hand-linearised hover plant from
%   tune_ideal.m plus the board's own extra delay (control_io ZOH and compute).
%
%   "Wary" means:
%     - nominal rate loop PM >= 55 deg and GM >= 10 dB (tuned set: 45 / 8)
%     - still PM >= 35 deg, GM >= 6 dB with inertia x1.7 (battery on top),
%       inertia x0.7, and 2 ms of extra unmodelled delay
%     - trimpots between 10 % and 90 % of travel (room to tune both ways)
%     - integrator clamp Kmix*0.75 V >= 75 us (enough to trim a ~20 mm CG offset)
%     - angle loop PM >= 70 deg and at least 5x slower than the rate loop
%   Among the candidates that pass, the one with the highest nominal
%   crossover is kept (ties: pots closest to mid travel).

o = struct('apply', false, 'PM', 55, 'GM', 10, 'PMr', 35, 'GMr', 6, 'Jhi', 1.7, 'Jlo', 0.7, ...
           'dly', 2e-3, 'travel', [0.1 0.9], 'Imin_us', 75, 'PMo', 70, 'sep', 5);
for k = 1:2:numel(varargin), o.(varargin{k}) = varargin{k+1}; end
here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..');
run(fullfile(root, 'pid_board', 'pid_board_params.m'));     % -> pcb (this function's workspace)
addpath(fullfile(root, 'quad_plant'));
evalin('base', sprintf('run(''%s'')', fullfile(root, 'quad_plant', 'quad_sim_params.m')));
quad = evalin('base', 'quad');
[~, info] = evalc('local_tune_info()');                      % plants from tune_ideal (quiet)

s = tf('s');
Kw = pcb.scale.Kw*180/pi;                                    % V per rad/s
Tbrd = pcb.ctrl.Ts/2 + pcb.ctrl.t_compute;                   % control_io ZOH + compute
Dbrd = pade(exp(-Tbrd*s), 2);
Dx   = pade(exp(-o.dly*s), 2);
fr = linspace(o.travel(1), o.travel(2), 9);
Kmixg = [50 75 100 125 150 200 250 300];
axs = {'roll', 'pitch', 'yaw'};
FF = struct('opts', o, 'Kw', Kw, 'Tbrd', Tbrd);

for a = 1:3
    ax = axs{a}; c = pcb.(ax);
    G = info.G.(ax)*Dbrd;
    Jaxis = quad.inertia.J(a, a);
    % plant with scaled inertia: G ~ 1/(J s + k): rescale that factor
    k_rot = quad.drag.k_rot(a);
    Ghi = G*(Jaxis*s + k_rot)/(o.Jhi*Jaxis*s + k_rot);
    Glo = G*(Jaxis*s + k_rot)/(o.Jlo*Jaxis*s + k_rot);
    Gdl = G*Dx;
    w1 = 1/(c.Rs*c.Cd); wL = 1/(c.RL*c.CI);
    useD = c.en_D;
    best = struct('wc', 0);
    for Kmix = Kmixg
        if Kmix*c.V_iclamp < o.Imin_us, continue; end
        for fP = fr
            RVP = fP*c.RV_P_max; Kpb = (c.R_P_fixed + RVP)/c.R_P_in;
            for fI = fr
                RVI = fI*c.RV_I_max; Kib = 1/((c.R_I_fixed + RVI)*c.CI);
                fDs = fr; if ~useD, fDs = 0.5; end
                for fD = fDs
                    RVD = fD*c.RV_D_max; Rd = c.Rd_fixed + RVD;
                    Kdb = useD*Rd*c.Cd; w2 = 1/(Rd*c.Cf);
                    C = Kmix*Kw*(Kpb + Kib/(s + wL) + Kdb*s/((1 + s/w1)*(1 + s/w2)));
                    [ok, wc, pm, gm] = local_check(C*G, o.PM, o.GM);
                    if ~ok || wc < best.wc, continue; end
                    if ~local_check(C*Ghi, o.PMr, o.GMr) || ~local_check(C*Glo, o.PMr, o.GMr) ...
                            || ~local_check(C*Gdl, o.PMr, o.GMr), continue; end
                    mid = abs(fP - 0.5) + abs(fI - 0.5) + useD*abs(fD - 0.5);
                    if wc > best.wc*1.02 || (wc > best.wc*0.98 && mid < best.mid)
                        best = struct('wc', wc, 'PM', pm, 'GM_dB', gm, 'Kmix', Kmix, ...
                            'RV_P', RVP, 'RV_I', RVI, 'RV_D', RVD*useD, 'fP', fP, 'fI', fI, 'fD', fD*useD, ...
                            'Kp_b', Kpb, 'Ki_b', Kib, 'Kd_b', Kdb, 'w2', w2, 'mid', mid, 'C', C);
                    end
                end
            end
        end
    end
    if best.wc == 0, error('first_flight_gains: no %s setting meets the margins', ax); end
    L = best.C*G;
    [~, best.PM_Jhi] = local_margins(best.C*Ghi); [~, best.PM_Jlo] = local_margins(best.C*Glo);
    [~, best.PM_dly] = local_margins(best.C*Gdl);
    % equivalent ideal-controller gains (ctrl_ideal units: us/(rad/s), us/rad, us/(rad/s^2))
    best.Kp = best.Kmix*Kw*best.Kp_b; best.Ki = best.Kmix*Kw*best.Ki_b; best.Kd = best.Kmix*Kw*best.Kd_b;
    best.I_max_us = best.Kmix*c.V_iclamp;
    % outer angle loop: Go = (10k + RV_A)/20k
    best.Ka = 0; best.RV_A = NaN; best.Go = NaN; best.PM_outer = NaN; best.wc_outer = NaN;
    if a < 3
        T = feedback(L, 1);
        kgo = c.g_ang_meas*(pcb.scale.Ktheta/pcb.scale.Kw)*(c.g_sp/c.g_fb);   % Ka per unit Go
        bo = struct('wc', 0);
        for fA = linspace(o.travel(1), o.travel(2), 33)
            RVA = fA*c.RV_A_max; Go = (c.R_A_fixed + RVA)/c.R_A_in; Ka = Go*kgo;
            [okA, wco, pmo] = local_check(Ka/s*T, o.PMo, 10);
            if okA && wco <= best.wc/o.sep && wco > bo.wc
                bo = struct('wc', wco, 'PM', pmo, 'RV_A', RVA, 'fA', fA, 'Go', Go, 'Ka', Ka);
            end
        end
        if bo.wc == 0, error('first_flight_gains: no %s angle setting meets PMo/sep', ax); end
        best.Ka = bo.Ka; best.RV_A = bo.RV_A; best.Go = bo.Go; best.fA = bo.fA;
        best.PM_outer = bo.PM; best.wc_outer = bo.wc;
    end
    best = rmfield(best, {'C', 'mid'});
    FF.(ax) = best;
end

%% Report
lines = local_report(FF, pcb);
fprintf('%s\n', lines{:});
fid = fopen(fullfile(here, 'first_flight_gains.md'), 'w');
fprintf(fid, '%s\n', lines{:}); fclose(fid);

if o.apply
    ideal = evalin('base', 'ideal');
    for a = 1:3
        ax = axs{a}; b = FF.(ax);
        ideal.(ax).Kp = b.Kp; ideal.(ax).Ki = b.Ki; ideal.(ax).Kd = b.Kd;
        ideal.(ax).w2 = b.w2; ideal.(ax).I_max = b.I_max_us;
        if a < 3, ideal.(ax).Ka = b.Ka; end
        pcb.(ax).RV_P = b.RV_P; pcb.(ax).RV_I = b.RV_I; pcb.(ax).RV_D = max(b.RV_D, pcb.(ax).RV_D*(a == 3));
        if a < 3, pcb.(ax).RV_A = b.RV_A; end
        pcb.ctrl.K_rpy(a) = b.Kmix;
    end
    assignin('base', 'ideal', ideal); assignin('base', 'pcb', pcb);
end
end

%% ------------------------------------------------------------------------
function info = local_tune_info()
[~, info] = tune_ideal();
end

function [ok, wc, pm, gm] = local_check(L, PMmin, GMmin)
[wc, pm, gm] = local_margins(L);
ok = ~isnan(wc) && pm >= PMmin && gm >= GMmin && isstable(feedback(L, 1));
end

function [wc, pm, gm] = local_margins(L)
[Gm, Pm, ~, Wcp] = margin(L);
if isempty(Wcp) || ~isfinite(Wcp), wc = NaN; pm = NaN; gm = NaN; return; end
wc = Wcp; pm = Pm; gm = 20*log10(Gm);
end

function L = local_report(FF, pcb)
o = FF.opts;
turns = @(f) 25*f;                         % 3296W: 25 turns end to end
L = {};
L{end+1} = '# First-flight gains (conservative)';
L{end+1} = '';
L{end+1} = sprintf(['Generated by `sim/gains/first_flight_gains.m` on %s. Rate loops: PM >= %g deg, GM >= %g dB nominal; ', ...
    'PM >= %g deg / GM >= %g dB with inertia x%g and x%g and +%g ms delay. Pots kept within %g-%g %% of travel.'], ...
    datestr(now, 'yyyy-mm-dd'), o.PM, o.GM, o.PMr, o.GMr, o.Jhi, o.Jlo, 1e3*o.dly, 100*o.travel(1), 100*o.travel(2));
L{end+1} = '';
L{end+1} = '## Trimpot settings';
L{end+1} = '';
L{end+1} = '| Axis | Pot | Set to | Travel | Turns from 0 Ω end | Board gain |';
L{end+1} = '|---|---|---|---|---|---|';
names = struct('roll', {{'RV201','RV202','RV203','RV204'}}, 'pitch', {{'RV301','RV302','RV303','RV304'}}, ...
               'yaw', {{'RV401','RV402','RV403',''}});
axs = {'roll','pitch','yaw'};
for a = 1:3
    ax = axs{a}; b = FF.(ax); n = names.(ax); c = pcb.(ax);
    L{end+1} = sprintf('| %s | %s (Kp) | %.1f kΩ | %.0f %% | %.1f | Kp = %.2f |', ax, n{1}, b.RV_P/1e3, 100*b.fP, turns(b.fP), b.Kp_b);
    L{end+1} = sprintf('| | %s (Ki) | %.1f kΩ | %.0f %% | %.1f | Ki = %.1f /s |', n{2}, b.RV_I/1e3, 100*b.fI, turns(b.fI), b.Ki_b);
    if a < 3
        L{end+1} = sprintf('| | %s (Kd) | %.1f kΩ | %.0f %% | %.1f | Kd = %.1f ms |', n{3}, b.RV_D/1e3, 100*b.fD, turns(b.fD), 1e3*b.Kd_b);
        L{end+1} = sprintf('| | %s (Go) | %.1f kΩ | %.0f %% | %.1f | Go = %.2f |', n{4}, b.RV_A/1e3, 100*b.fA, turns(b.fA), b.Go);
    else
        L{end+1} = sprintf('| | %s (Kd) | not summed on yaw (R413 DNP) | | | |', n{3});
    end
end
L{end+1} = '';
L{end+1} = sprintf(['Mixer scale for the control_io firmware (`pcb.ctrl.K_rpy`, us of ESC pulse per volt of U): ', ...
    'roll %g, pitch %g, yaw %g us/V.'], FF.roll.Kmix, FF.pitch.Kmix, FF.yaw.Kmix);
L{end+1} = '';
L{end+1} = '## What that means as an ideal PID';
L{end+1} = '';
L{end+1} = '| Axis | Ka (1/s) | Kp us/(rad/s) | Ki us/rad | Kd us/(rad/s²) | I clamp us | rate wc rad/s | PM | GM dB | PM J×1.7 | PM J×0.7 | PM +2 ms | angle PM |';
L{end+1} = '|---|---|---|---|---|---|---|---|---|---|---|---|---|';
for a = 1:3
    ax = axs{a}; b = FF.(ax);
    L{end+1} = sprintf('| %s | %.1f | %.1f | %.0f | %.2f | %.0f | %.0f | %.0f° | %.1f | %.0f° | %.0f° | %.0f° | %s |', ax, b.Ka, b.Kp, b.Ki, b.Kd, ...
        b.I_max_us, b.wc, b.PM, b.GM_dB, b.PM_Jhi, b.PM_Jlo, b.PM_dly, local_s(b.PM_outer));
end
L{end+1} = '';
L{end+1} = ['Turns assume a linear 25-turn 3296W measured from the end that gives 0 Ω in circuit; ', ...
    'check each pot with a meter (power off) before the first flight rather than counting turns.'];
end

function t = local_s(v)
if isnan(v), t = '-'; else, t = sprintf('%.0f°', v); end
end
