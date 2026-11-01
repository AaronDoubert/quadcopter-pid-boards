function G = gains_from_pots(pcbIn, axes, turnsIn)
%GAINS_FROM_POTS Equivalent controller gains of PID_Board rev B for given trimpot settings.
%   G = gains_from_pots()               all axes, pots from base-workspace pcb
%   G = gains_from_pots(pcb)            all axes of a given pcb struct
%   G = gains_from_pots(pcb, 'roll')    one axis ('roll' | 'pitch' | 'yaw' or a cell list)
%   G = gains_from_pots(pcb, 'roll', T) pots given in 3296W TURNS instead: T is a struct with
%                                       any of P, I, D, A (0..25 turns, 25 = full resistance)
%   Without an output argument a table is printed.
%
%   Board-stage gains (voltages re VG, schematic formulas of pid_board_spec.md section 4):
%     Kp  = (R_P_fixed + RV_P)/R_P_in                 [V/V]
%     Ki  = 1/((R_I_fixed + RV_I)*CI)                 [1/s]   I_OUT/E = -Ki/(s + wL)
%     wL  = 1/(RL*CI)  (I leak, rad/s),  I_dc = Ki/wL (DC gain of the leaky I)
%     Kd  = (Rd_fixed + RV_D)*Cd                      [s]     D_OUT/E = -Kd s/((1+s/w1)(1+s/w2))
%     w1  = 1/(Rs*Cd) (fixed, 59 Hz),  w2 = 1/((Rd_fixed + RV_D)*Cf) (108-339 Hz)
%     Go  = (R_A_fixed + RV_A)/R_A_in                 [V/V]   (roll/pitch angle P)
%   Loop-level equivalents (same units as ctrl_ideal_params.m, sim/quad_plant):
%     e_rate [rad/s] -> E [V]: g_fb*Kw_rad,  Kw_rad = Kw*180/pi;  U [V] -> us: K_rpy (mixer)
%     Kp_us = K_rpy*Kp*g_fb*Kw_rad     [us/(rad/s)]
%     Ki_us = K_rpy*Ki*g_fb*Kw_rad     [us/rad]      (leak wL unchanged)
%     Kd_us = K_rpy*Kd*g_fb*Kw_rad     [us/(rad/s^2)]
%     Ka    = Go*g_ang*(Ktheta/Kw)*(g_sp/g_fb)  [1/s] rate setpoint per angle error
%     I_max_us = K_rpy*V_iclamp,  u_max_us = K_rpy*|V_rail_lo|
%   (ctrl_ideal_params.m omits the 0.909 g_fb leg gain in Kp/Ki/Kd; include it when comparing.)
%
%   See also POT_FROM_GAINS.

if nargin < 1 || isempty(pcbIn), pcbIn = evalin('base', 'pcb'); end
if nargin < 2 || isempty(axes), axes = {'roll', 'pitch', 'yaw'}; end
if ischar(axes), axes = {axes}; end
TURNS = 25;   % Bourns 3296W: 25 turns end to end
G = struct();
for k = 1:numel(axes)
    an = axes{k};
    ax = pcbIn.(an);
    if nargin >= 3 && ~isempty(turnsIn)
        f = fieldnames(turnsIn);
        for j = 1:numel(f)
            ax.(['RV_' f{j}]) = min(max(turnsIn.(f{j}), 0), TURNS)/TURNS*ax.(['RV_' f{j} '_max']);
        end
    end
    g = struct();
    g.RV_P = ax.RV_P; g.RV_I = ax.RV_I; g.RV_D = ax.RV_D;
    g.turns_P = ax.RV_P/ax.RV_P_max*TURNS;
    g.turns_I = ax.RV_I/ax.RV_I_max*TURNS;
    g.turns_D = ax.RV_D/ax.RV_D_max*TURNS;
    g.Kp = (ax.R_P_fixed + ax.RV_P)/ax.R_P_in;
    g.Ki = 1/((ax.R_I_fixed + ax.RV_I)*ax.CI);
    g.wL = 1/(ax.RL*ax.CI);
    g.I_dc = g.Ki/g.wL;
    g.V_iclamp = ax.V_iclamp;
    Rd = ax.Rd_fixed + ax.RV_D;
    g.Kd = Rd*ax.Cd;
    g.w1 = 1/(ax.Rs*ax.Cd);  g.f1 = g.w1/(2*pi);
    g.w2 = 1/(Rd*ax.Cf);     g.f2 = g.w2/(2*pi);
    g.D_hf = Rd/ax.Rs;
    g.en = [ax.en_P ax.en_I ax.en_D];
    rsp = (ax.JP_SP == 1)*ax.R_src_sp; rfb = (ax.JP_FB == 1)*ax.R_src_fb;
    g.g_sp = ax.R_dsp_ref/(ax.R_dsp_in + rsp + ax.R_dsp_ref)*(1 + ax.R_dfb_f/(ax.R_dfb_in + rfb));
    g.g_fb = ax.R_dfb_f/(ax.R_dfb_in + ax.R_src_fb);
    iAx = find(strcmp(an, {'roll', 'pitch', 'yaw'}));
    Kmix = pcbIn.ctrl.K_rpy(iAx);
    Kw_rad = pcbIn.scale.Kw*180/pi;
    s = g.g_fb*Kw_rad*Kmix;
    g.ideal.Kp = g.Kp*s;
    g.ideal.Ki = g.Ki*s;
    g.ideal.Kd = g.Kd*s*g.en(3);
    g.ideal.wL = g.wL;
    g.ideal.w1 = g.w1; g.ideal.w2 = g.w2;
    g.ideal.I_max = Kmix*ax.V_iclamp;
    g.ideal.u_max = Kmix*abs(pcbIn.V_rail_lo);
    if isfield(ax, 'RV_A')
        g.RV_A = ax.RV_A;
        g.turns_A = ax.RV_A/ax.RV_A_max*TURNS;
        g.Go = (ax.R_A_fixed + ax.RV_A)/ax.R_A_in;
        g.g_ang = ax.R_a_f/(ax.R_acmd_in + ax.R_src_ang);
        g.Ka = g.Go*g.g_ang*(pcbIn.scale.Ktheta/pcbIn.scale.Kw)*(g.g_sp/g.g_fb);
        g.ideal.Ka = g.Ka;
    else
        g.ideal.Ka = 0;
    end
    G.(an) = g;
end
if nargout == 0
    fprintf('%-6s %8s %6s %7s %6s %8s %6s %7s %7s %6s %6s | %9s %8s %9s %6s\n', 'axis', 'RV_P', 'Kp', ...
        'RV_I', 'Ki', 'RV_D', 'Kd ms', 'f2 Hz', 'RV_A', 'Go', 'Ka', 'Kp us/r/s', 'Ki us/r', 'Kd us/r/s2', 'Ka');
    for k = 1:numel(axes)
        g = G.(axes{k});
        if isfield(g, 'Go'), ra = sprintf('%6.1fk', g.RV_A/1e3); go = g.Go; ka = g.Ka; else, ra = '    -'; go = NaN; ka = NaN; end
        fprintf('%-6s %7.1fk %6.3f %6.1fk %6.2f %7.1fk %6.2f %7.1f %7s %6.2f %6.2f | %9.1f %8.1f %9.3f %6.2f\n', ...
            axes{k}, g.RV_P/1e3, g.Kp, g.RV_I/1e3, g.Ki, g.RV_D/1e3, 1e3*g.Kd, g.f2, ra, go, ka, ...
            g.ideal.Kp, g.ideal.Ki, g.ideal.Kd, g.ideal.Ka);
    end
    clear G
end
end
