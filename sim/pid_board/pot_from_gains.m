function R = pot_from_gains(target, axisName, varargin)
%POT_FROM_GAINS Trimpot settings of PID_Board rev B that realise requested gains.
%   R = pot_from_gains(T, 'roll')                       T in board-stage units
%   R = pot_from_gains(T, 'roll', 'units', 'ideal')     T in ctrl_ideal units
%   R = pot_from_gains(..., 'pcb', pcbStruct)           default: base-workspace pcb
%   R = pot_from_gains(..., 'apply', true)              also write RV_* into base pcb
%
%   T fields (any subset; missing ones keep the current pot):
%     board units: Kp [V/V], Ki [1/s], Kd [s], Go [V/V]
%     ideal units: Kp [us/(rad/s)], Ki [us/rad], Kd [us/(rad/s^2)], Ka [1/s]
%                  (same as ideal.roll etc. in sim/quad_plant/ctrl_ideal_params.m,
%                   so R = pot_from_gains(ideal.roll, 'roll', 'units', 'ideal') works)
%   The ideal units are converted with gains_from_pots' loop factors
%   (g_fb*Kw*180/pi*K_rpy; Ka through Go*g_ang*Ktheta/Kw*g_sp/g_fb).
%
%   R: RV_P/RV_I/RV_D/RV_A [ohm] clipped to 0..RV_max, turns_* (3296W, 25 turns),
%   the achieved gains (gains_from_pots of the result), and R.clipped listing any
%   gain outside the pot range (achievable ranges at the default parts:
%   Kp 0.10-2.23, Ki 7.5-30 /s (yaw 8.7-67), Kd 4.7-14.7 ms, Go 0.5-3.0).
%   Note: Kd also moves the D roll-off f2 = 1/(2 pi (Rd_fixed+RV_D) Cf); w1 and the
%   I leak wL are fixed by parts (R209/C202, R208/C201) and cannot be tuned by a pot.

p = inputParser;
p.addParameter('units', 'board');
p.addParameter('pcb', []);
p.addParameter('apply', false);
p.parse(varargin{:});
o = p.Results;
pcbIn = o.pcb;
if isempty(pcbIn), pcbIn = evalin('base', 'pcb'); end
ax = pcbIn.(axisName);
G0 = gains_from_pots(pcbIn, axisName);
g0 = G0.(axisName);
T = target;
if strcmpi(o.units, 'ideal')
    s = g0.g_fb*pcbIn.scale.Kw*180/pi*pcbIn.ctrl.K_rpy(strcmp(axisName, {'roll', 'pitch', 'yaw'}));
    B = struct();
    if isfield(T, 'Kp'), B.Kp = T.Kp/s; end
    if isfield(T, 'Ki'), B.Ki = T.Ki/s; end
    if isfield(T, 'Kd') && T.Kd > 0, B.Kd = T.Kd/s; end
    if isfield(T, 'Ka') && T.Ka > 0 && isfield(g0, 'Go')
        B.Go = T.Ka/(g0.g_ang*(pcbIn.scale.Ktheta/pcbIn.scale.Kw)*(g0.g_sp/g0.g_fb));
    end
    T = B;
end
R = struct('clipped', {{}});
want = struct();
if isfield(T, 'Kp'), want.RV_P = T.Kp*ax.R_P_in - ax.R_P_fixed; end
if isfield(T, 'Ki'), want.RV_I = 1/(T.Ki*ax.CI) - ax.R_I_fixed; end
if isfield(T, 'Kd'), want.RV_D = T.Kd/ax.Cd - ax.Rd_fixed; end
if isfield(T, 'Go') && isfield(ax, 'RV_A'), want.RV_A = T.Go*ax.R_A_in - ax.R_A_fixed; end
f = fieldnames(want);
for k = 1:numel(f)
    rmax = ax.([f{k} '_max']);
    v = want.(f{k});
    if v < 0 || v > rmax
        R.clipped{end+1} = sprintf('%s: wanted %.1fk, pot range 0-%.0fk', f{k}, v/1e3, rmax/1e3);
    end
    ax.(f{k}) = min(max(v, 0), rmax);
end
pcbNew = pcbIn;
pcbNew.(axisName) = ax;
Gn = gains_from_pots(pcbNew, axisName);
g = Gn.(axisName);
R.RV_P = ax.RV_P; R.RV_I = ax.RV_I; R.RV_D = ax.RV_D;
R.turns_P = g.turns_P; R.turns_I = g.turns_I; R.turns_D = g.turns_D;
if isfield(ax, 'RV_A'), R.RV_A = ax.RV_A; R.turns_A = g.turns_A; end
R.gains = g;
if o.apply
    assignin('base', 'pcb', pcbNew);
end
if nargout == 0 || o.apply
    fprintf('%s: RV_P %.1fk (%.1f turns)  RV_I %.1fk (%.1f)  RV_D %.1fk (%.1f)', axisName, ...
        R.RV_P/1e3, R.turns_P, R.RV_I/1e3, R.turns_I, R.RV_D/1e3, R.turns_D);
    if isfield(R, 'RV_A'), fprintf('  RV_A %.1fk (%.1f)', R.RV_A/1e3, R.turns_A); end
    fprintf('\n   -> Kp %.3f  Ki %.2f /s  Kd %.2f ms (f2 %.0f Hz)', g.Kp, g.Ki, 1e3*g.Kd, g.f2);
    if isfield(g, 'Go'), fprintf('  Go %.2f (Ka %.2f /s)', g.Go, g.Ka); end
    fprintf('\n');
    for k = 1:numel(R.clipped), fprintf('   CLIPPED %s\n', R.clipped{k}); end
end
if nargout == 0, clear R, end
end
