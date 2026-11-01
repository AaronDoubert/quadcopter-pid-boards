%SWEEP_ROLL_POTS Best roll trimpot settings over motor lag and plant scale.
%   For each motor time constant tau_m and plant scale (plant gain relative
%   to 45.5 /s, set by the mixer scale KF), the script:
%     1. linearises U -> FB from roll_axis.slx (loop open at U),
%     2. grid-searches RV_P, RV_I, RV_D over 0-100k for the highest phase
%        margin with at least min_gm_db of gain margin (the briefing's
%        criterion), using the same P/I/D transfer functions as the model,
%     3. checks the pick on the full model (exact loop transfer) and runs a
%        nonlinear 100 mV rate step to report rise time and overshoot.
%   Prints a pot-setting table and writes sweep_roll_pots.csv.
%
%   Settings below: use_board_gains = false reproduces the briefing's table.

here = fileparts(mfilename('fullpath'));
addpath(here);
mdl = 'roll_axis';
if ~isfile(fullfile(here, [mdl '.slx'])), build_roll_model; end
roll_params;
load_system(mdl);

%% Sweep settings
tau_list   = [20 40 60 80]*1e-3;     % s, motor + prop + ESC lag
scale_list = [0.5 1 1.5 2];          % plant gain / 45.5 /s
rv_step    = 2.5e3;                  % ohm, pot search resolution
min_gm_db  = 6;
use_board_gains = true;              % false: ideal 1.0 / 1.0 diff-amp legs
min_fc_hz  = 0;                      % optional crossover floor, Hz

if ~use_board_gains
    g_sp = 1;
    g_fb = 1;
end
bench_mode    = 0;
angle_loop_on = 0;

%% Plant + sensor response from the model, U -> FB with the loop open at U
[sl, pt] = roll_linearizer(mdl);
[TM, SC] = ndgrid(tau_list, scale_list);
sl.Parameters = struct('Name', {'tau_m', 'KF'}, 'Value', {TM, SC*KF_nom});
G  = getIOTransfer(sl, pt.U, pt.FB, pt.U);
f  = logspace(-1, log10(500), 1500);
s  = 1j*2*pi*f(:);
Gw = reshape(freqresp(G, 2*pi*f), numel(f), []);   % nf x ncases

% Controller C(s) = U/E, the same expressions as the PID cell blocks
ctrl = @(RVP, RVI, RVD) en_P*(4.7e3 + RVP)/47e3 ...
    + en_I./((RI_fixed + RVI)*CI)./(s + 1/(RL*CI)) ...
    + en_D*(Rd_fixed + RVD)*Cd.*s./((1 + s*Rs*Cd).*(1 + s.*(Rd_fixed + RVD)*Cf));

% Sanity check: C*g_fb*G must equal the model's own loop transfer
Lm  = reshape(freqresp(getLoopTransfer(sl, pt.U, -1), 2*pi*f), numel(f), []);
Lc  = g_fb*ctrl(RV_P, RV_I, RV_D).*Gw;
err = max(abs(Lc(:) - Lm(:))./abs(Lm(:)));
fprintf('Controller formula vs model loop transfer: max rel. error %.1e\n', err);
assert(err < 1e-6, 'Controller formula does not match the model.');

%% Grid search
rv = 0:rv_step:100e3;
nc = numel(TM);
best = repmat(struct('pm', -Inf, 'gm', NaN, 'fc', NaN, 'rv', [NaN NaN NaN]), nc, 1);
Pw = en_P*(4.7e3 + rv)/47e3;                        % 1 x nP
for iI = 1:numel(rv)
    Iw = en_I/((RI_fixed + rv(iI))*CI)./(s + 1/(RL*CI));
    for iD = 1:numel(rv)
        Rd = Rd_fixed + rv(iD);
        Cw = Pw + Iw + en_D*Rd*Cd*s./((1 + s*Rs*Cd).*(1 + s*Rd*Cf));   % nf x nP
        for c = 1:nc
            [fc, pm, gm] = loop_margins(g_fb*Cw.*Gw(:, c), f);
            pm(gm < min_gm_db | ~(fc >= min_fc_hz)) = -Inf;
            [pmax, iP] = max(pm);
            if pmax > best(c).pm
                best(c) = struct('pm', pmax, 'gm', gm(iP), 'fc', fc(iP), ...
                                 'rv', [rv(iP) rv(iI) rv(iD)]);
            end
        end
    end
end

%% Check each pick on the full model: exact margins and a nonlinear step
fgrid = logspace(-2, log10(500), 4000);
in = Simulink.SimulationInput.empty;
for c = 1:nc
    assignin('base', 'tau_m', TM(c));
    assignin('base', 'KF', SC(c)*KF_nom);
    assignin('base', 'RV_P', best(c).rv(1));
    assignin('base', 'RV_I', best(c).rv(2));
    assignin('base', 'RV_D', best(c).rv(3));
    sl.Parameters = [];
    refresh(sl);
    Lw = squeeze(freqresp(getLoopTransfer(sl, pt.U, -1), 2*pi*fgrid));
    [best(c).fc, best(c).pm, best(c).gm] = loop_margins(Lw, fgrid);

    in(c) = Simulink.SimulationInput(mdl) ...
        .setVariable('tau_m', TM(c)).setVariable('KF', SC(c)*KF_nom) ...
        .setVariable('RV_P', best(c).rv(1)).setVariable('RV_I', best(c).rv(2)) ...
        .setVariable('RV_D', best(c).rv(3)).setVariable('g_sp', g_sp).setVariable('g_fb', g_fb) ...
        .setVariable('bench_mode', 0).setVariable('angle_loop_on', 0) ...
        .setVariable('rate_sp_V', 0.1).setVariable('t_stop', 1.0);
end
out = sim(in, 'ShowProgress', 'off');

%% Report
fprintf('\nRoll pot settings: highest PM with GM >= %g dB', min_gm_db);
if min_fc_hz > 0, fprintf(' and fc >= %g Hz', min_fc_hz); end
fprintf('\nDiff-amp legs SP %.3f / FB %.3f, pot grid %.1fk, %s step for OS/rise\n', ...
    g_sp, g_fb, rv_step/1e3, '100 mV rate');
hdr = sprintf('%6s %6s %7s | %6s %6s %6s | %-14s %-15s %-15s | %6s %7s %6s | %s', ...
    'tau_m', 'scale', 'KF', 'PM', 'GM', 'fc', 'RV_P (Kp)', 'RV_I (Ki)', 'RV_D (Kd)', ...
    'OS', 'rise', 'Umax', 'score');
fprintf('%s\n%s\n', hdr, repmat('-', 1, numel(hdr)));
fprintf('%6s %6s %7s | %6s %6s %6s | %-14s %-15s %-15s | %6s %7s %6s |\n', ...
    'ms', '', 'N/V', 'deg', 'dB', 'Hz', '', '', '', '%', 'ms', 'V');
rows = cell(nc, 1);
for c = 1:nc
    r  = best(c).rv;
    Kp = (4.7e3 + r(1))/47e3;
    Ki = 1/((RI_fixed + r(2))*CI);
    Kd = (Rd_fixed + r(3))*Cd;

    ts = out(c).log_omega_dps;
    t  = ts.Time;
    y  = squeeze(ts.Data);
    keep = t >= t_step;
    yf = 0.1*g_sp/g_fb/Kw*180/pi;                 % deg/s, integrating plant
    S  = stepinfo(y(keep), t(keep) - t_step, yf, 'RiseTimeLimits', [0.1 0.9]);
    Umax = max(abs(squeeze(out(c).log_U.Data)));

    score = {};
    if best(c).fc < 5 || best(c).fc > 10, score{end+1} = 'fc'; end %#ok<SAGROW>
    if best(c).pm < 45, score{end+1} = 'PM'; end %#ok<SAGROW>
    if best(c).gm < 6, score{end+1} = 'GM'; end %#ok<SAGROW>
    if S.Overshoot > 15, score{end+1} = 'OS'; end %#ok<SAGROW>
    if isempty(score), scoreStr = 'meets all'; else, scoreStr = ['misses ' strjoin(score, ',')]; end

    fprintf('%6.0f %6.2g %7.2f | %6.1f %6.1f %6.2f | %5.1fk (%4.2f) %5.1fk (%4.1f/s) %5.1fk (%4.1fms) | %6.1f %7.1f %6.2f | %s\n', ...
        1e3*TM(c), SC(c), SC(c)*KF_nom, best(c).pm, best(c).gm, best(c).fc, ...
        r(1)/1e3, Kp, r(2)/1e3, Ki, r(3)/1e3, 1e3*Kd, S.Overshoot, 1e3*S.RiseTime, Umax, scoreStr);
    rows{c} = {1e3*TM(c), SC(c), SC(c)*KF_nom, best(c).pm, best(c).gm, best(c).fc, ...
               r(1), Kp, r(2), Ki, r(3), Kd, S.Overshoot, S.RiseTime, Umax};
end
fprintf('score: rate crossover 5-10 Hz, PM >= 45 deg, GM >= 6 dB, step overshoot < 15%%\n');

T = cell2table(vertcat(rows{:}), 'VariableNames', {'tau_m_ms', 'plant_scale', 'KF_N_per_V', ...
    'PM_deg', 'GM_dB', 'fc_Hz', 'RV_P_ohm', 'Kp', 'RV_I_ohm', 'Ki_per_s', 'RV_D_ohm', 'Kd_s', ...
    'overshoot_pct', 'rise_s', 'Umax_V'});
writetable(T, fullfile(here, 'sweep_roll_pots.csv'));
fprintf('Wrote %s\n', fullfile(here, 'sweep_roll_pots.csv'));
roll_params;
