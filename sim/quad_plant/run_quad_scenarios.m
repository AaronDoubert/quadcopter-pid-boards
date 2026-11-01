function M = run_quad_scenarios(list, varargin)
%RUN_QUAD_SCENARIOS Run quad_sim.slx scenarios, print metrics, make summary plots.
%   M = run_quad_scenarios()                      all scenarios
%   M = run_quad_scenarios({'hover','roll_step'}) a subset
%   M = run_quad_scenarios(list, 'plot', false, 'save', true)
%   Scenarios (quad_scenario.m): hover, roll_step, pitch_step, yaw_step,
%   throttle_punch, takeoff, battery_drain.
%   Uses the controller selected by qsim.ctrl (default 1 = ideal). Needs
%   quad_sim_params to have run (it is run if quad/qsim/ideal are missing).
%   Plots are saved to results/ next to this file when 'save' is true.
%   M.(scenario) holds the metrics and the logged signals (M.(sc).sig).

o = struct('plot', true, 'save', true);
for k = 1:2:numel(varargin), o.(varargin{k}) = varargin{k+1}; end
here = fileparts(mfilename('fullpath'));
if ~all(cellfun(@(v) evalin('base', sprintf('exist(''%s'',''var'')', v)), {'quad','qsim','ideal','CtrlIn'}))
    evalin('base', sprintf('run(''%s'')', fullfile(here, 'quad_sim_params.m')));
end
if nargin < 1 || isempty(list)
    list = {'hover','roll_step','pitch_step','yaw_step','throttle_punch','takeoff','battery_drain'};
end
if ischar(list), list = {list}; end
quad  = evalin('base', 'quad');
ideal = evalin('base', 'ideal');
qsim0 = evalin('base', 'qsim');
cleanup = onCleanup(@() assignin('base', 'qsim', qsim0));
load_system(fullfile(here, 'quad_sim.slx'));

M = struct();
for i = 1:numel(list)
    sc = list{i};
    qs = quad_scenario(qsim0, quad, ideal, sc);
    assignin('base', 'qsim', qs);
    t0 = tic;
    out = sim('quad_sim');
    wall = toc(t0);
    S = local_signals(out);
    r = struct('wall_s', wall, 'StopTime', qs.StopTime);
    switch sc
        case 'hover'
            k = S.t > 5;
            r.esc_us_mean = mean(S.esc_us(k,:), 'all');
            r.duty_mean   = mean(S.duty(k,:), 'all');
            r.vbat_mean   = mean(S.vbat_V(k));
            r.duty_check  = quad.checks.hover_duty;                   % at V_nominal (15.4 V OCV)
            r.duty_check_at_vbat = quad.checks.Vm_hover/r.vbat_mean;  % same check at the simulated bus voltage
            r.rpm_mean    = mean(S.motor_rpm(k,:), 'all');
            r.rpm_check   = quad.checks.w_hover*60/(2*pi);
            r.I_motor_mean = mean(S.I_motor(k,:), 'all');
            r.I_check     = quad.checks.I_hover;
            r.att_rms_deg = rad2deg(sqrt(mean(S.euler(k,1:2).^2)));
            r.alt_err_rms = sqrt(mean((-S.Xe(k,3) - qs.alt0).^2));
            r.xy_drift_m  = norm(S.Xe(end,1:2));
        case {'roll_step','pitch_step','yaw_step'}
            ax = find(strcmp(sc, {'roll_step','pitch_step','yaw_step'}));
            seg = qs.cmd(3,1) <= S.t & S.t < qs.cmd(4,1);
            if ax == 1, y = S.euler(:,1); ref = qs.cmd(3,2)*ideal.max_angle;  unit = 'deg';
            elseif ax == 2, y = S.euler(:,2); ref = -qs.cmd(3,3)*ideal.max_angle; unit = 'deg';
            else, y = S.pqr(:,3); ref = qs.cmd(3,4)*ideal.max_yaw_rate; unit = 'deg/s';
            end
            ts = S.t(seg) - S.t(find(seg, 1)); ys = y(seg);
            si = stepinfo(ys, ts, ref, y(find(seg,1)-1), 'SettlingTimeThreshold', 0.05);
            r.unit = unit;
            r.ref = rad2deg(ref);
            r.overshoot_pct = si.Overshoot;
            r.rise_10_90_s  = si.RiseTime;
            r.settle_5pct_s = si.SettlingTime;
            r.peak = rad2deg(si.Peak);
            r.final = rad2deg(mean(ys(ts > ts(end) - 0.3)));
            r.ss_err_pct = 100*(r.final - r.ref)/r.ref;
            r.max_esc_us = max(S.esc_us(seg,:), [], 'all');
            r.min_esc_us = min(S.esc_us(seg,:), [], 'all');
            % inner rate loop of the same axis, from the controller debug vector
            if isfield(S, 'ctrl_dbg')
                r.rate_err_rms_dps = rad2deg(sqrt(mean(S.ctrl_dbg(seg, 3+ax).^2)));
            end
        case 'throttle_punch'
            k = S.t >= 1 & S.t < 1.6;
            r.I_bus_peak  = max(S.I_bus(k));
            r.vbat_before = mean(S.vbat_V(S.t > 0.5 & S.t < 1));
            r.vbat_min    = min(S.vbat_V(k));
            r.sag_V       = r.vbat_before - r.vbat_min;
            r.rpm_peak    = max(S.motor_rpm(k,:), [], 'all');
            r.rpm_check   = quad.checks.rpm_max(1);
            r.I_motor_peak = max(S.I_motor(k,:), [], 'all');
            r.climb_m     = max(-S.Xe(:,3)) - qs.alt0;
            r.att_max_deg = rad2deg(max(abs(S.euler(:,1:2)), [], 'all'));
        case 'takeoff'
            r.max_penetration_mm = 1e3*max(S.Xe(:,3));
            r.alt_final = -S.Xe(end,3);
            r.esc_disarmed_max = max(S.esc_us(S.t < 0.5,:), [], 'all');
            r.att_max_deg = rad2deg(max(abs(S.euler(:,1:2)), [], 'all'));
        case 'battery_drain'
            r.soc_end   = S.soc(end);
            r.vbat_start = mean(S.vbat_V(S.t > 2 & S.t < 10));
            r.vbat_end   = mean(S.vbat_V(S.t > qs.StopTime - 10));
            r.esc_hover_start = mean(S.esc_us(S.t > 2 & S.t < 10,:), 'all');
            r.esc_hover_end   = mean(S.esc_us(S.t > qs.StopTime - 10,:), 'all');
            tp = [20 120 220];
            for j = 1:numel(tp)
                k = S.t >= tp(j) & S.t < tp(j) + 0.4;
                kb = S.t >= tp(j) - 2 & S.t < tp(j);
                r.punch_t(j) = tp(j);
                r.punch_vbat_before(j) = mean(S.vbat_V(kb));
                r.punch_vbat_min(j) = min(S.vbat_V(k));
                r.punch_I_peak(j) = max(S.I_bus(k));
                r.punch_rpm_peak(j) = max(S.motor_rpm(k,:), [], 'all');
                r.punch_thrust_peak_N(j) = max(sum(S.thrust_N(k,:), 2));
            end
            r.alt_err_rms = sqrt(mean((-S.Xe(S.t > 5,3) - qs.alt0).^2));
    end
    r.sig = S;
    M.(sc) = r;
    local_print(sc, r);
    if o.plot, local_plot(sc, S, qs, ideal, here, o.save); end
end
end

%% ------------------------------------------------------------------------
function S = local_signals(out)
ls = out.logsout;
S = struct();
for i = 1:ls.numElements
    e = ls.getElement(i);
    v = e.Values;
    d = v.Data;
    if ndims(d) == 3, d = permute(d, [3 1 2]); d = reshape(d, size(d,1), []); end
    nm = matlab.lang.makeValidName(e.Name);
    S.([nm '_t']) = v.Time;
    S.(nm) = d;
end
% common time base = continuous plant signals (Xe)
S.t = S.Xe_t;
f = fieldnames(S);
for i = 1:numel(f)
    if endsWith(f{i}, '_t') || strcmp(f{i}, 't'), continue; end
    tt = S.([f{i} '_t']);
    if isempty(tt)                       % not logged (e.g. ctrl_dbg while the board is in the slot)
        S.(f{i}) = nan(numel(S.t), 1);
    elseif numel(tt) < 2                 % constant signal
        S.(f{i}) = repmat(double(S.(f{i})(1,:)), numel(S.t), 1);
    elseif numel(tt) ~= numel(S.t) || any(tt ~= S.t)
        S.(f{i}) = interp1(tt, double(S.(f{i})), S.t, 'previous', 'extrap');
    end
end
end

function local_print(sc, r)
fprintf('\n== %s  (%.0f s simulated in %.1f s wall)\n', sc, r.StopTime, r.wall_s);
f = fieldnames(r);
for i = 1:numel(f)
    if any(strcmp(f{i}, {'sig','wall_s','StopTime'})), continue; end
    v = r.(f{i});
    if ischar(v), fprintf('   %-22s %s\n', f{i}, v);
    else, fprintf('   %-22s %s\n', f{i}, mat2str(v, 4)); end
end
end

function local_plot(sc, S, qs, ideal, here, dosave)
fig = figure('Name', ['quad_sim: ' sc], 'Position', [100 100 1100 700], 'Visible', 'on');
tl = tiledlayout(fig, 3, 2, 'TileSpacing', 'compact');
title(tl, strrep(sc, '_', '\_'));
nexttile; plot(S.t, rad2deg(S.euler(:,1:2))); hold on;
cmdt = qs.cmd(:,1);
plot(cmdt, rad2deg(qs.cmd(:,2)*ideal.max_angle), 'k--', cmdt, rad2deg(-qs.cmd(:,3)*ideal.max_angle), 'k:');
ylabel('deg'); legend('\phi', '\theta', '\phi_{cmd}', '\theta_{cmd}', 'Location', 'best'); grid on; title('attitude');
nexttile; plot(S.t, rad2deg(S.pqr)); hold on; plot(cmdt, rad2deg(qs.cmd(:,4)*ideal.max_yaw_rate), 'k--');
ylabel('deg/s'); legend('p', 'q', 'r', 'r_{cmd}', 'Location', 'best'); grid on; title('body rates (true)');
nexttile; plot(S.t, S.esc_us); ylabel('us'); legend('M1', 'M2', 'M3', 'M4', 'Location', 'best'); grid on; title('esc\_us');
nexttile; plot(S.t, S.motor_rpm); ylabel('rpm'); grid on; title('motor speed');
nexttile; yyaxis left; plot(S.t, S.vbat_V); ylabel('V'); yyaxis right; plot(S.t, S.I_bus); ylabel('A'); grid on; title('battery: vbat, I\_bus');
nexttile; plot(S.t, -S.Xe(:,3), S.t, S.Xe(:,1), S.t, S.Xe(:,2)); ylabel('m'); legend('alt', 'x_N', 'y_E', 'Location', 'best'); grid on; title('position'); xlabel('s');
if dosave
    d = fullfile(here, 'results'); if ~exist(d, 'dir'), mkdir(d); end
    exportgraphics(fig, fullfile(d, ['quad_' sc '.png']), 'Resolution', 90);
end
end
