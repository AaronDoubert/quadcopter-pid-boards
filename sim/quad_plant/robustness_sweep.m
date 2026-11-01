function R = robustness_sweep()
%ROBUSTNESS_SWEEP How the tuned ideal gains cope with wrong inertia or an offset CG.
%   R = robustness_sweep() keeps the gains in ctrl_ideal_params fixed and
%   reruns quad_sim scenarios with the plant changed:
%     R.inertia  roll/pitch steps with J scaled by 0.7, 1, 1.4, 1.7, 2.5
%     R.cg       hover and takeoff with the CG moved fore/aft and sideways
%   Restores quad in the base workspace afterwards.

here = fileparts(mfilename('fullpath'));
evalin('base', sprintf('run(''%s'')', fullfile(here, 'quad_sim_params.m')));
quad0 = evalin('base', 'quad');
cleanup = onCleanup(@() assignin('base', 'quad', quad0));

%% Inertia error: the real J is k times what the gains were tuned for
ks = [0.7 1 1.4 1.7 2.5];
R.inertia = table();
for k = ks
    q = quad0;
    q.inertia.J = quad0.inertia.J.*diag([k k 1]) + quad0.inertia.J.*(1 - eye(3));
    q.inertia.Jinv = inv(q.inertia.J);
    assignin('base', 'quad', q);
    M = run_quad_scenarios({'roll_step','pitch_step'}, 'plot', false, 'save', false);
    R.inertia = [R.inertia; table(k, M.roll_step.overshoot_pct, M.roll_step.settle_5pct_s, ...
        M.pitch_step.overshoot_pct, M.pitch_step.settle_5pct_s, ...
        'VariableNames', {'J_scale','roll_os_pct','roll_ts_s','pitch_os_pct','pitch_ts_s'})];
end

%% CG offset: hover needs a steady torque, which only the I-term can supply
offs = [0 0; 5 0; 10 0; 20 0; 0 5; 0 10; 0 20]*1e-3;   % [x fwd, y right] in m
R.cg = table();
for i = 1:size(offs, 1)
    q = quad0;
    q.geom.cg_frame = quad0.geom.cg_frame + [offs(i,:)'; 0];
    q.geom.r_motor = q.geom.p_motor_frame - q.geom.cg_frame;
    q.geom.r_imu = [0; 0; -0.015] - q.geom.cg_frame;
    assignin('base', 'quad', q);
    M = run_quad_scenarios({'hover','takeoff'}, 'plot', false, 'save', false);
    e = M.hover.sig.esc_us(M.hover.sig.t > 5, :);
    spread = max(mean(e)) - min(mean(e));
    R.cg = [R.cg; table(1e3*offs(i,1), 1e3*offs(i,2), max(M.hover.att_rms_deg), ...
        spread, M.takeoff.att_max_deg, ...
        'VariableNames', {'cg_x_mm','cg_y_mm','hover_att_rms_deg','hover_esc_spread_us','takeoff_att_max_deg'})];
end
disp(R.inertia); disp(R.cg);
end
