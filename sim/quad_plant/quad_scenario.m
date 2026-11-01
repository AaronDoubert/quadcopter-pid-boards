function qsim = quad_scenario(qsim, quad, ideal, name)
%QUAD_SCENARIO Load a radio-command scenario into qsim (qsim.cmd, StopTime, start).
%   qsim = quad_scenario(qsim, quad, ideal, NAME), NAME one of
%     'hover'          10 s hover at 1 m (pilot altitude hold)
%     'roll_step'      roll stick 0 -> 0.25 (15 deg) at 1 s, back at 3.5 s
%     'pitch_step'     pitch stick 0 -> 0.25 (15 deg nose DOWN) at 1 s, back at 3.5 s
%     'yaw_step'       yaw stick 0 -> 0.4 (100 deg/s) at 1 s, back at 3 s
%     'throttle_punch' full throttle for 0.5 s from hover, then hold 4 m
%     'takeoff'        on the ground, disarmed; arm at 0.5 s, climb to 1.5 m
%     'battery_drain'  240 s hover from a full pack, 0.3 s punches at 20/120/220 s
%   qsim.cmd columns: [t roll pitch yaw thr arm alt_en alt_ref]
%   (sticks: roll/pitch/yaw -1..1, thr 0..1; alt_en 1 = simulated pilot
%   holds altitude alt_ref [m] by moving the throttle stick).
%   Each call re-derives qsim (initial conditions depend on the start mode).

thr_h = ((quad.esc.pwm_min_us + quad.checks.hover_duty*(quad.esc.pwm_max_us - quad.esc.pwm_min_us)) ...
         - ideal.us_idle)/(ideal.us_max - ideal.us_idle);    % throttle stick for hover (open loop)
h0 = qsim.alt0;
ep = 1e-6;
qsim.scenario = name;
qsim.start = 'air';
qsim.soc0 = quad.batt.soc0;
switch name
    case 'hover'
        T = 10;
        c = [0 0 0 0 thr_h 1 1 h0];
    case {'roll_step', 'pitch_step', 'yaw_step'}
        T = 6;
        ax = find(strcmp(name, {'roll_step','pitch_step','yaw_step'}));
        amp = [0.25 0.25 0.4];
        t2 = [3.5 3.5 3];
        s = zeros(1,3); s(ax) = amp(ax);
        c = [0      0 0 0 thr_h 1 1 h0
             1      0 0 0 thr_h 1 1 h0
             1+ep   s     thr_h 1 1 h0
             t2(ax) s     thr_h 1 1 h0
             t2(ax)+ep 0 0 0 thr_h 1 1 h0];
    case 'throttle_punch'
        T = 8;
        c = [0      0 0 0 thr_h 1 1 h0
             1      0 0 0 thr_h 1 1 h0
             1+ep   0 0 0 1     1 0 h0
             1.5    0 0 0 1     1 0 h0
             1.5+ep 0 0 0 thr_h 1 1 4];
    case 'takeoff'
        T = 6;
        qsim.start = 'ground';
        c = [0      0 0 0 0     0 0 0
             0.5    0 0 0 0     0 0 0
             0.5+ep 0 0 0 0     1 0 0
             1.0    0 0 0 0     1 0 0
             1.0+ep 0 0 0 thr_h 1 1 1.5];
    case 'battery_drain'
        T = 240;
        c = [0 0 0 0 thr_h 1 1 h0];
        for tp = [20 120 220]
            c = [c; tp 0 0 0 thr_h 1 1 h0; tp+ep 0 0 0 1 1 0 h0; ...
                    tp+0.3 0 0 0 1 1 0 h0; tp+0.3+ep 0 0 0 thr_h 1 1 h0]; %#ok<AGROW>
        end
    otherwise
        error('quad_scenario: unknown scenario "%s"', name);
end
if c(end,1) < T
    c = [c; T c(end,2:end)];
end
qsim.cmd = c;
qsim.StopTime = T;
qsim = quad_sim_derive(qsim, quad);
end
