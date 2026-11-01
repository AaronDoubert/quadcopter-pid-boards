function M = pid_board_bench_inputs(scenario, rec)
%PID_BOARD_BENCH_INPUTS Inputs for pid_board_bench.slx (writes pbsim.bench_in, pbsim.t_stop).
%   pid_board_bench_inputs('still')     level and still, armed, 30 % throttle (2 s)
%   pid_board_bench_inputs('wobble')    board rocked by hand: roll +-30 deg at 1.5 Hz,
%                                       pitch +-15 deg at 0.8 Hz, yaw rate 60 deg/s (4 s)
%   pid_board_bench_inputs('sticks')    level, armed; roll/pitch/yaw stick steps (3 s)
%   pid_board_bench_inputs('disarmed')  wobble with ARM off (all ESC = 1000 us)
%   pid_board_bench_inputs('bench_roll') still IMU + LOOP/GYRO bench mode on the roll cell:
%                                       sets pcb.roll.JP_SP = JP_FB = 2, flying lead
%                                       PLANT_OUT -> EXT_SP, AD3 W1 step on EXT_FB (1 s)
%   pid_board_bench_inputs('record', S) recorded data: S.t (N x 1) plus any of the CtrlIn
%                                       fields S.gyro_radps (N x 3), S.accel_mps2, S.rates_true,
%                                       S.euler_true, S.sticks (N x 4), S.arm, S.vbat_V (N x 1).
%                                       Missing fields: gyro = rates_true, accel from euler_true
%                                       (quasi-static), sticks 0 / throttle 0.3, arm 1, 15.4 V.
%   Columns of M: t | gyro(3) accel(3) rates_true(3) euler_true(3) sticks(4) arm vbat.
%   Body axes FRD, rad, rad/s, m/s^2 (see sim/common/quad_ctrl_buses.m).

g = 9.80665;
if nargin < 1, scenario = 'still'; end
switch scenario
    case {'still', 'disarmed', 'wobble', 'sticks', 'bench_roll'}
        T = struct('still', 2, 'disarmed', 3, 'wobble', 4, 'sticks', 3, 'bench_roll', 1);
        t = (0:1e-3:T.(scenario))';
        eul = zeros(numel(t), 3); w = zeros(numel(t), 3);
        st = repmat([0 0 0 0.3], numel(t), 1);
        arm = ones(numel(t), 1);
        if any(strcmp(scenario, {'wobble', 'disarmed'}))
            A = [30 15]*pi/180; f = [1.5 0.8];
            eul(:, 1) = A(1)*sin(2*pi*f(1)*t); w(:, 1) = A(1)*2*pi*f(1)*cos(2*pi*f(1)*t);
            eul(:, 2) = A(2)*sin(2*pi*f(2)*t); w(:, 2) = A(2)*2*pi*f(2)*cos(2*pi*f(2)*t);
            w(:, 3) = 60*pi/180*(t > 0.5);
            eul(:, 3) = cumtrapz(t, w(:, 3));
        end
        if strcmp(scenario, 'disarmed'), arm(:) = 0; st(:, 4) = 0.5; end
        if strcmp(scenario, 'sticks')
            st(t >= 0.5, 1) = 0.5; st(t >= 1.2, 2) = -0.5; st(t >= 2.0, 3) = 0.4;
        end
        if strcmp(scenario, 'bench_roll')
            pcb = evalin('base', 'pcb');
            pcb.roll.JP_SP = 2; pcb.roll.JP_FB = 2;
            pcb.bench.ext_sp_src(1) = 1; pcb.bench.ext_fb_src(1) = 2;
            assignin('base', 'pcb', pcb);
            fprintf(['bench_roll: set pcb.roll.JP_SP = JP_FB = 2, pcb.bench.ext_sp_src(1) = 1 ' ...
                '(PLANT_OUT flying lead), ext_fb_src(1) = 2 (W1). Re-run pid_board_params to undo.\n']);
        end
        S = struct('t', t, 'gyro_radps', w, 'rates_true', w, 'euler_true', eul, 'sticks', st, 'arm', arm);
    case 'record'
        S = rec;
    otherwise
        error('Unknown scenario %s', scenario);
end
t = S.t(:); N = numel(t);
f = @(name, def) local_field(S, name, def, N);
eul = f('euler_true', zeros(N, 3));
rt = f('rates_true', zeros(N, 3));
gy = f('gyro_radps', rt);
acc = f('accel_mps2', g*[sin(eul(:, 2)), -sin(eul(:, 1)).*cos(eul(:, 2)), -cos(eul(:, 1)).*cos(eul(:, 2))]);
st = f('sticks', repmat([0 0 0 0.3], N, 1));
arm = f('arm', ones(N, 1));
vb = f('vbat_V', 15.4*ones(N, 1));
M = [t gy acc rt eul st arm vb];
if evalin('base', 'exist(''pbsim'', ''var'')'), pbsim = evalin('base', 'pbsim'); else, pbsim = struct(); end
pbsim.bench_in = M;
pbsim.t_stop = t(end);
assignin('base', 'pbsim', pbsim);
end

function v = local_field(S, name, def, N)
if isfield(S, name) && ~isempty(S.(name)), v = S.(name); else, v = def; end
if size(v, 1) ~= N, v = v.'; end
end
