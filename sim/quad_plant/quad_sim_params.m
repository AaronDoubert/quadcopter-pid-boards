%QUAD_SIM_PARAMS Set up everything quad_sim.slx needs (Model 1).
%   Runs quad_params.m (struct quad), ../common/quad_ctrl_buses.m (bus
%   CtrlIn), ctrl_ideal_params.m (struct ideal) and defines struct qsim
%   (simulation options, derived plant/IMU/ESC parameters, the scenario
%   command table). Creates no other base-workspace variables.
%
%   After editing a qsim field by hand, re-derive with
%       qsim = quad_sim_derive(qsim, quad);
%   and load a scenario with
%       qsim = quad_scenario(qsim, quad, ideal, 'roll_step');
%
%   Controller slot (Variant Subsystem "Controller" in quad_sim.slx):
%       qsim.ctrl = 1  ideal          (ctrl_ideal.slx)          DEFAULT
%       qsim.ctrl = 2  board_fast     (sim/pid_board/pid_board.slx, fast variant)
%       qsim.ctrl = 3  board_circuit  (sim/pid_board/pid_board.slx, circuit variant)

simroot_ = fileparts(mfilename('fullpath'));
addpath(simroot_, fullfile(simroot_, '..', 'common'));
if exist(fullfile(simroot_, '..', 'pid_board'), 'dir')
    addpath(fullfile(simroot_, '..', 'pid_board'));
end
run(fullfile(simroot_, 'quad_params.m'));
run(fullfile(simroot_, '..', 'common', 'quad_ctrl_buses.m'));
run(fullfile(simroot_, 'ctrl_ideal_params.m'));
clear simroot_

qsim = struct();
%% Solver / rates
qsim.dt       = 5e-4;      % s, fixed step (ode4). Controller 1 kHz, ESC frame and IMU are multiples
qsim.Ts_bus   = 1e-3;      % s, CtrlIn bus is sampled at 1 kHz (nonvirtual bus = one rate)
qsim.StopTime = 10;        % s

%% Controller variant: 1 ideal, 2 board_fast, 3 board_circuit
qsim.ctrl = 1;

%% Initial condition: 'air' (hovering at alt0, motors at hover speed) or 'ground'
qsim.start = 'air';
qsim.alt0  = 1.0;          % m above ground for 'air'
qsim.soc0  = quad.batt.soc0;

%% Ground contact (spring-damper, only pushes up)
qsim.gnd.k    = 2000;      % N/m   (static deflection m*g/k = 3 mm)
qsim.gnd.c    = 50;        % N s/m
qsim.gnd.cxy  = 5;         % N s/m, sliding damping
qsim.gnd.klev = 2.0;       % N m/rad, levelling torque from the legs
qsim.gnd.clev = 0.05;      % N m s/rad

%% IMU options
qsim.imu.noise_on = 1;
qsim.imu.bias_on  = 1;
qsim.imu.seed     = [11 12 13 14 15 16];
% residual bias after boot calibration, fixed pattern (scaled by quad.imu limits)
qsim.imu.bias_g_pat = [0.6; -0.8; 0.5];   % x quad.imu.gyro.bias_resid
qsim.imu.bias_a_pat = [0.5; -0.5; 0.3];   % x quad.imu.accel.bias_init_xy/_z
qsim.imu.scale_g  = [0; 0; 0];            % sensitivity error per axis (tol +-0.03)
qsim.imu.scale_a  = [0; 0; 0];

%% Simulated pilot altitude hold on the throttle stick (outside the controller)
qsim.pilot.kh     = 1.5;   % 1/s, altitude error -> climb-rate demand
qsim.pilot.vz_max = 2.0;   % m/s, climb/descent rate limit
qsim.pilot.kvz    = 0.10;  % throttle per m/s of climb-rate error
qsim.pilot.ki     = 0.10;  % throttle per m (integral of climb-rate error)

%% Derived parameters and default scenario
qsim = quad_sim_derive(qsim, quad);
qsim = quad_scenario(qsim, quad, ideal, 'hover');
