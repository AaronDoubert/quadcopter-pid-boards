%ROLL_PARAMS Workspace parameters for roll_axis.slx (analog PID board, roll axis).
%   Values come from the "Analog PID Simulation Briefing" (2026-10-20).
%   Every controller voltage is measured from VG = 2.048 V. Block
%   parameters in the model use these names directly, so after changing a
%   pot (RV_*) or plant value you can simulate without re-running this file.
%   Re-running it restores every default.

%% Op-amp rails and clamps (relative to VG)
VG        = 2.048;     % V, virtual ground (reference only)
V_rail_lo = -2.0;      % V, op-amp output swing ~0.03 V
V_rail_hi = 2.9;       % V, op-amp output swing ~4.97 V
V_iclamp  = 0.75;      % V, 2 + 2 x 1N4148 across CI (about 0.6-0.9 V)

%% Trimpots (ohms) - the values you tune
RV_P = 50e3;           % RV201, 100k: Kp = (4.7k + RV_P)/47k       (1.16 nominal)
RV_I = 67e3;           % RV202, 100k: Ki = 1/((33k + RV_I)*1 uF)   (10 /s nominal)
RV_D = 50e3;           % RV203, 100k: Kd = (47k + RV_D)*100 nF     (9.7 ms nominal)
RV_A = 25e3;           % RV204,  50k: Go = (10k + RV_A)/20k        (1.75 nominal)

% Term enables (1 = fitted, 0 = switched off)
en_P = 1;
en_I = 1;
en_D = 1;

%% Fixed controller parts
RI_fixed = 33e3;       % I input resistor (yaw would be 15k)
CI       = 1e-6;       % I capacitor, film
RL       = 1e6;        % I leak resistor -> wL = 1 rad/s
Rs       = 27e3;       % D series resistor -> f1 = 59 Hz
Cd       = 100e-9;     % D capacitor (220 nF widens Kd to 10-32 ms)
Rd_fixed = 47e3;       % D feedback fixed part
Cf       = 10e-9;      % D feedback cap -> f2 = 108-339 Hz

% Diff-amp gains set by source impedance (1k filter in series with 10k)
g_sp  = 0.955;         % rate setpoint leg (driven by angle-P op-amp, ~0 ohm)
g_fb  = 0.909;         % gyro leg (through the 1k/1uF RC filter)
g_ang = 0.909;         % both angle diff-amp legs (both filtered)

%% Signal scales
Kw     = 0.006*180/pi; % V per rad/s (6 mV per deg/s)
Ktheta = 0.025;        % V per deg

%% Sensor, firmware and DAC path
t_dlpf  = 2.8e-3;      % s, MPU-6050 DLPF at 98 Hz as a pure delay
Ts_fw   = 1e-3;        % s, firmware loop (ZOH, 0.5 ms average) -> 3.3 ms total
dac_lsb = 1e-3;        % V, MCP4822 12-bit at 1 mV/LSB
dac_lo  = -2.048;      % V, DAC range 0-4.095 V around VG
dac_hi  = 2.047;
tau_rc  = 1e3*1e-6;    % s, 1k/1uF filter on each DAC output (159 Hz)

gyro_noise_on = 0;     % 1 injects gyro white noise
gyro_nd       = 0.005; % deg/s/sqrt(Hz)
noise_Ts      = 1e-4;  % s, noise block sample time

%% Mixer and ESC
mix_adc_lsb = 4.9e-3*(1e3 + 2.2e3)/2.2e3; % V at U: 10-bit ADC behind 1k/2.2k divider
t_esc       = 0;       % s, mixer compute + ESC protocol (PWM 50 Hz ~10 ms, DShot < 1 ms)

%% Airframe (5-inch example from the briefing - replace with measurements)
J   = 0.004;           % kg m^2, roll inertia Jx
L   = 0.11;            % m, motor-to-centre arm length
m   = 0.6;             % kg
g0  = 9.81;            % m/s^2
tau_m = 0.04;          % s, motor + prop + ESC time constant (measure first)

T_hover = m*g0/4;      % N per motor
T_max   = 4*T_hover;   % N per motor at full throttle (placeholder)
T_min   = 0.1*T_hover; % N per motor at idle (placeholder)

% Mixer scale KF (N per V of U). KF_nom lands the plant on K = 45.5 /s,
% the 22 ms bench integrator the nominal pots were sized for.
K_target    = 45.5;    % 1/s
plant_scale = 1;       % plant gain relative to K_target
KF_nom      = K_target*J/(2*sqrt(2)*L*Kw);
KF          = plant_scale*KF_nom;

% Single-axis rig extras (0 for free flight)
b_fric = 0;            % N m s/rad, bearing friction
mgh    = 0;            % N m, pendulum torque m*g*h (CG h below pivot)

%% Bench plant (U202D, 22k / 1uF) and mode switches
tau_p         = 0.022; % s
bench_mode    = 0;     % 1: U202D bench integrator drives FB directly (no sensor path)
angle_loop_on = 1;     % 0: rate-only, setpoint from the rate_sp step

%% Test inputs
t_step        = 0.05;  % s, angle and rate step time
theta_cmd_deg = 0;     % deg, angle command step
rate_sp_V     = 0;     % V, rate setpoint step (rate-only mode, 0.1 V = 17 deg/s)
t_dist        = 0.5;   % s
dist_Nm       = 0;     % N m, torque disturbance step
t_arm         = 0;     % s, I_HOLD keeps the integrator at zero until this time
t_stop        = 1;     % s
