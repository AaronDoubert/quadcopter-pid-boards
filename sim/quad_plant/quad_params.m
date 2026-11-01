%QUAD_PARAMS Physical parameters for a 6-DOF model of the 5" analog-PID quad.
%   Defines ONE struct, `quad`, in SI units (m, kg, s, N, rad, V, A).
%   Sources, fits and the status of each number (measured / computed /
%   estimated) are in quad_params_sources.md next to this file.
%
%   Aircraft: 5" freestyle X frame, 4x 2207 1800KV, 5x4.3x3 tri-blade
%   props, 4S 1500 mAh LiPo (1300 mAh alternate), Aero Selfie 4in1 30A ESC,
%   GY-521 (MPU-6050), PID_Board rev B (90x210 mm) on the frame.
%
%   CONVENTIONS (used everywhere below)
%   * Body frame FRD: x forward, y right, z down. Origin at the CG.
%   * Euler angles roll/pitch/yaw are right-handed about x/y/z: positive
%     roll = right side down, positive pitch = nose up, positive yaw = nose
%     right (clockwise seen from above).
%   * Motor numbering is Betaflight Quad-X:
%       M1 rear-right, M2 front-right, M3 rear-left, M4 front-left.
%   * Spin direction, Betaflight default "props in":
%       M1 CW, M2 CCW, M3 CCW, M4 CW (as seen from ABOVE).
%     quad.motor.spin = +1 for CW-from-above. In FRD that rotor angular
%     velocity points along +z (down), so:
%       rotor angular momentum  h_i   = spin_i * J_rotor * w_i * [0;0;1]
%       reaction torque on body Q_i   = -spin_i * kQ * w_i^2  (about +z)
%     Check: speeding up M2+M3 (CCW, spin=-1) gives +z torque = nose right,
%     which matches the Betaflight Quad-X yaw mixer column (-1 +1 +1 -1).
%   * Thrust of rotor i:  T_i = kT * w_i^2  along -z (up), w in rad/s.

%% Mass
quad.mass.total      = 0.650;   % kg, all-up weight with 1500 mAh pack (given, WEIGH IT)
quad.mass.total_1300 = 0.622;   % kg, same quad with a ~155 g 1300 mAh 4S pack (estimated)
quad.g               = 9.80665; % m/s^2, standard gravity
quad.rho             = 1.225;   % kg/m^3, ISA sea-level air density

%% Geometry (from PID_Board/MOUNTING.md sketch, not yet measured)
% Motor centres in the frame-centre system (FRD axes, origin = frame centre
% mark on the top of the bottom plate). MOUNTING.md gives (+-87, +-74.5) mm
% in KiCad x/y; KiCad x is lateral and -y is the front, so the frame is a
% slightly "wide X": 87 mm lateral, 74.5 mm fore-aft, 229 mm diagonal.
% If the real front turns out to be along KiCad x, swap the two numbers.
quad.geom.arm_lat   = 0.087;    % m, lateral (y) motor offset = roll moment arm
quad.geom.arm_lon   = 0.0745;   % m, fore-aft (x) motor offset = pitch moment arm
quad.geom.arm       = hypot(quad.geom.arm_lat, quad.geom.arm_lon); % m, 0.1145 centre-to-motor
quad.geom.diag      = 2*quad.geom.arm;  % m, 0.229 motor-to-motor diagonal
quad.geom.z_prop    = -0.030;   % m, prop plane above the frame-centre origin (estimated)
% Columns = M1..M4 (Betaflight order), rows = x,y,z, frame-centre origin
quad.geom.p_motor_frame = [ -quad.geom.arm_lon   quad.geom.arm_lon  -quad.geom.arm_lon   quad.geom.arm_lon ;
                             quad.geom.arm_lat   quad.geom.arm_lat  -quad.geom.arm_lat  -quad.geom.arm_lat ;
                             quad.geom.z_prop*[1 1 1 1] ];
quad.geom.cg_frame  = [0.0023; 0; -0.0018]; % m, CG from the mass breakdown (computed)
quad.geom.r_motor   = quad.geom.p_motor_frame - quad.geom.cg_frame; % m, rotor hubs relative to CG
quad.geom.r_imu     = [0; 0; -0.015] - quad.geom.cg_frame; % m, MPU-6050 over the frame centre, ~15 mm up (estimated)

%% Inertia (computed from a component mass breakdown, see sources.md)
% Battery assumed strapped UNDER the frame (the 90x210 board covers the top).
% Product of inertia Ixz is small (front-heavy misc mass); Ixy = Iyz = 0.
quad.inertia.J = [ 1.62e-3   0        1.8e-5 ;
                   0         1.87e-3  0      ;
                   1.8e-5    0        3.14e-3 ];   % kg m^2, body FRD about the CG
quad.inertia.Jinv = inv(quad.inertia.J);
% Spin-axis inertia of one rotor (bell+magnets ~1.8e-6, prop ~3.6e-6)
quad.inertia.J_rotor = 5.3e-6;  % kg m^2, estimated (spin-down test to measure)

%% Propeller aerodynamics (HQ 5x4.3x3 V1S; Gemfan 51433 alternative)
% Static fits T = kT w^2, Q = kQ w^2 (w in rad/s). kT from MiniQuadTestBench
% HQ v1s 5x4.3x3 data at ~16 V, 25-100 % rows (least squares). kQ from the
% same data via phase current x Kt (100 % row) and a power balance (75 %).
quad.prop.diameter = 5*0.0254;  % m
quad.prop.pitch    = 4.3*0.0254;% m
quad.prop.blades   = 3;
quad.prop.mass     = 3.8e-3;    % kg each
quad.prop.kT       = 1.32e-6;   % N/(rad/s)^2   (HQ 5x4.3x3, fit, +-5 %)
quad.prop.kQ       = 1.40e-8;   % N m/(rad/s)^2 (fit, +-15 %), kQ/kT = 10.6 mm
quad.prop.kT_GF51433 = 1.45e-6; % N/(rad/s)^2, Gemfan 51433 fit 18-26k rpm (Skystars 6S data)
quad.prop.kQ_GF51433 = 1.60e-8; % N m/(rad/s)^2, scaled with kQ/kT = 0.011
% Raw thrust-stand data so a lookup table can replace kT*w^2 if wanted
quad.prop.data_HQ.rpm      = [5869 11004 18326 24588 29990];
quad.prop.data_HQ.thrust_N = [45 170 494 893 1309]*1e-3*quad.g;
quad.prop.data_HQ.P_elec_W = [14 42 144 318 553];
quad.prop.data_GF.rpm      = [18301 20592 24050 26781 29185 30474];
quad.prop.data_GF.thrust_N = [497 696 964 1236 1465 1605]*1e-3*quad.g;

%% Motor electrical / mechanical model (iFlight XING-E 2207 1800KV specs)
% DC-equivalent BLDC:  V_m = w/Kv + R*I_m,   Q_motor = Kt*(I_m - I0)
% V_m = ESC duty * V_bus. R is an EFFECTIVE resistance: winding (81.5 mOhm,
% line-line) + ~60 mOhm for ESC FETs, wiring and commutation losses, sized
% so a 2550KV motor on the same prop reproduces the MQTB 4S full-throttle RPM.
quad.motor.KV     = 1800;                     % rpm/V (label)
quad.motor.Kv     = quad.motor.KV*2*pi/60;    % rad/s/V = 188.5
quad.motor.Kt     = 1/quad.motor.Kv;          % N m/A = 5.31e-3 (= Ke in V s/rad)
quad.motor.R_wind = 0.0815;                   % ohm, winding (manufacturer)
quad.motor.R      = 0.14;                     % ohm, effective R used in the model (estimated)
quad.motor.I0     = 1.2;                      % A, no-load current at 10 V (manufacturer, "<1.2 A")
quad.motor.mass   = 33.8e-3;                  % kg
quad.motor.spin   = [+1 -1 -1 +1];            % M1..M4, +1 = CW seen from above
% Linearised mechanical time constant tau = J_rotor/(Kt^2/R + 2*kQ*w):
% 23 ms at hover, 20 ms near full speed (computed). Use as a first-order
% thrust lag if the electrical model is not simulated.
quad.motor.tau_hover = quad.inertia.J_rotor/(quad.motor.Kt^2/quad.motor.R + 2*quad.prop.kQ*1099);
quad.motor.tau       = 0.025;                 % s, recommended first-order lag (20-30 ms band)
quad.motor.I_max_cont_ESC = 10;               % A per channel continuous (ESC listing)
quad.motor.I_max_peak_ESC = 30;               % A per channel, 30 s burst (ESC listing)

%% Battery (4S LiPo, 1500 mAh default)
quad.batt.cells      = 4;
quad.batt.capacity   = 1.5*3600;    % C (1.5 Ah)
quad.batt.capacity_1300 = 1.3*3600; % C, alternate pack
quad.batt.mass       = 0.183;       % kg (CNHL 1500 4S 100C class)
quad.batt.mass_1300  = 0.155;       % kg (estimated)
% Per-cell open-circuit voltage vs state of charge (generic LCO/LiPo curve)
quad.batt.soc_tab = [0    0.02 0.05 0.12 0.22 0.38 0.52 0.65 0.78 0.90 1.00];
quad.batt.ocv_tab = [3.00 3.30 3.40 3.50 3.60 3.70 3.80 3.90 4.00 4.10 4.20]; % V per cell
quad.batt.R_cell      = 0.006;      % ohm per cell, DC (AC-IR ~3.3 mOhm measured x ~1.8)
quad.batt.R_cell_1300 = 0.007;      % ohm per cell, smaller pack (estimated)
quad.batt.R_wire      = 0.004;      % ohm, XT60 + leads + ESC pads (estimated)
quad.batt.R_total     = quad.batt.cells*quad.batt.R_cell + quad.batt.R_wire; % 0.028 ohm
quad.batt.V_full      = 4.20*quad.batt.cells;  % V OCV fully charged
quad.batt.V_nominal   = 3.85*quad.batt.cells;  % V OCV mid-pack
quad.batt.V_land      = 3.50*quad.batt.cells;  % V loaded, "land now"
quad.batt.soc0        = 1.0;        % initial SOC for simulations
quad.batt.I_avionics  = 0.3;        % A, PID board + RX + buck (estimated)

%% ESC (Aero Selfie 4in1 30A, 20x20)
% Listing: PWM / Oneshot / DShot input, 2-4S (6-18 V), 8-bit MCU, 10 A
% continuous / 30 A 30 s burst per channel. Firmware not published: most
% likely BLHeli_S (EFM8). Check it with esc-configurator.com before flying.
% The analog board's ATtiny drives it; standard PWM is the default here.
quad.esc.protocol     = 'PWM';     % 'PWM' | 'Oneshot125' | 'DShot300' ...
quad.esc.pwm_min_us   = 1000;      % us -> duty 0 (after throttle calibration)
quad.esc.pwm_max_us   = 2000;      % us -> duty 1
quad.esc.update_hz    = 490;       % Hz, PWM frame rate from the mixer (estimated; 50-490 Hz)
quad.esc.frame_by_protocol_us = struct('PWM',2000,'Oneshot125',250,'Oneshot42',84, ...
                                       'Multishot',25,'DShot150',106.7,'DShot300',53.3,'DShot600',26.7);
% Total command latency = half a frame (sampling) + pulse + ESC processing
quad.esc.latency      = 0.5/quad.esc.update_hz + 2.0e-3 + 0.2e-3; % s, ~3.2 ms with PWM 490 Hz
quad.esc.idle_duty    = 0.05;      % duty while armed at zero throttle (estimated, BLHeli_S-like)
quad.esc.V_min        = 6;         % V, input range from listing
quad.esc.V_max        = 18;        % V
quad.esc.I_limit      = 30;        % A per channel, burst
% Throttle -> motor voltage (BLHeli_S "damped light" is close to linear):
%   duty = clamp((pulse_us - pwm_min_us)/(pwm_max_us - pwm_min_us), idle_duty, 1)
%   V_motor = duty * V_bus ;  V_bus = OCV(soc)*cells - R_total*I_bus

%% Drag
% Linear rotor drag (Faessler et al. 2018, 610 g racing quad: dx 0.49-0.54,
% dy 0.24-0.39 1/s). F_drag_body = -mass*diag(d_lin)*v_body.
quad.drag.d_lin  = [0.40; 0.40; 0.0];         % 1/s, mass-normalised (estimated)
quad.drag.k_lin  = quad.mass.total*quad.drag.d_lin; % N/(m/s) = 0.26 in x,y
% Quadratic parasite drag F = -0.5*rho*CdA.*|v|.*v (body axes)
quad.drag.CdA    = [0.010; 0.013; 0.035];     % m^2 (estimated from frontal/plan areas)
% Rotational damping from rotor inflow change (momentum theory) and
% in-plane rotor drag: M_damp = -diag(k_rot)*omega_body
quad.drag.k_rot  = [3.4e-3; 2.5e-3; 3.4e-3];  % N m/(rad/s) (estimated, +-100 %)

%% IMU: MPU-6050 on GY-521 (PS-MPU-6000A-00 rev 3.1; RM-MPU-6000A-00 rev 4.0)
quad.imu.gyro.fs_sel_dps   = 250;            % +-250/500/1000/2000 deg/s; set to match firmware
quad.imu.gyro.lsb_per_dps  = [131 65.5 32.8 16.4]; % for FS_SEL 0..3
quad.imu.gyro.noise_density = 0.005*pi/180;  % rad/s/sqrt(Hz) (0.005 deg/s/rtHz)
quad.imu.gyro.rms_noise_100Hz = 0.05*pi/180; % rad/s rms at DLPF_CFG=2
quad.imu.gyro.bias_init_max  = 20*pi/180;    % rad/s, initial ZRO tolerance (calibrate at boot)
quad.imu.gyro.bias_temp_max  = 20*pi/180;    % rad/s, ZRO change over -40..85 C
quad.imu.gyro.bias_resid     = 0.5*pi/180;   % rad/s, residual after boot calibration (estimated)
quad.imu.gyro.scale_tol      = 0.03;         % +-3 % sensitivity tolerance
quad.imu.gyro.cross_axis     = 0.02;         % +-2 %
quad.imu.gyro.g_sens         = 0.1*pi/180;   % rad/s per g, linear-accel sensitivity
quad.imu.accel.fs_g          = 2;            % +-2/4/8/16 g
quad.imu.accel.lsb_per_g     = [16384 8192 4096 2048];
quad.imu.accel.noise_density = 400e-6*quad.g;% m/s^2/sqrt(Hz) (400 ug/rtHz)
quad.imu.accel.bias_init_xy  = 0.050*quad.g; % m/s^2, +-50 mg
quad.imu.accel.bias_init_z   = 0.080*quad.g; % m/s^2, +-80 mg
quad.imu.accel.bias_temp_xy  = 0.035*quad.g; % m/s^2 over 0..70 C
quad.imu.accel.bias_temp_z   = 0.060*quad.g;
quad.imu.accel.scale_tol     = 0.03;
quad.imu.accel.cross_axis    = 0.02;
% DLPF_CFG 0..6 table: bandwidth (Hz) and group delay (s)
quad.imu.dlpf.cfg        = 0:6;
quad.imu.dlpf.gyro_bw    = [256 188 98 42 20 10 5];
quad.imu.dlpf.gyro_delay = [0.98 1.9 2.8 4.8 8.3 13.4 18.6]*1e-3;
quad.imu.dlpf.acc_bw     = [260 184 94 44 21 10 5];
quad.imu.dlpf.acc_delay  = [0 2.0 3.0 4.9 8.5 13.8 19.0]*1e-3;
quad.imu.dlpf.gyro_rate  = [8000 1000 1000 1000 1000 1000 1000]; % Hz internal rate
quad.imu.dlpf_cfg_used   = 2;        % matches roll_params.m (98 Hz, 2.8 ms)
quad.imu.Ts              = 1e-3;     % s, sample period read by the shim firmware

%% Derived checks (recomputed from the fields above)
% Hover: per-motor thrust, speed, current, terminal voltage, duty
quad.checks.T_hover   = quad.mass.total*quad.g/4;                         % N (1.59)
quad.checks.w_hover   = sqrt(quad.checks.T_hover/quad.prop.kT);           % rad/s (~1099, 10.5k rpm)
quad.checks.I_hover   = quad.motor.I0 + quad.prop.kQ*quad.checks.w_hover^2/quad.motor.Kt; % A (~4.4)
quad.checks.Vm_hover  = quad.checks.w_hover/quad.motor.Kv + quad.motor.R*quad.checks.I_hover; % V
quad.checks.Vbus_hover = quad.batt.V_nominal - quad.batt.R_total*(4*quad.checks.Vm_hover*quad.checks.I_hover/quad.batt.V_nominal + quad.batt.I_avionics);
quad.checks.hover_duty = quad.checks.Vm_hover/quad.checks.Vbus_hover;    % ~0.42 (0.40 full, 0.46 low)
quad.checks.P_hover    = 4*quad.checks.Vm_hover*quad.checks.I_hover;     % W (~113)
% Full throttle: solve (R*kQ/Kt) w^2 + w/Kv + (R*I0 - V) = 0 for loaded bus
% voltages found by iterating the battery sag (done offline, see sources.md)
quad.checks.Vbus_full_load = [14.97 13.80 12.62]; % V at OCV 4.20 / 3.85 / 3.50 V per cell
quad.checks.w_max = (-1/quad.motor.Kv + sqrt(1/quad.motor.Kv^2 - 4*(quad.motor.R*quad.prop.kQ/quad.motor.Kt) ...
                    .*(quad.motor.R*quad.motor.I0 - quad.checks.Vbus_full_load))) ...
                    ./(2*quad.motor.R*quad.prop.kQ/quad.motor.Kt);        % rad/s [2388 2222 2050]
quad.checks.rpm_max   = quad.checks.w_max*60/(2*pi);                     % [22.8k 21.2k 19.6k]
quad.checks.T_max     = quad.prop.kT*quad.checks.w_max.^2;               % N per motor [7.5 6.5 5.6]
quad.checks.I_max     = quad.motor.I0 + quad.prop.kQ*quad.checks.w_max.^2/quad.motor.Kt; % A [16.3 14.2 12.3]
quad.checks.TW        = 4*quad.checks.T_max/(quad.mass.total*quad.g);    % [4.7 4.1 3.5]
% Peak angular accelerations, full pack: two motors at max, two at idle
quad.checks.w_idle    = 108;                                             % rad/s at 5 % duty
quad.checks.alpha_roll  = 2*quad.prop.kT*(quad.checks.w_max(1)^2-quad.checks.w_idle^2)*quad.geom.arm_lat/quad.inertia.J(1,1); % ~800 rad/s^2
quad.checks.alpha_pitch = 2*quad.prop.kT*(quad.checks.w_max(1)^2-quad.checks.w_idle^2)*quad.geom.arm_lon/quad.inertia.J(2,2); % ~600 rad/s^2
quad.checks.alpha_yaw   = 2*quad.prop.kQ*(quad.checks.w_max(1)^2-quad.checks.w_idle^2)/quad.inertia.J(3,3);                   % ~51 rad/s^2
