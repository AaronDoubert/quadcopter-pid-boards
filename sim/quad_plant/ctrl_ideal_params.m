%CTRL_IDEAL_PARAMS Gains and settings of the ideal 5-loop cascaded PID (ctrl_ideal.slx).
%   Defines ONE struct, `ideal`, in the base workspace. Needs `quad`
%   (run quad_params.m first; quad_sim_params.m does both).
%
%   Loop structure mirrors the PID board (sim/pid_board/pid_board_spec.md):
%     roll/pitch: rate_sp = Ka*(angle_cmd - angle)                (angle P)
%     all axes  : e = rate_sp - gyro
%                 u = Kp*e + I + D                                 [us]
%                 dI/dt = Ki*e - wL*I,  |I| <= I_max   (leaky, clamped)
%                 D = Kd*s/((1+s/w1)(1+s/w2)) * e      (band-limited, on the error)
%     mixer     : esc_us(i) = thr_us + mix(i,:)*[u_roll; u_pitch; u_yaw]
%   Units: e in rad/s, u in microseconds of ESC pulse. So Kp is us/(rad/s),
%   Ki us/rad, Kd us/(rad/s^2), Ka 1/s.
%
%   Mapping onto the board (pcb = pid_board_params, Kw = pcb.scale.Kw in
%   V/(deg/s), Kth = pcb.scale.Ktheta V/deg, Kmix = control_io mixer us/V):
%     Kp_ideal = Kmix * Kp_board * (Kw*180/pi)
%     Ki_ideal = Kmix * Ki_board * (Kw*180/pi),   wL = 1/(R_L*C_I) = 1 rad/s
%     Kd_ideal = Kmix * Kd_board * (Kw*180/pi)
%     Ka_ideal = Go * g_ang * (Kth/Kw) * (g_sp/g_fb)          (= 7.0 1/s at Go = 1.75)
%     I_max    = Kmix * V_iclamp,  u_max = Kmix * |rail|
%   ctrl_ideal_to_board() in this folder does the inverse (ideal -> RV_*).
%
%   Default gains below are the result of tune_ideal.m (see README_model1.md).

ideal = struct();
ideal.Ts = 1e-3;                     % s, controller rate (1 kHz, like the shim frame)

%% Command scaling (sticks -> setpoints), mirrors the shim assumptions
ideal.max_angle    = 60*pi/180;      % rad at full roll/pitch stick (pcb.shim.angle_max)
ideal.max_yaw_rate = 250*pi/180;     % rad/s at full yaw stick (pcb.shim.yaw_rate_max)
ideal.max_rate_sp  = 341*pi/180;     % rad/s, rate setpoint clip (board DAC clips at +-341 deg/s)

%% Measurement selection
% Angles always come from euler_true (the ideal controller has no attitude
% estimator; the board twin runs its own complementary filter on accel/gyro).
ideal.use_true_rates = 0;            % 0: gyro_radps (IMU model), 1: rates_true

%% Rate PID + angle P per axis: tune_ideal.m result (2026-10-31, hover op point,
%  rate loop PM >= 45 deg / GM >= 8 dB, Ki = 6*Kp, angle loop wc = rate wc/4)
ideal.roll.Ka    = 16.9;             % 1/s, angle P (rate_sp per rad of angle error)
ideal.roll.Kp    = 42.5;             % us/(rad/s)
ideal.roll.Ki    = 255;              % us/rad
ideal.roll.Kd    = 0.940;            % us/(rad/s^2)
ideal.roll.wL    = 1.0;              % rad/s, integrator leak (board R208 1M * C201 1uF)
ideal.roll.I_max = 150;              % us, integrator clamp (board 0.75 V * 200 us/V)
ideal.roll.w1    = 1/(27e3*100e-9);     % rad/s, board D pole R209*C202 (59 Hz)
ideal.roll.w2    = 1/((47e3+50e3)*10e-9); % rad/s, board D roll-off Rd*Cf at RV_D = 50k (164 Hz)

ideal.pitch      = ideal.roll;       % same structure; shorter pitch arm + 15 % more inertia
ideal.pitch.Ka   = 17.6;
ideal.pitch.Kp   = 71.4;
ideal.pitch.Ki   = 429;
ideal.pitch.Kd   = 1.17;

ideal.yaw        = ideal.roll;       % rate loop only (Ka unused)
ideal.yaw.Ka     = 0;
ideal.yaw.Kp     = 156;
ideal.yaw.Ki     = 934;
ideal.yaw.Kd     = 0;                % board: yaw D built but not summed (R413 DNP)

ideal.u_max = 400;                   % us, PID output clip per axis (board rails ~ +-2 V * 200 us/V)

%% Arm, integrator hold, throttle and mixer
ideal.us_off   = quad.esc.pwm_min_us;                          % 1000 us while disarmed
ideal.us_max   = quad.esc.pwm_max_us;                          % 2000 us
ideal.us_idle  = quad.esc.pwm_min_us + quad.esc.idle_duty*(quad.esc.pwm_max_us - quad.esc.pwm_min_us); % 1050 us
ideal.i_hold_thr = 0.05;             % integrators held at 0 while throttle stick < this (board I_HOLD, assumed)
% Quad-X mixer, rows M1..M4 (RR FR RL FL), columns [roll pitch yaw] in THIS
% model's sign convention (phi>0 right down, theta>0 nose up, r>0 nose right):
%   roll+  -> left motors (M3, M4) up;  pitch+ -> front motors (M2, M4) up;
%   yaw+   -> CCW motors (M2, M3) up (their reaction torque is +z).
% NOTE the pitch column is the NEGATIVE of the board spec's (Betaflight)
% column, whose pitch axis is nose-down positive.
ideal.mix = [ -1  -1  -1 ;   % M1 rear-right  CW
              -1  +1  +1 ;   % M2 front-right CCW
              +1  -1  +1 ;   % M3 rear-left   CCW
              +1  +1  -1 ];  % M4 front-left  CW
