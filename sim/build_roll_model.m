%BUILD_ROLL_MODEL Build roll_axis.slx, the roll axis of the analog PID board.
%   Rebuilds the model from scratch next to this file. Blocks reference
%   the workspace variables defined in roll_params.m. All controller
%   voltages are relative to VG = 2.048 V.
%
%   Top level: command path -> Roll PID cell -> Mixer + ESC -> Airframe,
%   with the sensor path (DLPF delay, firmware hold, DAC, RC) feeding
%   back. bench_mode = 1 swaps the feedback to the U202D 22 ms bench
%   integrator; angle_loop_on = 0 takes the rate setpoint from rate_sp.

here = fileparts(mfilename('fullpath'));
mdl  = 'roll_axis';
if bdIsLoaded(mdl), close_system(mdl, 0); end
slx = fullfile(here, [mdl '.slx']);
if isfile(slx), delete(slx); end

roll_params;
new_system(mdl);
set_param(mdl, 'Solver', 'ode45', 'MaxStep', '1e-4', 'StopTime', 't_stop', ...
    'ReturnWorkspaceOutputs', 'on', ...
    'PreLoadFcn', 'if ~exist(''RV_P'',''var''), roll_params; end');

%% Command path: theta_cmd [deg] -> firmware -> DAC -> RC
cp = newSub(mdl, 'Command path', [220 40 340 100], {'theta_cmd_deg'}, {'theta_cmd_V'});
add(cp, 'simulink/Math Operations/Gain', 'Ktheta', 'Gain', 'Ktheta');
dacChain(cp, 'cmd', 'Ktheta/1', 'theta_cmd_V/1');

%% Sensor path: true rate and angle -> DLPF delay -> firmware -> DAC -> RC
sp = newSub(mdl, 'Sensor path', [1200 320 1320 420], {'omega_rad_s', 'theta_rad'}, {'V_gyro', 'V_theta'});
add(sp, 'simulink/Sources/Band-Limited White Noise', 'Gyro noise', ...
    'Cov', 'gyro_noise_on*(gyro_nd*pi/180)^2', 'Ts', 'noise_Ts', 'seed', '23341');
add(sp, 'simulink/Math Operations/Sum', 'Add noise', 'Inputs', '++');
add(sp, 'simulink/Math Operations/Gain', 'Kw', 'Gain', 'Kw');
add(sp, 'simulink/Continuous/Transport Delay', 'DLPF delay (gyro)', 'DelayTime', 't_dlpf');
wire(sp, 'omega_rad_s/1', 'Add noise/1');
wire(sp, 'Gyro noise/1', 'Add noise/2');
wire(sp, 'Add noise/1', 'Kw/1');
wire(sp, 'Kw/1', 'DLPF delay (gyro)/1');
dacChain(sp, 'gyro', 'DLPF delay (gyro)/1', 'V_gyro/1');
add(sp, 'simulink/Math Operations/Gain', 'Ktheta per rad', 'Gain', 'Ktheta*180/pi');
add(sp, 'simulink/Continuous/Transport Delay', 'DLPF delay (angle)', 'DelayTime', 't_dlpf');
wire(sp, 'theta_rad/1', 'Ktheta per rad/1');
wire(sp, 'Ktheta per rad/1', 'DLPF delay (angle)/1');
dacChain(sp, 'angle', 'DLPF delay (angle)/1', 'V_theta/1');

%% Roll PID cell (U201, U202)
pc = newSub(mdl, 'Roll PID cell', [440 40 600 260], ...
    {'theta_cmd_V', 'theta_meas_V', 'rate_sp_ext_V', 'FB_V'}, {'U', 'E', 'P', 'I', 'D', 'SP'});
% Stage 6: angle error, inputs swapped (E_A = theta_meas - theta_cmd)
add(pc, 'simulink/Math Operations/Gain', 'g_ang cmd', 'Gain', 'g_ang');
add(pc, 'simulink/Math Operations/Gain', 'g_ang meas', 'Gain', 'g_ang');
add(pc, 'simulink/Math Operations/Sum', 'U202B angle diff amp', 'Inputs', '+-');
rails(pc, 'U202B rails');
wire(pc, 'theta_meas_V/1', 'g_ang meas/1');
wire(pc, 'theta_cmd_V/1', 'g_ang cmd/1');
wire(pc, 'g_ang meas/1', 'U202B angle diff amp/1');
wire(pc, 'g_ang cmd/1', 'U202B angle diff amp/2');
wire(pc, 'U202B angle diff amp/1', 'U202B rails/1');
% Stage 7: angle P, -Go
add(pc, 'simulink/Math Operations/Gain', 'U202C angle P (-Go)', 'Gain', '-(10e3 + RV_A)/20e3');
rails(pc, 'U202C rails');
wire(pc, 'U202B rails/1', 'U202C angle P (-Go)/1');
wire(pc, 'U202C angle P (-Go)/1', 'U202C rails/1');
% Setpoint source: angle loop or external rate setpoint
add(pc, 'simulink/Sources/Constant', 'angle_loop_on', 'Value', 'angle_loop_on');
add(pc, 'simulink/Signal Routing/Switch', 'Setpoint select', ...
    'Criteria', 'u2 >= Threshold', 'Threshold', '0.5');
wire(pc, 'U202C rails/1', 'Setpoint select/1');
wire(pc, 'angle_loop_on/1', 'Setpoint select/2');
wire(pc, 'rate_sp_ext_V/1', 'Setpoint select/3');
wire(pc, 'Setpoint select/1', 'SP/1');
% Stage 1: rate error E = g_sp*SP - g_fb*FB
add(pc, 'simulink/Math Operations/Gain', 'g_sp', 'Gain', 'g_sp');
add(pc, 'simulink/Math Operations/Gain', 'g_fb', 'Gain', 'g_fb');
add(pc, 'simulink/Math Operations/Sum', 'U201A rate diff amp', 'Inputs', '+-');
rails(pc, 'U201A rails');
wire(pc, 'Setpoint select/1', 'g_sp/1');
wire(pc, 'FB_V/1', 'g_fb/1');
wire(pc, 'g_sp/1', 'U201A rate diff amp/1');
wire(pc, 'g_fb/1', 'U201A rate diff amp/2');
wire(pc, 'U201A rate diff amp/1', 'U201A rails/1');
wire(pc, 'U201A rails/1', 'E/1');
% Stage 2: P, -Kp
add(pc, 'simulink/Math Operations/Gain', 'U201B P (-Kp)', 'Gain', '-en_P*(4.7e3 + RV_P)/47e3');
rails(pc, 'U201B rails');
wire(pc, 'U201A rails/1', 'U201B P (-Kp)/1');
wire(pc, 'U201B P (-Kp)/1', 'U201B rails/1');
wire(pc, 'U201B rails/1', 'P/1');
% Stage 3: leaky I with diode clamp, dx/dt = -Ki*E - x/(RL*CI), |x| <= V_iclamp
add(pc, 'simulink/Math Operations/Gain', 'U201C input (-Ki)', 'Gain', '-en_I/((RI_fixed + RV_I)*CI)');
add(pc, 'simulink/Math Operations/Gain', 'U201C leak (RL)', 'Gain', '1/(RL*CI)');
add(pc, 'simulink/Math Operations/Sum', 'U201C node', 'Inputs', '+-');
add(pc, 'simulink/Continuous/Integrator', 'U201C CI + clamp', ...
    'LimitOutput', 'on', 'UpperSaturationLimit', 'V_iclamp', 'LowerSaturationLimit', '-V_iclamp', ...
    'ExternalReset', 'level', 'InitialCondition', '0');
add(pc, 'simulink/Sources/Clock', 'Clock');
add(pc, 'simulink/Logic and Bit Operations/Compare To Constant', 'I_HOLD (t < t_arm)', ...
    'relop', '<', 'const', 't_arm');
rails(pc, 'U201C rails');
wire(pc, 'U201A rails/1', 'U201C input (-Ki)/1');
wire(pc, 'U201C input (-Ki)/1', 'U201C node/1');
wire(pc, 'U201C leak (RL)/1', 'U201C node/2');
wire(pc, 'U201C node/1', 'U201C CI + clamp/1');
wire(pc, 'Clock/1', 'I_HOLD (t < t_arm)/1');
wire(pc, 'I_HOLD (t < t_arm)/1', 'U201C CI + clamp/2');
wire(pc, 'U201C CI + clamp/1', 'U201C leak (RL)/1');
wire(pc, 'U201C CI + clamp/1', 'U201C rails/1');
wire(pc, 'U201C rails/1', 'I/1');
% Stage 4: band-limited D on error, -s*Rd*Cd/((1 + s*Rs*Cd)(1 + s*Rd*Cf))
add(pc, 'simulink/Continuous/Transfer Fcn', 'U201D D (band-limited)', ...
    'Numerator', '[-en_D*(Rd_fixed + RV_D)*Cd 0]', ...
    'Denominator', 'conv([Rs*Cd 1], [(Rd_fixed + RV_D)*Cf 1])');
rails(pc, 'U201D rails');
wire(pc, 'U201A rails/1', 'U201D D (band-limited)/1');
wire(pc, 'U201D D (band-limited)/1', 'U201D rails/1');
wire(pc, 'U201D rails/1', 'D/1');
% Stage 5: unity summer, -(P + I + D)
add(pc, 'simulink/Math Operations/Sum', 'U202A summer', 'Inputs', '+++');
add(pc, 'simulink/Math Operations/Gain', 'U202A invert', 'Gain', '-1');
rails(pc, 'U202A rails');
wire(pc, 'U201B rails/1', 'U202A summer/1');
wire(pc, 'U201C rails/1', 'U202A summer/2');
wire(pc, 'U201D rails/1', 'U202A summer/3');
wire(pc, 'U202A summer/1', 'U202A invert/1');
wire(pc, 'U202A invert/1', 'U202A rails/1');
wire(pc, 'U202A rails/1', 'U/1');

%% Mixer + ESC: roll column of the quad-X mixer (left +1, right -1)
mx = newSub(mdl, 'Mixer + ESC', [700 40 820 100], {'U'}, {'torque_Nm', 'thrust_N'});
add(mx, 'simulink/Discontinuities/Quantizer', 'Mixer ADC (10-bit)', 'QuantizationInterval', 'mix_adc_lsb');
add(mx, 'simulink/Continuous/Transport Delay', 'Mixer + ESC delay', 'DelayTime', 't_esc');
add(mx, 'simulink/Math Operations/Gain', 'KF x a_i', 'Gain', '[1; -1]*KF', ...
    'Multiplication', 'Element-wise(K.*u)');
add(mx, 'simulink/Sources/Constant', 'T_hover', 'Value', 'T_hover');
add(mx, 'simulink/Math Operations/Sum', 'Thrust cmd', 'Inputs', '++');
add(mx, 'simulink/Discontinuities/Saturation', 'Motor limits (idle to full)', ...
    'UpperLimit', 'T_max', 'LowerLimit', 'T_min');
add(mx, 'simulink/Math Operations/Sum', 'Motor lag error', 'Inputs', '+-');
add(mx, 'simulink/Math Operations/Gain', '1 over tau_m', 'Gain', '1/max(tau_m, 1e-5)');
add(mx, 'simulink/Continuous/Integrator', 'Motor thrust', 'InitialCondition', 'T_hover');
add(mx, 'simulink/Math Operations/Gain', 'Roll arm (sqrt2 L)', 'Gain', 'sqrt(2)*L*[1 -1]', ...
    'Multiplication', 'Matrix(K*u)');
wire(mx, 'U/1', 'Mixer ADC (10-bit)/1');
wire(mx, 'Mixer ADC (10-bit)/1', 'Mixer + ESC delay/1');
wire(mx, 'Mixer + ESC delay/1', 'KF x a_i/1');
wire(mx, 'KF x a_i/1', 'Thrust cmd/1');
wire(mx, 'T_hover/1', 'Thrust cmd/2');
wire(mx, 'Thrust cmd/1', 'Motor limits (idle to full)/1');
wire(mx, 'Motor limits (idle to full)/1', 'Motor lag error/1');
wire(mx, 'Motor thrust/1', 'Motor lag error/2');
wire(mx, 'Motor lag error/1', '1 over tau_m/1');
wire(mx, '1 over tau_m/1', 'Motor thrust/1');
wire(mx, 'Motor thrust/1', 'Roll arm (sqrt2 L)/1');
wire(mx, 'Roll arm (sqrt2 L)/1', 'torque_Nm/1');
wire(mx, 'Motor thrust/1', 'thrust_N/1');

%% Airframe: J*domega/dt = torque + dist - b*omega - mgh*sin(theta)
af = newSub(mdl, 'Airframe', [920 40 1040 100], {'torque_Nm', 'dist_Nm'}, {'omega_rad_s', 'theta_rad'});
add(af, 'simulink/Math Operations/Sum', 'Torque sum', 'Inputs', '++--');
add(af, 'simulink/Math Operations/Gain', '1 over J', 'Gain', '1/J');
add(af, 'simulink/Continuous/Integrator', 'omega', 'InitialCondition', '0');
add(af, 'simulink/Continuous/Integrator', 'theta', 'InitialCondition', '0');
add(af, 'simulink/Math Operations/Gain', 'Friction b', 'Gain', 'b_fric');
add(af, 'simulink/Math Operations/Trigonometric Function', 'sin', 'Operator', 'sin');
add(af, 'simulink/Math Operations/Gain', 'Pendulum mgh', 'Gain', 'mgh');
wire(af, 'torque_Nm/1', 'Torque sum/1');
wire(af, 'dist_Nm/1', 'Torque sum/2');
wire(af, 'Friction b/1', 'Torque sum/3');
wire(af, 'Pendulum mgh/1', 'Torque sum/4');
wire(af, 'Torque sum/1', '1 over J/1');
wire(af, '1 over J/1', 'omega/1');
wire(af, 'omega/1', 'theta/1');
wire(af, 'omega/1', 'Friction b/1');
wire(af, 'theta/1', 'sin/1');
wire(af, 'sin/1', 'Pendulum mgh/1');
wire(af, 'omega/1', 'omega_rad_s/1');
wire(af, 'theta/1', 'theta_rad/1');

%% Bench plant: U202D 22 ms integrator on the swapped diff-amp legs
bp = newSub(mdl, 'Bench plant U202D', [700 200 820 240], {'U'}, {'FB_bench_V'});
add(bp, 'simulink/Math Operations/Gain', '1 over tau_p', 'Gain', '1/tau_p');
add(bp, 'simulink/Continuous/Integrator', '22k 1uF integrator', 'InitialCondition', '0', ...
    'LimitOutput', 'on', 'UpperSaturationLimit', '-V_rail_lo', 'LowerSaturationLimit', '-V_rail_hi');
wire(bp, 'U/1', '1 over tau_p/1');
wire(bp, '1 over tau_p/1', '22k 1uF integrator/1');
wire(bp, '22k 1uF integrator/1', 'FB_bench_V/1');

%% Top level
add(mdl, 'simulink/Sources/Step', 'theta_cmd step', 'Position', [60 55 100 85], ...
    'Time', 't_step', 'Before', '0', 'After', 'theta_cmd_deg');
add(mdl, 'simulink/Sources/Step', 'rate_sp step', 'Position', [300 160 340 190], ...
    'Time', 't_step', 'Before', '0', 'After', 'rate_sp_V');
add(mdl, 'simulink/Sources/Step', 'Torque disturbance', 'Position', [800 140 840 170], ...
    'Time', 't_dist', 'Before', '0', 'After', 'dist_Nm');
add(mdl, 'simulink/Sources/Constant', 'bench_mode', 'Position', [1100 230 1150 250], ...
    'Value', 'bench_mode');
add(mdl, 'simulink/Signal Routing/Switch', 'FB select', 'Position', [1220 200 1260 260], ...
    'Criteria', 'u2 >= Threshold', 'Threshold', '0.5');
wire(mdl, 'theta_cmd step/1', 'Command path/1');
wire(mdl, 'Command path/1', 'Roll PID cell/1');
wire(mdl, 'Sensor path/2', 'Roll PID cell/2');
wire(mdl, 'rate_sp step/1', 'Roll PID cell/3');
wire(mdl, 'FB select/1', 'Roll PID cell/4');
wire(mdl, 'Roll PID cell/1', 'Mixer + ESC/1');
wire(mdl, 'Roll PID cell/1', 'Bench plant U202D/1');
wire(mdl, 'Mixer + ESC/1', 'Airframe/1');
wire(mdl, 'Torque disturbance/1', 'Airframe/2');
wire(mdl, 'Airframe/1', 'Sensor path/1');
wire(mdl, 'Airframe/2', 'Sensor path/2');
wire(mdl, 'Bench plant U202D/1', 'FB select/1');
wire(mdl, 'bench_mode/1', 'FB select/2');
wire(mdl, 'Sensor path/1', 'FB select/3');

% Logging (timeseries in the SimulationOutput)
logTo(mdl, 'Roll PID cell/1', 'log_U');
logTo(mdl, 'Roll PID cell/2', 'log_E');
logTo(mdl, 'Roll PID cell/4', 'log_I');
logTo(mdl, 'Roll PID cell/5', 'log_D');
logTo(mdl, 'Roll PID cell/6', 'log_SP');
logTo(mdl, 'FB select/1', 'log_FB');
logTo(mdl, 'Mixer + ESC/2', 'log_thrust');
add(mdl, 'simulink/Math Operations/Gain', 'rad2deg rate', 'Gain', '180/pi');
add(mdl, 'simulink/Math Operations/Gain', 'rad2deg angle', 'Gain', '180/pi');
wire(mdl, 'Airframe/1', 'rad2deg rate/1');
wire(mdl, 'Airframe/2', 'rad2deg angle/1');
logTo(mdl, 'rad2deg rate/1', 'log_omega_dps');
logTo(mdl, 'rad2deg angle/1', 'log_theta_deg');
% P output is used by the summer only; terminate the spare outport
add(mdl, 'simulink/Sinks/Terminator', 'P term');
wire(mdl, 'Roll PID cell/3', 'P term/1');

for s = {cp, sp, pc, mx, af, bp, mdl}
    Simulink.BlockDiagram.arrangeSystem(s{1});
end
save_system(mdl, slx);
fprintf('Saved %s\n', slx);

%% Local helpers
function h = add(sys, lib, name, varargin)
h = add_block(lib, [sys '/' name], varargin{:});
end

function wire(sys, src, dst)
add_line(sys, src, dst, 'autorouting', 'smart');
end

function rails(sys, name)
add(sys, 'simulink/Discontinuities/Saturation', name, ...
    'UpperLimit', 'V_rail_hi', 'LowerLimit', 'V_rail_lo');
end

function dacChain(sys, tag, src, dst)
% Firmware ZOH -> MCP4822 clip and 1 mV quantiser -> 1k/1uF RC filter
hold = ['Firmware hold (' tag ')'];
clip = ['DAC clip (' tag ')'];
q    = ['DAC 12-bit (' tag ')'];
rc   = ['RC 159 Hz (' tag ')'];
add(sys, 'simulink/Discrete/Zero-Order Hold', hold, 'SampleTime', 'Ts_fw');
add(sys, 'simulink/Discontinuities/Saturation', clip, 'UpperLimit', 'dac_hi', 'LowerLimit', 'dac_lo');
add(sys, 'simulink/Discontinuities/Quantizer', q, 'QuantizationInterval', 'dac_lsb');
add(sys, 'simulink/Continuous/Transfer Fcn', rc, 'Numerator', '1', 'Denominator', '[tau_rc 1]');
wire(sys, src, [hold '/1']);
wire(sys, [hold '/1'], [clip '/1']);
wire(sys, [clip '/1'], [q '/1']);
wire(sys, [q '/1'], [rc '/1']);
wire(sys, [rc '/1'], dst);
end

function logTo(sys, src, var)
add(sys, 'simulink/Sinks/To Workspace', var, 'VariableName', var, 'SaveFormat', 'Timeseries');
wire(sys, src, [var '/1']);
end

function sub = newSub(sys, name, pos, inNames, outNames)
sub = [sys '/' name];
add_block('simulink/Ports & Subsystems/Subsystem', sub, 'Position', pos);
delete_line(sub, 'In1/1', 'Out1/1');
delete_block([sub '/In1']);
delete_block([sub '/Out1']);
for k = 1:numel(inNames)
    add_block('simulink/Sources/In1', [sub '/' inNames{k}]);
end
for k = 1:numel(outNames)
    add_block('simulink/Sinks/Out1', [sub '/' outNames{k}]);
end
end
