function pid_board_build(which)
%PID_BOARD_BUILD Build the Model 2 (fast) Simulink models of PID_Board rev B.
%   pid_board_build           builds everything below
%   pid_board_build('analog') pid_analog_fast.slx   analog section (referenced, continuous)
%   pid_board_build('board')  pid_board.slx         CtrlIn -> esc_us board model (referenced)
%   pid_board_build('bench')  pid_board_bench.slx   standalone bench harness
%   pid_board_build('tb')     pid_board_tb_analog.slx + pid_board_cl.slx (validation rigs)
%
%   Every block parameter is an expression of the base-workspace struct pcb
%   (pid_board_params.m) evaluated through subsystem masks, so pot, jumper
%   and firmware changes take effect at the next simulation without a
%   rebuild. Harness inputs come from the base struct pbsim.
%   See README_model2.md and ANALOG_INTERFACE.md.

if nargin < 1, which = 'all'; end
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here, '..', 'common'));
if ~evalin('base', 'exist(''pcb'', ''var'')'), evalin('base', 'pid_board_params;'); end
if ~evalin('base', 'exist(''CtrlIn'', ''var'')'), evalin('base', 'quad_ctrl_buses;'); end
if any(strcmp(which, {'all', 'analog'})), buildAnalog(here); end
if any(strcmp(which, {'all', 'board'})),  buildBoard(here);  end
if any(strcmp(which, {'all', 'bench'})),  buildBench(here);  end
if any(strcmp(which, {'all', 'tb'})),     buildTbAnalog(here); buildCl(here); end
end

%% ======================================================================
function buildAnalog(here)
mdl = 'pid_analog_fast';
startModel(mdl, here);
setCommonConfig(mdl);
inNames = {'ROLL_ANGLE','ROLL_RATE','PITCH_ANGLE','PITCH_RATE','YAW_RATE', ...
    'YAW_RATE_CMD','ROLL_ANGLE_CMD','PITCH_ANGLE_CMD','I_HOLD', ...
    'EXT_SP_ROLL','EXT_FB_ROLL','EXT_SP_PITCH','EXT_FB_PITCH','EXT_SP_YAW','EXT_FB_YAW'};
outNames = {'U_ROLL','U_PITCH','U_YAW','VGND_A','PLANT_OUT_ROLL','PLANT_OUT_PITCH'};
for k = 1:numel(inNames)
    add(mdl, 'simulink/Sources/In1', inNames{k}, 'Port', num2str(k), 'OutDataTypeStr', 'double');
end
for k = 1:numel(outNames)
    add(mdl, 'simulink/Sinks/Out1', outNames{k}, 'Port', num2str(k));
end

% Reference: TL431 + U101 followers -> VGND_A/B/C (ideal: all = VG)
ref = newSub(mdl, 'Reference U101 U102', {}, {'VGND'});
add(ref, 'simulink/Sources/Constant', 'VSET follower', 'Value', 'vg', 'SampleTime', '0');
wire(ref, 'VSET follower/1', 'VGND/1');
addMask(ref, {'brd', 'pcb'}, maskInitCommon());

cells = {'Roll cell U201 U202', 'pcb.roll', 'roll', true;
         'Pitch cell U301 U302', 'pcb.pitch', 'pitch', true;
         'Yaw cell U401 U402', 'pcb.yaw', 'yaw', false};
for k = 1:3
    buildCell(mdl, cells{k, :});
end
% Wiring
r = cells{1, 1}; p = cells{2, 1}; y = cells{3, 1};
wire(mdl, 'ROLL_ANGLE/1', [r '/1']);  wire(mdl, 'ROLL_RATE/1', [r '/2']);
wire(mdl, 'ROLL_ANGLE_CMD/1', [r '/3']); wire(mdl, 'I_HOLD/1', [r '/4']);
wire(mdl, 'EXT_SP_ROLL/1', [r '/5']); wire(mdl, 'EXT_FB_ROLL/1', [r '/6']);
wire(mdl, 'PITCH_ANGLE/1', [p '/1']); wire(mdl, 'PITCH_RATE/1', [p '/2']);
wire(mdl, 'PITCH_ANGLE_CMD/1', [p '/3']); wire(mdl, 'I_HOLD/1', [p '/4']);
wire(mdl, 'EXT_SP_PITCH/1', [p '/5']); wire(mdl, 'EXT_FB_PITCH/1', [p '/6']);
wire(mdl, 'YAW_RATE/1', [y '/1']); wire(mdl, 'YAW_RATE_CMD/1', [y '/2']);
wire(mdl, 'I_HOLD/1', [y '/3']);
wire(mdl, 'EXT_SP_YAW/1', [y '/4']); wire(mdl, 'EXT_FB_YAW/1', [y '/5']);
wire(mdl, [r '/1'], 'U_ROLL/1'); wire(mdl, [p '/1'], 'U_PITCH/1'); wire(mdl, [y '/1'], 'U_YAW/1');
wire(mdl, [r '/2'], 'PLANT_OUT_ROLL/1'); wire(mdl, [p '/2'], 'PLANT_OUT_PITCH/1');
wire(mdl, 'Reference U101 U102/1', 'VGND_A/1');
set_param(mdl, 'Description', ['PID_Board rev B analog section, behavioural (fast) model. ' ...
    'Ports: absolute node voltages, see ANALOG_INTERFACE.md. Parameters: pcb (pid_board_params.m).']);
finishModel(mdl, here);
end

function s = maskInitCommon()
s = ['vg = brd.ref.ideal*brd.VG + (1 - brd.ref.ideal)*brd.ref.V_tl431*brd.ref.RV_set_wiper_gnd/brd.ref.RV_set;' newline ...
     'rlo = brd.V_rail_lo + brd.VG - vg;' newline ...
     'rhi = brd.V_rail_hi + brd.VG - vg;' newline];
end

function buildCell(mdl, name, axExpr, tag, hasAngle)
if hasAngle
    ins = {'ANGLE', 'RATE', 'ANGLE_CMD', 'I_HOLD', 'EXT_SP', 'EXT_FB'};
    outs = {'U', 'PLANT_OUT'};
else
    ins = {'RATE', 'RATE_CMD', 'I_HOLD', 'EXT_SP', 'EXT_FB'};
    outs = {'U'};
end
c = newSub(mdl, name, ins, outs);
init = [maskInitCommon() ...
    '% Stage 1 diff amp: SP leg gain depends on both source impedances (jumper dependent)' newline ...
    'jpsp = ax.JP_SP; jpfb = ax.JP_FB;' newline ...
    'rsp = (jpsp == 1)*ax.R_src_sp; rfb = (jpfb == 1)*ax.R_src_fb;' newline ...
    'gsp = ax.R_dsp_ref/(ax.R_dsp_in + rsp + ax.R_dsp_ref)*(1 + ax.R_dfb_f/(ax.R_dfb_in + rfb));' newline ...
    'gfb = ax.R_dfb_f/(ax.R_dfb_in + ax.R_src_fb);' newline ...
    'gfbx = ax.R_dfb_f/ax.R_dfb_in;' newline ...
    'tauInv = brd.dac.tau_inv_leg; tauNi = brd.dac.tau_ni_leg;' newline ...
    '% Stage 2 P, 3 leaky I + clamp + CD4066, 4 band-limited D' newline ...
    'Kp = (ax.R_P_fixed + ax.RV_P)/ax.R_P_in;' newline ...
    'kI = 1/((ax.R_I_fixed + ax.RV_I)*ax.CI);' newline ...
    'wL = 1/(ax.RL*ax.CI);' newline ...
    'wH = 1/(brd.ihold.R_on*ax.CI);' newline ...
    'Vcl = min([ax.V_iclamp, rhi, -rlo]);' newline ...
    'Rd = ax.Rd_fixed + ax.RV_D;' newline ...
    'numD = [-Rd*ax.Cd 0];' newline ...
    'denD = conv([ax.Rs*ax.Cd 1], [Rd*ax.Cf 1]);' newline ...
    'enP = ax.en_P; enI = ax.en_I; enD = ax.en_D;' newline ...
    'Vth = brd.VDD/2;' newline];
if hasAngle
    init = [init ...
        '% Stage 6 angle diff (swapped legs), 7 angle P, 8 bench plant' newline ...
        'gac = ax.R_a_f/(ax.R_acmd_in + ax.R_src_ang);' newline ...
        'gam = ax.R_a_ref/(ax.R_ameas_in + ax.R_src_ang + ax.R_a_ref)*(1 + ax.R_a_f/(ax.R_acmd_in + ax.R_src_ang));' newline ...
        'Go = (ax.R_A_fixed + ax.RV_A)/ax.R_A_in;' newline ...
        'kPl = 1/(ax.R_plant*ax.C_plant);' newline];
end
addMask(c, {'brd', 'pcb'; 'ax', axExpr}, init);

% Feedback leg: LOOP_FB jumper (1 = gyro DAC via loaded RC, 2 = EXT_FB direct)
add(c, 'simulink/Math Operations/Bias', 'rel RATE', 'Bias', '-vg');
add(c, 'simulink/Continuous/Transfer Fcn', 'RC RATE', 'Numerator', '1', 'Denominator', '[tauInv 1]');
add(c, 'simulink/Math Operations/Gain', 'g_fb', 'Gain', 'gfb');
add(c, 'simulink/Math Operations/Bias', 'rel EXT_FB', 'Bias', '-vg');
add(c, 'simulink/Math Operations/Gain', 'g_fb_ext', 'Gain', 'gfbx');
add(c, 'simulink/Sources/Constant', 'JP_FB', 'Value', 'jpfb');
add(c, 'simulink/Signal Routing/Switch', 'LOOP_FB jumper', 'Criteria', 'u2 >= Threshold', 'Threshold', '1.5');
wire(c, 'RATE/1', 'rel RATE/1'); wire(c, 'rel RATE/1', 'RC RATE/1'); wire(c, 'RC RATE/1', 'g_fb/1');
wire(c, 'EXT_FB/1', 'rel EXT_FB/1'); wire(c, 'rel EXT_FB/1', 'g_fb_ext/1');
wire(c, 'g_fb_ext/1', 'LOOP_FB jumper/1'); wire(c, 'JP_FB/1', 'LOOP_FB jumper/2'); wire(c, 'g_fb/1', 'LOOP_FB jumper/3');

% Setpoint source
if hasAngle
    add(c, 'simulink/Math Operations/Bias', 'rel ANGLE', 'Bias', '-vg');
    add(c, 'simulink/Continuous/Transfer Fcn', 'RC ANGLE', 'Numerator', '1', 'Denominator', '[tauNi 1]');
    add(c, 'simulink/Math Operations/Gain', 'g_ang_meas', 'Gain', 'gam');
    add(c, 'simulink/Math Operations/Bias', 'rel ANGLE_CMD', 'Bias', '-vg');
    add(c, 'simulink/Continuous/Transfer Fcn', 'RC ANGLE_CMD', 'Numerator', '1', 'Denominator', '[tauInv 1]');
    add(c, 'simulink/Math Operations/Gain', 'g_ang_cmd', 'Gain', 'gac');
    add(c, 'simulink/Math Operations/Sum', 'angle diff amp', 'Inputs', '+-');
    rails(c, 'rails E_A');
    add(c, 'simulink/Math Operations/Gain', 'angle P -Go', 'Gain', '-Go');
    rails(c, 'rails RATE_SP');
    wire(c, 'ANGLE/1', 'rel ANGLE/1'); wire(c, 'rel ANGLE/1', 'RC ANGLE/1'); wire(c, 'RC ANGLE/1', 'g_ang_meas/1');
    wire(c, 'ANGLE_CMD/1', 'rel ANGLE_CMD/1'); wire(c, 'rel ANGLE_CMD/1', 'RC ANGLE_CMD/1');
    wire(c, 'RC ANGLE_CMD/1', 'g_ang_cmd/1');
    wire(c, 'g_ang_meas/1', 'angle diff amp/1'); wire(c, 'g_ang_cmd/1', 'angle diff amp/2');
    wire(c, 'angle diff amp/1', 'rails E_A/1'); wire(c, 'rails E_A/1', 'angle P -Go/1');
    wire(c, 'angle P -Go/1', 'rails RATE_SP/1');
    spSrc = 'rails RATE_SP/1';
    logsig(c, 'rails E_A', 1, [tag '_E_A']);
else
    add(c, 'simulink/Math Operations/Bias', 'rel RATE_CMD', 'Bias', '-vg');
    add(c, 'simulink/Continuous/Transfer Fcn', 'RC RATE_CMD', 'Numerator', '1', 'Denominator', '[tauNi 1]');
    wire(c, 'RATE_CMD/1', 'rel RATE_CMD/1'); wire(c, 'rel RATE_CMD/1', 'RC RATE_CMD/1');
    spSrc = 'RC RATE_CMD/1';
end
add(c, 'simulink/Math Operations/Bias', 'rel EXT_SP', 'Bias', '-vg');
add(c, 'simulink/Sources/Constant', 'JP_SP', 'Value', 'jpsp');
add(c, 'simulink/Signal Routing/Switch', 'LOOP_SP jumper', 'Criteria', 'u2 >= Threshold', 'Threshold', '1.5');
wire(c, 'EXT_SP/1', 'rel EXT_SP/1');
wire(c, 'rel EXT_SP/1', 'LOOP_SP jumper/1'); wire(c, 'JP_SP/1', 'LOOP_SP jumper/2'); wire(c, spSrc, 'LOOP_SP jumper/3');
if hasAngle, logsig(c, 'rails RATE_SP', 1, [tag '_RATE_SP']); end

% Stage 1: rate diff amp E = g_sp*SP - g_fb*FB
add(c, 'simulink/Math Operations/Gain', 'g_sp', 'Gain', 'gsp');
add(c, 'simulink/Math Operations/Sum', 'rate diff amp', 'Inputs', '+-');
rails(c, 'rails E');
wire(c, 'LOOP_SP jumper/1', 'g_sp/1'); wire(c, 'g_sp/1', 'rate diff amp/1');
wire(c, 'LOOP_FB jumper/1', 'rate diff amp/2'); wire(c, 'rate diff amp/1', 'rails E/1');
% Stage 2: P
add(c, 'simulink/Math Operations/Gain', 'P -Kp', 'Gain', '-Kp');
rails(c, 'rails P');
wire(c, 'rails E/1', 'P -Kp/1'); wire(c, 'P -Kp/1', 'rails P/1');
% Stage 3: leaky I, diode clamp, CD4066 reset through Ron
add(c, 'simulink/Math Operations/Gain', 'I input -Ki', 'Gain', '-kI');
add(c, 'simulink/Math Operations/Gain', 'I leak RL', 'Gain', 'wL');
add(c, 'simulink/Logic and Bit Operations/Compare To Constant', 'CD4066 closed', 'relop', '>=', 'const', 'Vth');
add(c, 'simulink/Signal Attributes/Data Type Conversion', 'to double', 'OutDataTypeStr', 'double');
add(c, 'simulink/Math Operations/Product', 'switch current', 'Inputs', '2');
add(c, 'simulink/Math Operations/Gain', 'I reset Ron', 'Gain', 'wH');
add(c, 'simulink/Math Operations/Sum', 'I node', 'Inputs', '+--');
add(c, 'simulink/Continuous/Integrator', 'CI with diode clamp', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'Vcl', 'LowerSaturationLimit', '-Vcl', 'InitialCondition', '0');
rails(c, 'rails I');
wire(c, 'rails E/1', 'I input -Ki/1'); wire(c, 'I input -Ki/1', 'I node/1');
wire(c, 'I leak RL/1', 'I node/2'); wire(c, 'I reset Ron/1', 'I node/3');
wire(c, 'I node/1', 'CI with diode clamp/1');
wire(c, 'CI with diode clamp/1', 'I leak RL/1');
wire(c, 'I_HOLD/1', 'CD4066 closed/1'); wire(c, 'CD4066 closed/1', 'to double/1');
wire(c, 'to double/1', 'switch current/1'); wire(c, 'CI with diode clamp/1', 'switch current/2');
wire(c, 'switch current/1', 'I reset Ron/1');
wire(c, 'CI with diode clamp/1', 'rails I/1');
% Stage 4: band-limited D
add(c, 'simulink/Continuous/Transfer Fcn', 'D band limited', 'Numerator', 'numD', 'Denominator', 'denD');
rails(c, 'rails D');
wire(c, 'rails E/1', 'D band limited/1'); wire(c, 'D band limited/1', 'rails D/1');
% Stage 5: summer U = -(P + I + D), en_* = summing resistor fitted
add(c, 'simulink/Math Operations/Gain', 'R sum P', 'Gain', 'enP');
add(c, 'simulink/Math Operations/Gain', 'R sum I', 'Gain', 'enI');
add(c, 'simulink/Math Operations/Gain', 'R sum D', 'Gain', 'enD');
add(c, 'simulink/Math Operations/Sum', 'summer', 'Inputs', '---');
rails(c, 'rails U');
add(c, 'simulink/Math Operations/Bias', 'abs U', 'Bias', 'vg');
wire(c, 'rails P/1', 'R sum P/1'); wire(c, 'rails I/1', 'R sum I/1'); wire(c, 'rails D/1', 'R sum D/1');
wire(c, 'R sum P/1', 'summer/1'); wire(c, 'R sum I/1', 'summer/2'); wire(c, 'R sum D/1', 'summer/3');
wire(c, 'summer/1', 'rails U/1'); wire(c, 'rails U/1', 'abs U/1'); wire(c, 'abs U/1', 'U/1');
logsig(c, 'rails E', 1, [tag '_E']);
logsig(c, 'rails P', 1, [tag '_P']);
logsig(c, 'rails I', 1, [tag '_I']);
logsig(c, 'rails D', 1, [tag '_D']);
logsig(c, 'rails U', 1, [tag '_U']);
if hasAngle
    % Stage 8: bench plant U202D, -1/(22k*1uF s), rails, no reset
    add(c, 'simulink/Math Operations/Gain', 'bench plant -1 over tau', 'Gain', '-kPl');
    add(c, 'simulink/Continuous/Integrator', 'C204 integrator', 'LimitOutput', 'on', ...
        'UpperSaturationLimit', 'rhi', 'LowerSaturationLimit', 'rlo', 'InitialCondition', '0');
    add(c, 'simulink/Math Operations/Bias', 'abs PLANT_OUT', 'Bias', 'vg');
    wire(c, 'rails U/1', 'bench plant -1 over tau/1');
    wire(c, 'bench plant -1 over tau/1', 'C204 integrator/1');
    wire(c, 'C204 integrator/1', 'abs PLANT_OUT/1'); wire(c, 'abs PLANT_OUT/1', 'PLANT_OUT/1');
    logsig(c, 'C204 integrator', 1, [tag '_PLANT']);
end
end

%% ======================================================================
function buildBoard(here)
mdl = 'pid_board';
startModel(mdl, here);
setCommonConfig(mdl);
add(mdl, 'simulink/Sources/In1', 'ctrl_in', 'OutDataTypeStr', 'Bus: CtrlIn', 'SampleTime', '1e-3');  % Model 1 bus rate qsim.Ts_bus
add(mdl, 'simulink/Signal Routing/Bus Selector', 'CtrlIn select', ...
    'OutputSignals', 'gyro_radps,accel_mps2,euler_true,sticks,arm');
add(mdl, 'simulink/Sinks/Out1', 'esc_us');
wire(mdl, 'ctrl_in/1', 'CtrlIn select/1');

%% Shim U1
sh = newSub(mdl, 'Shim U1 ATtiny1614', {'gyro_radps', 'accel_mps2', 'euler_true', 'sticks', 'arm'}, ...
    {'DAC', 'I_HOLD', 'ARM'});
shn = 'Shim U1 ATtiny1614';
addMask(sh, {'brd', 'pcb'}, [ ...
    'Ts = brd.shim.Ts; g0 = 9.80665;' newline ...
    'shp = [brd.shim.Ts, brd.shim.cf_alpha, brd.shim.axis_sign(:).'', brd.shim.angle_src, ' ...
    'brd.scale.Kw, brd.scale.Ktheta, brd.shim.angle_max, brd.shim.yaw_rate_max, ' ...
    'brd.scale.thr_V0, brd.scale.thr_V1, brd.shim.ihold_low_thr, brd.shim.thr_low, ' ...
    'brd.ihold.t_release, brd.VG];' newline ...
    'crp = [brd.shim.crsf_min, brd.shim.crsf_mid, brd.shim.crsf_max];' newline ...
    'nstd = brd.shim.noise_on*brd.shim.gyro_nd*sqrt(1/(2*brd.shim.Ts));' newline]);
% IMU (MPU-6050): DLPF group delay, noise, FS saturation, LSB quantisation
add(sh, 'simulink/Discrete/Delay', 'DLPF delay gyro', 'DelayLength', 'round(brd.shim.t_dlpf/Ts)', 'SampleTime', '-1');
add(sh, 'simulink/Math Operations/Gain', 'rad2deg', 'Gain', '180/pi');
add(sh, 'simulink/Discrete/Zero-Order Hold', 'I2C read gyro', 'SampleTime', 'Ts');
add(sh, 'simulink/Sources/Random Number', 'rate noise', 'Mean', '[0 0 0]', 'Variance', '[1 1 1]', ...
    'Seed', '[23341 23342 23343]', 'SampleTime', 'Ts');
add(sh, 'simulink/Math Operations/Gain', 'noise std', 'Gain', 'nstd');
add(sh, 'simulink/Math Operations/Sum', 'add noise', 'Inputs', '++');
add(sh, 'simulink/Discontinuities/Saturation', 'gyro FS', 'UpperLimit', 'brd.shim.gyro_fs', ...
    'LowerLimit', '-brd.shim.gyro_fs');
add(sh, 'simulink/Discontinuities/Quantizer', 'gyro LSB', 'QuantizationInterval', 'brd.shim.gyro_lsb');
wire(sh, 'gyro_radps/1', 'rad2deg/1'); wire(sh, 'rad2deg/1', 'I2C read gyro/1');
wire(sh, 'I2C read gyro/1', 'DLPF delay gyro/1'); wire(sh, 'DLPF delay gyro/1', 'add noise/1');
wire(sh, 'rate noise/1', 'noise std/1'); wire(sh, 'noise std/1', 'add noise/2');
wire(sh, 'add noise/1', 'gyro FS/1'); wire(sh, 'gyro FS/1', 'gyro LSB/1');
add(sh, 'simulink/Discrete/Delay', 'DLPF delay accel', 'DelayLength', 'round(brd.shim.t_dlpf/Ts)', 'SampleTime', '-1');
add(sh, 'simulink/Math Operations/Gain', 'to g', 'Gain', '1/g0');
add(sh, 'simulink/Discrete/Zero-Order Hold', 'I2C read accel', 'SampleTime', 'Ts');
add(sh, 'simulink/Discontinuities/Saturation', 'accel FS', 'UpperLimit', 'brd.shim.acc_fs', ...
    'LowerLimit', '-brd.shim.acc_fs');
add(sh, 'simulink/Discontinuities/Quantizer', 'accel LSB', 'QuantizationInterval', 'brd.shim.acc_lsb');
wire(sh, 'accel_mps2/1', 'to g/1'); wire(sh, 'to g/1', 'I2C read accel/1');
wire(sh, 'I2C read accel/1', 'DLPF delay accel/1'); wire(sh, 'DLPF delay accel/1', 'accel FS/1');
wire(sh, 'accel FS/1', 'accel LSB/1');
add(sh, 'simulink/Discrete/Zero-Order Hold', 'sample euler_true', 'SampleTime', 'Ts');
wire(sh, 'euler_true/1', 'sample euler_true/1');
% ELRS -> CRSF frames
add(sh, 'simulink/Discrete/Zero-Order Hold', 'CRSF frame sticks', 'SampleTime', 'brd.shim.crsf_Ts');
add(sh, 'simulink/Discrete/Zero-Order Hold', 'CRSF frame arm', 'SampleTime', 'brd.shim.crsf_Ts');
mlfn(sh, 'CRSF channels', crsfCode(), {'crp'}, {'sticks', '[4 1]'; 'st', '[4 1]'});
wire(sh, 'sticks/1', 'CRSF frame sticks/1'); wire(sh, 'CRSF frame sticks/1', 'CRSF channels/1');
wire(sh, 'arm/1', 'CRSF frame arm/1');
% Firmware frame: attitude estimate, scaling, ARM / I_HOLD
add(sh, 'simulink/Discrete/Zero-Order Hold', 'frame sticks', 'SampleTime', 'Ts');
add(sh, 'simulink/Discrete/Zero-Order Hold', 'frame arm', 'SampleTime', 'Ts');
wire(sh, 'CRSF channels/1', 'frame sticks/1'); wire(sh, 'CRSF frame arm/1', 'frame arm/1');
mlfn(sh, 'Shim firmware', shimCode(), {'shp'}, {'gyro_dps', '[3 1]'; 'acc_g', '[3 1]'; 'euler_rad', '[3 1]'; ...
    'st', '[4 1]'; 'arm_in', '1'; 'dac_rel', '[10 1]'; 'arm', '1'; 'ihold', '1'; 'att_deg', '[2 1]'});
wire(sh, 'gyro LSB/1', 'Shim firmware/1'); wire(sh, 'accel LSB/1', 'Shim firmware/2');
wire(sh, 'sample euler_true/1', 'Shim firmware/3'); wire(sh, 'frame sticks/1', 'Shim firmware/4');
wire(sh, 'frame arm/1', 'Shim firmware/5');
% MCP4822 x5: 12-bit code, clip, 1 mV LSB (absolute volts out)
add(sh, 'simulink/Math Operations/Gain', 'V to code', 'Gain', '1/brd.dac.lsb');
add(sh, 'simulink/Math Operations/Bias', 'midscale', 'Bias', 'brd.dac.code0');
add(sh, 'simulink/Math Operations/Rounding Function', 'round code', 'Operator', 'round');
add(sh, 'simulink/Discontinuities/Saturation', 'code clip', 'UpperLimit', '2^brd.dac.bits - 1', 'LowerLimit', '0');
add(sh, 'simulink/Math Operations/Gain', 'MCP4822 VOUT', 'Gain', 'brd.dac.lsb');
wire(sh, 'Shim firmware/1', 'V to code/1'); wire(sh, 'V to code/1', 'midscale/1');
wire(sh, 'midscale/1', 'round code/1'); wire(sh, 'round code/1', 'code clip/1');
wire(sh, 'code clip/1', 'MCP4822 VOUT/1'); wire(sh, 'MCP4822 VOUT/1', 'DAC/1');
add(sh, 'simulink/Math Operations/Gain', 'PA7 I_HOLD', 'Gain', 'brd.VDD');
wire(sh, 'Shim firmware/3', 'PA7 I_HOLD/1'); wire(sh, 'PA7 I_HOLD/1', 'I_HOLD/1');
wire(sh, 'Shim firmware/2', 'ARM/1');
add(sh, 'simulink/Sinks/Terminator', 'att term');
wire(sh, 'Shim firmware/4', 'att term/1');
logsig(sh, 'MCP4822 VOUT', 1, 'dac_V');
logsig(sh, 'Shim firmware', 2, 'ARM');
logsig(sh, 'PA7 I_HOLD', 1, 'I_HOLD');
logsig(sh, 'Shim firmware', 4, 'att_est_deg');

wire(mdl, 'CtrlIn select/1', [shn '/1']); wire(mdl, 'CtrlIn select/2', [shn '/2']);
wire(mdl, 'CtrlIn select/3', [shn '/3']); wire(mdl, 'CtrlIn select/4', [shn '/4']);
wire(mdl, 'CtrlIn select/5', [shn '/5']);

%% DAC channel split (pcb.dac.order)
add(mdl, 'simulink/Signal Routing/Demux', 'DAC channels', 'Outputs', '10');
% DAC pins hold their value between frames: explicit discrete -> continuous transition
add(mdl, 'simulink/Signal Attributes/Rate Transition', 'DAC hold', 'OutPortSampleTime', '0');
add(mdl, 'simulink/Signal Attributes/Rate Transition', 'I_HOLD hold', 'OutPortSampleTime', '0');
wire(mdl, [shn '/1'], 'DAC hold/1'); wire(mdl, 'DAC hold/1', 'DAC channels/1');
wire(mdl, [shn '/2'], 'I_HOLD hold/1');
add(mdl, 'simulink/Sinks/Terminator', 'DAC_SPARE J1-5');
wire(mdl, 'DAC channels/10', 'DAC_SPARE J1-5/1');

%% Bench test-point sources (AD3 W1 generator, PLANT_OUT flying lead)
bs = newSub(mdl, 'Bench test points', {'PLANT_OUT_ROLL', 'PLANT_OUT_PITCH'}, ...
    {'EXT_SP_ROLL', 'EXT_FB_ROLL', 'EXT_SP_PITCH', 'EXT_FB_PITCH', 'EXT_SP_YAW', 'EXT_FB_YAW'});
bsn = 'Bench test points';
addMask(bs, {'brd', 'pcb'}, ['w1p = [brd.VG + brd.bench.w1_offset, brd.bench.w1_amp, ' ...
    'brd.bench.w1_freq, brd.bench.w1_t0];' newline]);
add(bs, 'simulink/Sources/Clock', 'Clock');
mlfn(bs, 'AD3 W1 generator', w1Code(), {'w1p'}, {'t', '1'; 'v', '1'});
add(bs, 'simulink/Sources/Constant', 'VG open', 'Value', 'brd.VG', 'SampleTime', '0');
wire(bs, 'Clock/1', 'AD3 W1 generator/1');
% Flying lead PLANT_OUT -> EXT_SP: a 100 us lag (lead + probe C) also breaks the
% artificial algebraic loop through the referenced analog model.
add(bs, 'simulink/Continuous/State-Space', 'lead ROLL', 'A', '-1/brd.bench.lead_tau', ...
    'B', '1/brd.bench.lead_tau', 'C', '1', 'D', '0', 'InitialCondition', 'brd.VG');
add(bs, 'simulink/Continuous/State-Space', 'lead PITCH', 'A', '-1/brd.bench.lead_tau', ...
    'B', '1/brd.bench.lead_tau', 'C', '1', 'D', '0', 'InitialCondition', 'brd.VG');
wire(bs, 'PLANT_OUT_ROLL/1', 'lead ROLL/1'); wire(bs, 'PLANT_OUT_PITCH/1', 'lead PITCH/1');
axs = {'ROLL', 1, 'lead ROLL/1'; 'PITCH', 2, 'lead PITCH/1'; 'YAW', 3, 'VG open/1'};
for k = 1:3
    a = axs{k, 1}; n = num2str(axs{k, 2});
    add(bs, 'simulink/Sources/Constant', ['sp src ' a], 'Value', ['brd.bench.ext_sp_src(' n ')']);
    add(bs, 'simulink/Sources/Constant', ['fb src ' a], 'Value', ['brd.bench.ext_fb_src(' n ')']);
    add(bs, 'simulink/Signal Routing/Multiport Switch', ['EXT_SP ' a], 'Inputs', '3', ...
        'DataPortOrder', 'Zero-based contiguous');
    add(bs, 'simulink/Signal Routing/Multiport Switch', ['EXT_FB ' a], 'Inputs', '3', ...
        'DataPortOrder', 'Zero-based contiguous');
    wire(bs, ['sp src ' a '/1'], ['EXT_SP ' a '/1']);
    wire(bs, 'VG open/1', ['EXT_SP ' a '/2']);
    wire(bs, axs{k, 3}, ['EXT_SP ' a '/3']);
    wire(bs, 'AD3 W1 generator/1', ['EXT_SP ' a '/4']);
    wire(bs, ['fb src ' a '/1'], ['EXT_FB ' a '/1']);
    wire(bs, 'VG open/1', ['EXT_FB ' a '/2']);
    wire(bs, 'VG open/1', ['EXT_FB ' a '/3']);
    wire(bs, 'AD3 W1 generator/1', ['EXT_FB ' a '/4']);
    wire(bs, ['EXT_SP ' a '/1'], ['EXT_SP_' a '/1']);
    wire(bs, ['EXT_FB ' a '/1'], ['EXT_FB_' a '/1']);
end
logsig(bs, 'AD3 W1 generator', 1, 'W1');

%% Analog section: Variant Subsystem (fast / circuit)
vs = [mdl '/Analog section'];
add_block('simulink/Ports & Subsystems/Variant Subsystem', vs);
old = find_system(vs, 'SearchDepth', 1, 'MatchFilter', @Simulink.match.allVariants);
for k = 2:numel(old), delete_block(old{k}); end
aIn = {'ROLL_ANGLE','ROLL_RATE','PITCH_ANGLE','PITCH_RATE','YAW_RATE', ...
    'YAW_RATE_CMD','ROLL_ANGLE_CMD','PITCH_ANGLE_CMD','I_HOLD', ...
    'EXT_SP_ROLL','EXT_FB_ROLL','EXT_SP_PITCH','EXT_FB_PITCH','EXT_SP_YAW','EXT_FB_YAW'};
aOut = {'U_ROLL','U_PITCH','U_YAW','VGND_A','PLANT_OUT_ROLL','PLANT_OUT_PITCH'};
for k = 1:numel(aIn), add_block('simulink/Sources/In1', [vs '/' aIn{k}], 'Port', num2str(k)); end
for k = 1:numel(aOut), add_block('simulink/Sinks/Out1', [vs '/' aOut{k}], 'Port', num2str(k)); end
add_block('simulink/Ports & Subsystems/Model', [vs '/pid_analog_fast'], ...
    'ModelName', 'pid_analog_fast', 'VariantControl', 'board_variant == 1');
% Circuit choice = inner Variant Subsystem (update diagram, pcb.circuit_enable) so the
% Simscape model is only compiled when wanted: pcb.circuit_enable = 0 lets the fast
% variant run while pid_analog_circuit.slx is missing / not fixed-step compatible.
cv = [vs '/circuit'];
add_block('simulink/Ports & Subsystems/Variant Subsystem', cv, 'VariantControl', 'board_variant == 2');
old = find_system(cv, 'SearchDepth', 1, 'MatchFilter', @Simulink.match.allVariants);
for k = 2:numel(old), delete_block(old{k}); end
for k = 1:numel(aIn), add_block('simulink/Sources/In1', [cv '/' aIn{k}], 'Port', num2str(k)); end
for k = 1:numel(aOut), add_block('simulink/Sinks/Out1', [cv '/' aOut{k}], 'Port', num2str(k)); end
add_block('simulink/Ports & Subsystems/Model', [cv '/pid_analog_circuit'], ...
    'ModelName', 'pid_analog_circuit', 'VariantControl', 'pcb.circuit_enable == 1');
set_param(cv, 'VariantActivationTime', 'update diagram', 'AllowZeroVariantControls', 'on', ...
    'TreatAsAtomicUnit', 'on');
% Model argument board_variant (1 fast, 2 circuit) selects the analog section per
% Model-block instance; model arguments need 'startup' activation (both choices compile).
set_param(vs, 'VariantActivationTime', 'startup');
mws = get_param(mdl, 'ModelWorkspace');
bv = Simulink.Parameter(1); bv.DataType = 'double';
bv.Description = 'Analog section: 1 = pid_analog_fast, 2 = pid_analog_circuit';
assignin(mws, 'board_variant', bv);
set_param(mdl, 'ParameterArgumentNames', 'board_variant');
for k = 1:8, wire(mdl, ['DAC channels/' num2str(k)], ['Analog section/' num2str(k)]); end
wire(mdl, 'I_HOLD hold/1', 'Analog section/9');
for k = 1:6, wire(mdl, [bsn '/' num2str(k)], ['Analog section/' num2str(9 + k)]); end
wire(mdl, 'Analog section/5', [bsn '/1']); wire(mdl, 'Analog section/6', [bsn '/2']);
logsig(mdl, 'Analog section', 5, 'PLANT_OUT_ROLL');
logsig(mdl, 'Analog section', 6, 'PLANT_OUT_PITCH');

%% control_io U3
ci = newSub(mdl, 'control_io U3 ATtiny1614', {'U_ROLL', 'U_PITCH', 'U_YAW', 'VGND_A', 'THROTTLE_DAC', 'ARM'}, ...
    {'esc_us'});
cin = 'control_io U3 ATtiny1614';
addMask(ci, {'brd', 'pcb'}, [ ...
    'Ts = brd.ctrl.Ts;' newline ...
    'cp = [brd.ctrl.adc_lsb, brd.ctrl.adc_bits, brd.scale.thr_V0, brd.scale.thr_V1, ' ...
    'brd.ctrl.us_off, brd.ctrl.us_idle, brd.ctrl.us_max, brd.ctrl.us_res, brd.ctrl.K_rpy(:).''];' newline ...
    'mixm = brd.ctrl.mix;' newline]);
add(ci, 'simulink/Continuous/Transfer Fcn', 'R13 C23 THROTTLE RC', 'Numerator', '1', ...
    'Denominator', '[brd.dac.tau_unloaded 1]');
add(ci, 'simulink/Signal Routing/Mux', 'ADC pins', 'Inputs', '5');
add(ci, 'simulink/Discrete/Zero-Order Hold', 'ADC sample', 'SampleTime', 'Ts');
add(ci, 'simulink/Discrete/Zero-Order Hold', 'PB3 ARM sample', 'SampleTime', 'Ts');
mlfn(ci, 'control_io firmware', ctrlCode(), {'cp', 'mixm'}, {'v', '[5 1]'; 'arm', '1'; 'us', '[4 1]'; 'u_V', '[3 1]'});
add(ci, 'simulink/Discrete/Delay', 'frame delay', 'DelayLength', 'brd.ctrl.delay_frames', ...
    'InitialCondition', 'brd.ctrl.us_off', 'SampleTime', '-1');
wire(ci, 'U_ROLL/1', 'ADC pins/1'); wire(ci, 'U_PITCH/1', 'ADC pins/2'); wire(ci, 'U_YAW/1', 'ADC pins/3');
wire(ci, 'THROTTLE_DAC/1', 'R13 C23 THROTTLE RC/1'); wire(ci, 'R13 C23 THROTTLE RC/1', 'ADC pins/4');
wire(ci, 'VGND_A/1', 'ADC pins/5');
wire(ci, 'ADC pins/1', 'ADC sample/1'); wire(ci, 'ADC sample/1', 'control_io firmware/1');
wire(ci, 'ARM/1', 'PB3 ARM sample/1'); wire(ci, 'PB3 ARM sample/1', 'control_io firmware/2');
wire(ci, 'control_io firmware/1', 'frame delay/1'); wire(ci, 'frame delay/1', 'esc_us/1');
add(ci, 'simulink/Sinks/Terminator', 'adc term');
wire(ci, 'control_io firmware/2', 'adc term/1');
logsig(ci, 'control_io firmware', 2, 'adc_u_V');
logsig(ci, 'R13 C23 THROTTLE RC', 1, 'THROTTLE');

wire(mdl, 'Analog section/1', [cin '/1']); wire(mdl, 'Analog section/2', [cin '/2']);
wire(mdl, 'Analog section/3', [cin '/3']); wire(mdl, 'Analog section/4', [cin '/4']);
wire(mdl, 'DAC channels/9', [cin '/5']); wire(mdl, [shn '/3'], [cin '/6']);
wire(mdl, [cin '/1'], 'esc_us/1');
logsig(mdl, 'Analog section', 1, 'U_ROLL');
logsig(mdl, 'Analog section', 2, 'U_PITCH');
logsig(mdl, 'Analog section', 3, 'U_YAW');
logsig(mdl, 'Analog section', 4, 'VGND_A');
logsig(mdl, cin, 1, 'esc_us');
set_param(mdl, 'Description', ['PID_Board rev B digital twin (Model 2, fast). ctrl_in (CtrlIn bus) -> ' ...
    'shim U1 -> DACs -> analog section (variant pcb.variant) -> control_io U3 -> esc_us [M1 RR, M2 FR, M3 RL, M4 FL].']);
finishModel(mdl, here);
end

%% ======================================================================
function buildBench(here)
% Standalone bench harness: recorded or synthetic IMU/sticks -> board.
mdl = 'pid_board_bench';
startModel(mdl, here);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode4', 'FixedStep', 'pcb.sim_dt', 'StopTime', 'pbsim.t_stop', ...
    'SignalLogging', 'on', 'ReturnWorkspaceOutputs', 'on');
set_param(mdl, 'PostLoadFcn', postLoad(mdl, true));
busSource(mdl, 'pbsim.bench_in');
add(mdl, 'simulink/Ports & Subsystems/Model', 'PID_Board rev B', 'ModelName', 'pid_board');
setVariantArg([mdl '/PID_Board rev B']);
wire(mdl, 'CtrlIn/1', 'PID_Board rev B/1');
add(mdl, 'simulink/Sinks/Scope', 'ESC pulses');
wire(mdl, 'PID_Board rev B/1', 'ESC pulses/1');
logsig(mdl, 'PID_Board rev B', 1, 'esc_us_bench');
set_param(mdl, 'Description', ['Bench harness: pbsim.bench_in from pid_board_bench_inputs. ' ...
    'Test points (U_ROLL, roll_E, roll_I, PLANT_OUT_ROLL, ...) are in logsout / Simulation Data Inspector.']);
finishModel(mdl, here);
end

function busSource(mdl, var)
% From Workspace [t, 19 columns] -> CtrlIn bus (column order in pid_board_bench_inputs)
add(mdl, 'simulink/Sources/From Workspace', 'IMU and sticks', 'VariableName', var, 'SampleTime', '1e-3', ...
    'Interpolate', 'on', 'OutputAfterFinalValue', 'Holding final value');
add(mdl, 'simulink/Signal Routing/Demux', 'split', 'Outputs', '[3 3 3 3 4 1 1]');
add(mdl, 'simulink/Signal Routing/Bus Creator', 'CtrlIn', 'Inputs', '7', ...
    'OutDataTypeStr', 'Bus: CtrlIn', 'InheritFromInputs', 'off');
wire(mdl, 'IMU and sticks/1', 'split/1');
names = {'gyro_radps', 'accel_mps2', 'rates_true', 'euler_true', 'sticks', 'arm', 'vbat_V'};
for k = 1:7
    h = add_line(mdl, ['split/' num2str(k)], ['CtrlIn/' num2str(k)], 'autorouting', 'smart');
    set_param(h, 'Name', names{k});
end
end

%% ======================================================================
function buildTbAnalog(here)
% Analog-only test bench: pbsim.an_in = [t, 15 absolute input voltages]
mdl = 'pid_board_tb_analog';
startModel(mdl, here);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode4', 'FixedStep', 'pcb.sim_dt', 'StopTime', 'pbsim.t_stop', ...
    'SignalLogging', 'on', 'ReturnWorkspaceOutputs', 'on');
set_param(mdl, 'PostLoadFcn', postLoad(mdl, false));
add(mdl, 'simulink/Sources/From Workspace', 'DAC pins and TPs', 'VariableName', 'pbsim.an_in', ...
    'Interpolate', 'on', 'OutputAfterFinalValue', 'Holding final value');
add(mdl, 'simulink/Signal Routing/Demux', 'split', 'Outputs', '15');
add(mdl, 'simulink/Ports & Subsystems/Model', 'analog', 'ModelName', 'pid_analog_fast');
wire(mdl, 'DAC pins and TPs/1', 'split/1');
for k = 1:15, wire(mdl, ['split/' num2str(k)], ['analog/' num2str(k)]); end
on = {'U_ROLL', 'U_PITCH', 'U_YAW', 'VGND_A', 'PLANT_OUT_ROLL', 'PLANT_OUT_PITCH'};
for k = 1:6
    add(mdl, 'simulink/Sinks/Terminator', ['t' num2str(k)]);
    wire(mdl, ['analog/' num2str(k)], ['t' num2str(k) '/1']);
    logsig(mdl, 'analog', k, on{k});
end
finishModel(mdl, here);
end

function buildCl(here)
% Closed loop: pid_board + per-axis rigid-body attitude plant (pbsim.plant)
mdl = 'pid_board_cl';
startModel(mdl, here);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode4', 'FixedStep', 'pcb.sim_dt', 'StopTime', 'pbsim.t_stop', ...
    'SignalLogging', 'on', 'ReturnWorkspaceOutputs', 'on');
set_param(mdl, 'PostLoadFcn', postLoad(mdl, true));
add(mdl, 'simulink/Ports & Subsystems/Model', 'PID_Board rev B', 'ModelName', 'pid_board');
setVariantArg([mdl '/PID_Board rev B']);
pl = newSub(mdl, 'Rigid body attitude plant', {'esc_us', 'sticks_arm'}, {'ctrl_in'});
addMask(pl, {'pl', 'pbsim.plant'}, '');
add(pl, 'simulink/Math Operations/Bias', 'us minus hover', 'Bias', '-pl.us_h');
add(pl, 'simulink/Math Operations/Gain', 'dT per us', 'Gain', 'pl.dTdus');
add(pl, 'simulink/Math Operations/Bias', 'plus hover thrust', 'Bias', 'pl.T_h');
add(pl, 'simulink/Discontinuities/Saturation', 'thrust >= 0', 'UpperLimit', 'pl.T_max', 'LowerLimit', '0');
add(pl, 'simulink/Continuous/Transport Delay', 'ESC latency', 'DelayTime', 'pl.t_lat', ...
    'InitialOutput', 'pl.T_h');
add(pl, 'simulink/Continuous/State-Space', 'motor lag', 'A', '-eye(4)/pl.tau', 'B', 'eye(4)/pl.tau', ...
    'C', 'eye(4)', 'D', 'zeros(4)', 'InitialCondition', 'pl.T_h*ones(4,1)');
add(pl, 'simulink/Continuous/Integrator', 'omega euler', 'InitialCondition', 'pl.x0');
mlfn(pl, 'rigid body', rbCode(), {'pl'}, {'x', '[6 1]'; 'T', '[4 1]'; 'xdot', '[6 1]'; ...
    'omega', '[3 1]'; 'f_body', '[3 1]'; 'eul', '[3 1]'});
add(pl, 'simulink/Signal Routing/Demux', 'sa', 'Outputs', '[4 1]');
add(pl, 'simulink/Sources/Constant', 'vbat', 'Value', 'pl.vbat', 'SampleTime', '0');
add(pl, 'simulink/Signal Routing/Bus Creator', 'CtrlIn', 'Inputs', '7', ...
    'OutDataTypeStr', 'Bus: CtrlIn', 'InheritFromInputs', 'off');
wire(pl, 'esc_us/1', 'us minus hover/1'); wire(pl, 'us minus hover/1', 'dT per us/1');
wire(pl, 'dT per us/1', 'plus hover thrust/1'); wire(pl, 'plus hover thrust/1', 'thrust >= 0/1');
wire(pl, 'thrust >= 0/1', 'ESC latency/1'); wire(pl, 'ESC latency/1', 'motor lag/1');
wire(pl, 'omega euler/1', 'rigid body/1'); wire(pl, 'motor lag/1', 'rigid body/2');
wire(pl, 'rigid body/1', 'omega euler/1');
wire(pl, 'sticks_arm/1', 'sa/1');
names = {'gyro_radps', 'accel_mps2', 'rates_true', 'euler_true', 'sticks', 'arm', 'vbat_V'};
srcs = {'rigid body/2', 'rigid body/3', 'rigid body/2', 'rigid body/4', 'sa/1', 'sa/2', 'vbat/1'};
for k = 1:7
    h = add_line(pl, srcs{k}, ['CtrlIn/' num2str(k)], 'autorouting', 'smart');
    set_param(h, 'Name', names{k});
end
wire(pl, 'CtrlIn/1', 'ctrl_in/1');
logsig(pl, 'rigid body', 4, 'euler_rad');
logsig(pl, 'rigid body', 2, 'omega_radps');
add(mdl, 'simulink/Sources/From Workspace', 'sticks and arm', 'VariableName', 'pbsim.cl_sticks', ...
    'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value');
wire(mdl, 'sticks and arm/1', 'Rigid body attitude plant/2');
add(mdl, 'simulink/Discrete/Zero-Order Hold', 'IMU bus 1 kHz', 'SampleTime', '1e-3');
wire(mdl, 'Rigid body attitude plant/1', 'IMU bus 1 kHz/1');
wire(mdl, 'IMU bus 1 kHz/1', 'PID_Board rev B/1');
wire(mdl, 'PID_Board rev B/1', 'Rigid body attitude plant/1');
logsig(mdl, 'PID_Board rev B', 1, 'esc_cl');
finishModel(mdl, here);
end

%% ======================================================================
%  MATLAB Function block code
function s = crsfCode()
s = strjoin({
'function st = crsf(sticks, crp)'
'%ELRS -> CRSF 11-bit channels (172..992..1811) and back to normalised sticks'
'half = (crp(3) - crp(1))/2;'
'ch = zeros(4, 1);'
'ch(1:3) = round(crp(2) + sticks(1:3)*half);'
'ch(4) = round(crp(1) + sticks(4)*(crp(3) - crp(1)));'
'ch = min(max(ch, crp(1)), crp(3));'
'st = zeros(4, 1);'
'st(1:3) = (ch(1:3) - crp(2))/half;'
'st(4) = (ch(4) - crp(1))/(crp(3) - crp(1));'
}, newline);
end

function s = shimCode()
s = strjoin({
'function [dac_rel, arm, ihold, att_deg] = shim_fw(gyro_dps, acc_g, euler_rad, st, arm_in, shp)'
'%Shim U1 firmware model (ASSUMED behaviour, no PCB firmware exists).'
'% shp = [Ts alpha sgn_r sgn_p sgn_y angle_src Kw Kth angle_max yaw_max thr_V0 thr_V1'
'%        ihold_low thr_low t_release VG]'
'% dac_rel: 10 DAC channels in pcb.dac.order, volts relative to VG.'
'persistent phi th k init'
'Ts = shp(1); a = shp(2); sg = shp(3:5); Kw = shp(7); Kth = shp(8);'
'if isempty(k)'
'    k = 0; phi = 0; th = 0; init = false;'
'end'
'% accelerometer tilt (specific force, FRD): phi = atan2(-fy,-fz), theta = atan2(fx, |fyz|)'
'phi_a = atan2(-acc_g(2), -acc_g(3))*180/pi;'
'th_a = atan2(acc_g(1), sqrt(acc_g(2)^2 + acc_g(3)^2))*180/pi;'
'if ~init  % seed from the accelerometer at boot (first valid sample after the DLPF delay)'
'    phi = phi_a; th = th_a;'
'    init = norm(acc_g) > 0.5;'
'else'
'    phi = a*(phi + gyro_dps(1)*Ts) + (1 - a)*phi_a;'
'    th = a*(th + gyro_dps(2)*Ts) + (1 - a)*th_a;'
'end'
'if shp(6) > 0.5'
'    ang = [phi; th];'
'else'
'    ang = euler_rad(1:2)*180/pi;'
'end'
'thr = min(max(st(4), 0), 1);'
'dac_rel = zeros(10, 1);'
'dac_rel(1) = Kth*sg(1)*ang(1);                 % ROLL_ANGLE'
'dac_rel(2) = Kw*sg(1)*gyro_dps(1);             % ROLL_RATE'
'dac_rel(3) = Kth*sg(2)*ang(2);                 % PITCH_ANGLE'
'dac_rel(4) = Kw*sg(2)*gyro_dps(2);             % PITCH_RATE'
'dac_rel(5) = Kw*sg(3)*gyro_dps(3);             % YAW_RATE'
'dac_rel(6) = Kw*sg(3)*shp(10)*st(3);           % YAW_RATE_CMD   (yaw +1 = nose right)'
'dac_rel(7) = Kth*sg(1)*shp(9)*st(1);           % ROLL_ANGLE_CMD (roll +1 = phi > 0)'
'dac_rel(8) = Kth*sg(2)*(-shp(9)*st(2));        % PITCH_ANGLE_CMD (pitch +1 = nose down = theta < 0)'
'dac_rel(9) = shp(11) + thr*(shp(12) - shp(11)) - shp(16);  % THROTTLE (absolute scale)'
'dac_rel(10) = 0;                               % DAC_SPARE'
'armed = arm_in > 0.5;'
'arm = double(armed);'
'ihold = double(~armed || (shp(13) > 0.5 && thr < shp(14)) || k*Ts < shp(15));'
'att_deg = [phi; th];'
'k = k + 1;'
}, newline);
end

function s = ctrlCode()
s = strjoin({
'function [us, u_V] = ctrl_fw(v, arm, cp, mixm)'
'%control_io U3 firmware model (ASSUMED): 10-bit ADC (VDD ref), quad-X mix, PWM 1000-2000 us'
'% v = [U_ROLL U_PITCH U_YAW THROTTLE VGND_A] absolute volts'
'% cp = [lsb bits thr_V0 thr_V1 us_off us_idle us_max us_res K_roll K_pitch K_yaw]'
'code = min(max(floor(v/cp(1)), 0), 2^cp(2) - 1);'
'vq = code*cp(1);'
'u_V = vq(1:3) - vq(5);'
'thr = min(max((vq(4) - cp(3))/(cp(4) - cp(3)), 0), 1);'
'thr_us = cp(5) + thr*(cp(7) - cp(5));'
'Krpy = [cp(9); cp(10); cp(11)];'
'm = mixm(:, 1)*thr_us + mixm(:, 2:4)*(Krpy.*u_V);'
'if arm > 0.5'
'    m = min(max(m, cp(6)), cp(7));'
'else'
'    m = cp(5)*ones(4, 1);'
'end'
'us = round(m/cp(8))*cp(8);'
}, newline);
end

function s = w1Code()
s = strjoin({
'function v = w1(t, w1p)'
'%AD3 W1 generator: w1p = [level_abs amp freq t0]; freq 0 = step'
'v = w1p(1);'
'if t >= w1p(4)'
'    if w1p(3) > 0'
'        v = w1p(1) + w1p(2)*sin(2*pi*w1p(3)*(t - w1p(4)));'
'    else'
'        v = w1p(1) + w1p(2);'
'    end'
'end'
}, newline);
end

function s = rbCode()
s = strjoin({
'function [xdot, omega, f_body, eul] = rb(x, T, pl)'
'%Rigid-body attitude dynamics (FRD), thrust T(4) along -z at r_i, Q_i = -spin_i*kQ/kT*T_i'
'omega = x(1:3); eul = x(4:6);'
'tau = [-pl.ry*T; pl.rx*T; -(pl.spin*T)*pl.kQT];'
'wdot = pl.Jinv*(tau - cross(omega, pl.J*omega) - pl.krot.*omega);'
'ph = eul(1); th = eul(2);'
'p = omega(1); q = omega(2); r = omega(3);'
'eulDot = [p + (q*sin(ph) + r*cos(ph))*tan(th); q*cos(ph) - r*sin(ph); (q*sin(ph) + r*cos(ph))/cos(th)];'
'xdot = [wdot; eulDot];'
'% quasi-static specific force (hover in place): -gravity in body axes'
'f_body = pl.g*[sin(th); -sin(ph)*cos(th); -cos(ph)*cos(th)];'
}, newline);
end

%% ======================================================================
%  Helpers
function startModel(mdl, here)
if bdIsLoaded(mdl), close_system(mdl, 0); end
slx = fullfile(here, [mdl '.slx']);
if isfile(slx), delete(slx); end
new_system(mdl);
end

function setCommonConfig(mdl)
% Fixed-step ode4 at pcb.sim_dt (= Model 1 qsim.dt, 0.5 ms): referenced models must match the parent step.
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode4', 'FixedStep', 'pcb.sim_dt', ...
    'SignalLogging', 'on', 'ModelReferenceNumInstancesAllowed', 'Multi');
set_param(mdl, 'PostLoadFcn', postLoad(mdl, true));
end

function s = postLoad(mdl, needBus)
% Callbacks run in the base workspace: load defaults only if missing.
s = ['pbdir_ = fileparts(get_param(''' mdl ''', ''FileName''));' ...
     ' if ~exist(''pcb'', ''var''), run(fullfile(pbdir_, ''pid_board_params.m'')); end;'];
if needBus
    s = [s ' if ~exist(''CtrlIn'', ''var''), run(fullfile(pbdir_, ''..'', ''common'', ''quad_ctrl_buses.m'')); end;'];
end
s = [s ' clear pbdir_'];
end

function finishModel(mdl, here)
subs = find_system(mdl, 'MatchFilter', @Simulink.match.allVariants, 'LookUnderMasks', 'all', 'BlockType', 'SubSystem');
for k = 1:numel(subs)
    if strcmp(get_param(subs{k}, 'Variant'), 'off') && isempty(get_param(subs{k}, 'ReferenceBlock')) ...
            && ~strcmp(get_param(subs{k}, 'SFBlockType'), 'MATLAB Function')
        try, Simulink.BlockDiagram.arrangeSystem(subs{k}); catch, end
    end
end
Simulink.BlockDiagram.arrangeSystem(mdl);
try
    save_system(mdl, fullfile(here, [mdl '.slx']));
    fprintf('Saved %s\n', fullfile(here, [mdl '.slx']));
catch err
    % e.g. pid_analog_circuit open with unsaved edits in this MATLAB session
    warning('pid_board_build:notSaved', '%s NOT saved: %s\nSave it later with save_system(''%s'').', ...
        mdl, err.message, mdl);
end
end

function setVariantArg(blk)
% harness instances follow pcb.variant
ip = get_param(blk, 'InstanceParameters');
for j = 1:numel(ip)
    if strcmp(ip(j).Name, 'board_variant'), ip(j).Value = 'pcb.variant'; end
end
set_param(blk, 'InstanceParameters', ip);
end

function h = add(sys, lib, name, varargin)
h = add_block(lib, [sys '/' name], varargin{:});
end

function wire(sys, src, dst)
add_line(sys, src, dst, 'autorouting', 'smart');
end

function rails(sys, name)
add(sys, 'simulink/Discontinuities/Saturation', name, 'UpperLimit', 'rhi', 'LowerLimit', 'rlo');
end

function logsig(sys, blk, port, name)
ph = get_param([sys '/' blk], 'PortHandles');
l = get_param(ph.Outport(port), 'Line');
if l > 0, set_param(l, 'Name', name); end
set_param(ph.Outport(port), 'DataLogging', 'on', 'DataLoggingNameMode', 'SignalName');
end

function sub = newSub(sys, name, inNames, outNames)
sub = [sys '/' name];
add_block('simulink/Ports & Subsystems/Subsystem', sub);
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

function addMask(blk, params, init)
m = Simulink.Mask.create(blk);
for k = 1:size(params, 1)
    m.addParameter('Name', params{k, 1}, 'Prompt', params{k, 1}, 'Value', params{k, 2}, ...
        'Evaluate', 'on', 'Tunable', 'off');
end
m.Initialization = init;
end

function mlfn(sys, name, code, paramNames, sizes)
% sizes: {dataName, sizeString; ...} to pin port dimensions
if nargin < 5, sizes = {}; end
blk = [sys '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', blk);
ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', blk);
ch.Script = code;
for k = 1:numel(paramNames)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', paramNames{k});
    d.Scope = 'Parameter';
    d.Tunable = false;
end
for k = 1:size(sizes, 1)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', sizes{k, 1});
    d.Props.Array.Size = sizes{k, 2};
end
end
