function quad_build_models(which_models)
%QUAD_BUILD_MODELS (Re)build ctrl_ideal.slx and quad_sim.slx from source.
%   quad_build_models            builds both
%   quad_build_models('ctrl')    builds ctrl_ideal.slx only
%   quad_build_models('top')     builds quad_sim.slx only
%   The MATLAB Function blocks get their code from the files
%   ctrl_ideal_core.m and quad_fcn_*.m in this folder (copied into the
%   blocks, so the .slx files are self-contained). Run quad_sim_params first.
%   Overwrites the two .slx files.

if nargin < 1, which_models = 'all'; end
root = fileparts(mfilename('fullpath'));
if any(strcmp(which_models, {'all','ctrl'})), build_ctrl(root); end
if any(strcmp(which_models, {'all','top'})),  build_top(root);  end
end

%% ------------------------------------------------------------------------
function build_ctrl(root)
m = 'ctrl_ideal';
if bdIsLoaded(m), close_system(m, 0); end
new_system(m);
set_param(m, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', 'auto', ...
    'Description', ['Ideal 5-loop cascaded PID (angle P -> rate PID roll/pitch, rate PID yaw) + ' ...
    'Quad-X mixer, 1 kHz. Interface: Inport ctrl_in (bus CtrlIn), Outport esc_us [4x1] us. ' ...
    'Gains: struct ideal (ctrl_ideal_params.m).'], ...
    'PreLoadFcn', preload_cb());

add_block('built-in/Inport', [m '/ctrl_in'], 'OutDataTypeStr', 'Bus: CtrlIn', ...
    'BusOutputAsStruct', 'on', 'SampleTime', 'ideal.Ts', 'Position', [40 100 70 114]);
fb = add_fcn(m, 'ctrl_core', fullfile(root, 'ctrl_ideal_core.m'), {}, [160 80 300 160]);
set_data(fb, 'ci', 'DataType', 'Bus: CtrlIn');
set_data(fb, 'ideal', 'Scope', 'Parameter');
add_block('built-in/Outport', [m '/esc_us'], 'Position', [420 95 450 109]);
add_block('built-in/Terminator', [m '/dbg_term'], 'Position', [420 135 440 155]);
add_line(m, 'ctrl_in/1', 'ctrl_core/1');
h = add_line(m, 'ctrl_core/1', 'esc_us/1'); set_param(h, 'Name', 'esc_us');
h = add_line(m, 'ctrl_core/2', 'dbg_term/1'); set_param(h, 'Name', 'ctrl_dbg');
set_log(m, 'ctrl_core', 2, 'ctrl_dbg');
save_system(m, fullfile(root, [m '.slx']));
end

%% ------------------------------------------------------------------------
function build_top(root)
m = 'quad_sim';
if bdIsLoaded(m), close_system(m, 0); end
new_system(m);
set_param(m, 'SolverType', 'Fixed-step', 'Solver', 'ode4', 'FixedStep', 'qsim.dt', ...
    'StopTime', 'qsim.StopTime', 'SignalLogging', 'on', 'SignalLoggingName', 'logsout', ...
    'ReturnWorkspaceOutputs', 'on', 'SaveTime', 'off', 'SaveOutput', 'off', ...
    'AlgebraicLoopMsg', 'error', 'PreLoadFcn', preload_cb(), ...
    'Description', ['Model 1: 5" quad 6-DOF plant + controller slot (Variant Subsystem ' ...
    '"Controller", qsim.ctrl: 1 ideal, 2 board_fast, 3 board_circuit). Run quad_sim_params first.']);

% ---------------- Radio + simulated pilot
s = new_sub(m, 'Radio', {'Xe','Ve'}, {'sticks','arm'}, [40 40 160 120]);
add_block('simulink/Sources/From Workspace', [s '/cmd_table'], 'VariableName', 'qsim.cmd', ...
    'Interpolate', 'on', 'OutputAfterFinalValue', 'Holding final value', 'SampleTime', '0');
f = add_fcn(s, 'pilot', fullfile(sroot(root), 'quad_fcn_pilot.m'), {'P','qsim.pilot'}, []);
add_block('built-in/Integrator', [s '/alt_int']);
add_line(s, 'cmd_table/1', 'pilot/1'); add_line(s, 'Xe/1', 'pilot/2'); add_line(s, 'Ve/1', 'pilot/3');
add_line(s, 'alt_int/1', 'pilot/4');
add_line(s, 'pilot/1', 'sticks/1'); add_line(s, 'pilot/2', 'arm/1'); add_line(s, 'pilot/3', 'alt_int/1');
arrange(s); %#ok<NASGU>
f; %#ok<VUNUS>

% ---------------- CtrlIn bus (sampled at qsim.Ts_bus)
s = new_sub(m, 'CtrlBus', {'gyro','accel','pqr','euler','sticks','arm','vbat'}, {'ctrl_in'}, [700 40 820 260]);
names = {'gyro_radps','accel_mps2','rates_true','euler_true','sticks','arm','vbat_V'};
ins   = {'gyro','accel','pqr','euler','sticks','arm','vbat'};
add_block('simulink/Signal Routing/Bus Creator', [s '/bus'], 'Inputs', '7', ...
    'OutDataTypeStr', 'Bus: CtrlIn', 'NonVirtualBus', 'on');
for i = 1:7
    zb = ['zoh_' ins{i}];
    add_block('simulink/Discrete/Zero-Order Hold', [s '/' zb], 'SampleTime', 'qsim.Ts_bus');
    add_line(s, [ins{i} '/1'], [zb '/1']);
    h = add_line(s, [zb '/1'], sprintf('bus/%d', i)); set_param(h, 'Name', names{i});
end
add_line(s, 'bus/1', 'ctrl_in/1');
arrange(s);

% ---------------- Controller slot (Variant Subsystem)
vs = [m '/Controller'];
add_block('simulink/Ports & Subsystems/Variant Subsystem', vs, 'Position', [900 120 1040 180]);
kids = find_system(vs, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'MatchFilter', @Simulink.match.allVariants);
for i = 2:numel(kids), delete_block(kids{i}); end
lh = find_system(vs, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line'); delete_line(lh);
add_block('built-in/Inport',  [vs '/ctrl_in'], 'Position', [40 100 70 114]);
add_block('built-in/Outport', [vs '/esc_us'],  'Position', [500 100 530 114]);
add_block('simulink/Ports & Subsystems/Model', [vs '/ideal'], 'ModelName', 'ctrl_ideal', ...
    'Position', [200 40 360 100], 'VariantControl', 'qsim.ctrl == 1');
set_param([vs '/ideal'], 'DefaultDataLogging', 'on');
% Board variants: Model blocks to sim/pid_board/pid_board.slx. The file
% does not exist yet; quad_set_controller() fills ModelName when it does.
add_block('built-in/SubSystem', [vs '/board_fast'], 'Position', [200 140 360 200]);
add_block('built-in/SubSystem', [vs '/board_circuit'], 'Position', [200 240 360 300]);
for b = {'board_fast','board_circuit'}
    p = [vs '/' b{1}];
    add_block('built-in/Inport', [p '/ctrl_in']);
    add_block('built-in/Outport', [p '/esc_us']);
    add_block('built-in/Constant', [p '/placeholder_1000us'], 'Value', '1000*ones(4,1)');
    add_block('built-in/Terminator', [p '/term']);
    add_line(p, 'ctrl_in/1', 'term/1'); add_line(p, 'placeholder_1000us/1', 'esc_us/1');
    set_param(p, 'Description', 'PLACEHOLDER until sim/pid_board/pid_board.slx exists: run quad_set_controller(''install_board'').');
end
set_param([vs '/board_fast'], 'VariantControl', 'qsim.ctrl == 2');
set_param([vs '/board_circuit'], 'VariantControl', 'qsim.ctrl == 3');
set_param(vs, 'Description', ['Controller slot. qsim.ctrl selects: 1 ideal (ctrl_ideal.slx), ' ...
    '2 board_fast, 3 board_circuit (sim/pid_board/pid_board.slx). Interface = Inport ctrl_in ' ...
    '(bus CtrlIn), Outport esc_us [4x1] double us.']);

% ---------------- ESC: PWM frame ZOH + latency + pulse -> duty
s = new_sub(m, 'ESC', {'esc_us'}, {'duty'}, [1100 120 1220 180]);
add_block('simulink/Discrete/Zero-Order Hold', [s '/pwm_frame'], 'SampleTime', 'qsim.esc.Ts');
add_block('simulink/Continuous/Transport Delay', [s '/latency'], 'DelayTime', 'qsim.esc.delay', ...
    'InitialOutput', 'qsim.esc.us0', 'BufferSize', '4096');
add_block('simulink/Math Operations/Bias', [s '/minus_min'], 'Bias', '-qsim.esc.us_min');
add_block('simulink/Math Operations/Gain', [s '/per_span'], 'Gain', '1/qsim.esc.us_span');
add_block('simulink/Discontinuities/Saturation', [s '/clip01'], 'UpperLimit', '1', 'LowerLimit', '0');
chain(s, {'esc_us','pwm_frame','latency','minus_min','per_span','clip01','duty'});
arrange(s);

% ---------------- Motors + battery
s = new_sub(m, 'Motors_Battery', {'duty'}, {'w','rpm','I_m','Qm','vbat','I_bus','soc'}, [1300 100 1440 260]);
add_fcn(s, 'motor_batt', fullfile(sroot(root), 'quad_fcn_motor_batt.m'), {'P','qsim.p'}, []);
add_block('built-in/Integrator', [s '/w_int'], 'InitialCondition', 'qsim.init.w0', ...
    'LimitOutput', 'on', 'LowerSaturationLimit', '0', 'UpperSaturationLimit', 'inf');
add_block('built-in/Integrator', [s '/soc_int'], 'InitialCondition', 'qsim.init.soc0', ...
    'LimitOutput', 'on', 'LowerSaturationLimit', '0', 'UpperSaturationLimit', '1');
add_block('simulink/Math Operations/Gain', [s '/to_rpm'], 'Gain', '60/(2*pi)');
add_line(s, 'duty/1', 'motor_batt/1'); add_line(s, 'w_int/1', 'motor_batt/2'); add_line(s, 'soc_int/1', 'motor_batt/3');
add_line(s, 'motor_batt/1', 'w_int/1'); add_line(s, 'motor_batt/6', 'soc_int/1');
add_line(s, 'w_int/1', 'w/1'); add_line(s, 'w_int/1', 'to_rpm/1'); add_line(s, 'to_rpm/1', 'rpm/1');
add_line(s, 'motor_batt/2', 'I_m/1'); add_line(s, 'motor_batt/3', 'Qm/1');
add_line(s, 'motor_batt/4', 'vbat/1'); add_line(s, 'motor_batt/5', 'I_bus/1'); add_line(s, 'soc_int/1', 'soc/1');
arrange(s);

% ---------------- Airframe: forces/moments + 6DOF (quaternion)
s = new_sub(m, 'Airframe', {'w','Qm'}, {'Xe','Ve','euler','Vb','pqr','f_imu','thrust'}, [1520 100 1660 260]);
add_fcn(s, 'forces', fullfile(sroot(root), 'quad_fcn_forces.m'), {'P','qsim.p'}, []);
add_block('aerolib6dof/6DoF (Quaternion)', [s '/rigid_body_6dof'], 'xme_0', 'qsim.init.Xe0', ...
    'Vm_0', '[0 0 0]', 'eul_0', '[0 0 0]', 'pm_0', '[0 0 0]', 'Mass', 'quad.mass.total', ...
    'Inertia', 'quad.inertia.J');
add_block('built-in/Terminator', [s '/pdot_term']);
% forces(w, Qm, Vb, wb, DCM, Xe, Ve, eul)
add_line(s, 'w/1', 'forces/1'); add_line(s, 'Qm/1', 'forces/2');
add_line(s, 'rigid_body_6dof/5', 'forces/3'); add_line(s, 'rigid_body_6dof/6', 'forces/4');
add_line(s, 'rigid_body_6dof/4', 'forces/5'); add_line(s, 'rigid_body_6dof/2', 'forces/6');
add_line(s, 'rigid_body_6dof/1', 'forces/7'); add_line(s, 'rigid_body_6dof/3', 'forces/8');
add_line(s, 'forces/1', 'rigid_body_6dof/1'); add_line(s, 'forces/2', 'rigid_body_6dof/2');
add_line(s, 'rigid_body_6dof/7', 'pdot_term/1');
add_line(s, 'rigid_body_6dof/2', 'Xe/1'); add_line(s, 'rigid_body_6dof/1', 'Ve/1');
add_line(s, 'rigid_body_6dof/3', 'euler/1'); add_line(s, 'rigid_body_6dof/5', 'Vb/1');
add_line(s, 'rigid_body_6dof/6', 'pqr/1'); add_line(s, 'forces/3', 'f_imu/1');
add_line(s, 'forces/4', 'thrust/1');
arrange(s);

% ---------------- IMU (MPU-6050)
s = new_sub(m, 'IMU', {'pqr','f_imu'}, {'gyro','accel'}, [1520 340 1660 400]);
add_block('simulink/Discrete/Zero-Order Hold', [s '/zoh_w'], 'SampleTime', 'qsim.imu.Ts');
add_block('simulink/Discrete/Zero-Order Hold', [s '/zoh_f'], 'SampleTime', 'qsim.imu.Ts');
add_block('simulink/Sources/Random Number', [s '/noise'], 'Mean', '0', 'Variance', '1', ...
    'Seed', 'qsim.imu.seed', 'SampleTime', 'qsim.imu.Ts');
add_fcn(s, 'imu', fullfile(sroot(root), 'quad_fcn_imu.m'), {'P','qsim.imu'}, []);
add_line(s, 'pqr/1', 'zoh_w/1'); add_line(s, 'f_imu/1', 'zoh_f/1');
add_line(s, 'zoh_w/1', 'imu/1'); add_line(s, 'zoh_f/1', 'imu/2'); add_line(s, 'noise/1', 'imu/3');
add_line(s, 'imu/1', 'gyro/1'); add_line(s, 'imu/2', 'accel/1');
arrange(s);

% ---------------- top-level wiring (+ named, logged signals)
L = @(src, dst, nm) top_line(m, src, dst, nm);
L('Radio/1', 'CtrlBus/5', 'sticks');
L('Radio/2', 'CtrlBus/6', 'arm');
L('CtrlBus/1', 'Controller/1', 'ctrl_in');
L('Controller/1', 'ESC/1', 'esc_us');
L('ESC/1', 'Motors_Battery/1', 'duty');
L('Motors_Battery/1', 'Airframe/1', 'w_radps');
L('Motors_Battery/4', 'Airframe/2', 'Qm');
L('Motors_Battery/5', 'CtrlBus/7', 'vbat_V');
L('Airframe/1', 'Radio/1', 'Xe');
L('Airframe/2', 'Radio/2', 'Ve');
L('Airframe/3', 'CtrlBus/4', 'euler');
L('Airframe/5', 'CtrlBus/3', 'pqr');
L('Airframe/5', 'IMU/1', '');
L('Airframe/6', 'IMU/2', 'f_imu');
L('IMU/1', 'CtrlBus/1', 'gyro');
L('IMU/2', 'CtrlBus/2', 'accel');
tnames = {'Motors_Battery/2','motor_rpm'; 'Motors_Battery/3','I_motor'; 'Motors_Battery/6','I_bus'; ...
          'Motors_Battery/7','soc'; 'Airframe/4','Vb'; 'Airframe/7','thrust_N'};
for i = 1:size(tnames, 1)
    tb = ['log_' tnames{i,2}];
    add_block('built-in/Terminator', [m '/' tb]);
    L(tnames{i,1}, [tb '/1'], tnames{i,2});
end
logs = {'Radio',1; 'Radio',2; 'Controller',1; 'ESC',1; 'Motors_Battery',1; 'Motors_Battery',2; ...
        'Motors_Battery',3; 'Motors_Battery',5; 'Motors_Battery',6; 'Motors_Battery',7; ...
        'Airframe',1; 'Airframe',2; 'Airframe',3; 'Airframe',4; 'Airframe',5; 'Airframe',6; ...
        'Airframe',7; 'IMU',1; 'IMU',2};
for i = 1:size(logs, 1)
    set_log(m, logs{i,1}, logs{i,2}, '');
end
arrange(m);
save_system(m, fullfile(root, [m '.slx']));
end

%% ------------------------------------------------------------------------
function r = sroot(root), r = root; end

function cb = preload_cb()
cb = ['if ~exist(''qsim'',''var'') || ~exist(''quad'',''var'') || ~exist(''ideal'',''var'') || ~exist(''CtrlIn'',''var''), ' ...
      'run(fullfile(fileparts(which(''quad_sim_params'')),''quad_sim_params.m'')); end'];
end

function s = new_sub(m, name, ins, outs, pos)
s = [m '/' name];
add_block('built-in/SubSystem', s, 'Position', pos);
for i = 1:numel(ins)
    add_block('built-in/Inport', [s '/' ins{i}], 'Position', [20 40+60*i 50 54+60*i]);
end
for i = 1:numel(outs)
    add_block('built-in/Outport', [s '/' outs{i}], 'Position', [700 40+60*i 730 54+60*i]);
end
end

function blk = add_fcn(sys, name, file, maskp, pos)
blk = [sys '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', blk);
if ~isempty(pos), set_param(blk, 'Position', pos); end
code = fileread(file);
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', blk);
ch.Script = code;
if ~isempty(maskp)
    set_data(blk, maskp{1}, 'Scope', 'Parameter');
    mk = Simulink.Mask.create(blk);
    mk.addParameter('Name', maskp{1}, 'Value', maskp{2}, 'Type', 'edit', 'Prompt', maskp{1});
    mk.Display = sprintf('disp(''%s'')', name);
end
end

function set_data(blk, dname, prop, val)
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', blk);
d = ch.find('-isa', 'Stateflow.Data', 'Name', dname);
d.(prop) = val;
end

function chain(s, names)
for i = 1:numel(names)-1
    add_line(s, [names{i} '/1'], [names{i+1} '/1']);
end
end

function top_line(m, src, dst, nm)
h = add_line(m, src, dst, 'autorouting', 'smart');
if ~isempty(nm), set_param(h, 'Name', nm); end
end

function set_log(m, blk, port, nm) %#ok<INUSD>
ph = get_param([m '/' blk], 'PortHandles');
set_param(ph.Outport(port), 'DataLogging', 'on');
end

function out = arrange(s)
out = [];
try
    Simulink.BlockDiagram.arrangeSystem(s);
catch
end
end
