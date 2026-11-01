function quad_set_controller(choice, varargin)
%QUAD_SET_CONTROLLER Select or install the controller in quad_sim.slx's slot.
%   quad_set_controller('ideal')          qsim.ctrl = 1 (ctrl_ideal.slx)  [default]
%   quad_set_controller('board_fast')     qsim.ctrl = 2
%   quad_set_controller('board_circuit')  qsim.ctrl = 3
%       Only sets qsim.ctrl in the base workspace. The Variant Subsystem
%       quad_sim/Controller activates the choice whose condition
%       "qsim.ctrl == N" is true at the next update/sim.
%
%   quad_set_controller('install_board')
%   quad_set_controller('install_board', 'model', 'pid_board', 'arg', 'board_variant', ...
%                       'fast', '1', 'circuit', '2')
%       Replaces the two placeholder subsystems quad_sim/Controller/board_fast
%       and .../board_circuit by Model blocks that reference MODEL (default
%       pid_board, must be on the path: sim/pid_board is added by
%       quad_sim_params). If MODEL declares a model argument named ARG, the
%       fast choice gets ARG = FAST and the circuit choice ARG = CIRCUIT
%       (InstanceParameters). If it has no such argument, both Model blocks
%       reference the same model unchanged and the board model's own variant
%       control decides fast vs circuit (a warning says so). Saves quad_sim.slx.

idx = struct('ideal', 1, 'board_fast', 2, 'board_circuit', 3);
if isfield(idx, choice)
    q = evalin('base', 'qsim');
    q.ctrl = idx.(choice);
    assignin('base', 'qsim', q);
    fprintf('Controller slot -> %s (qsim.ctrl = %d)\n', choice, q.ctrl);
    return
end
if ~strcmp(choice, 'install_board')
    error('quad_set_controller: unknown choice "%s"', choice);
end

o = struct('model', 'pid_board', 'arg', 'board_variant', 'fast', '1', 'circuit', '2');
for k = 1:2:numel(varargin), o.(varargin{k}) = varargin{k+1}; end
if isempty(which([o.model '.slx'])) && isempty(which(o.model))
    error('quad_set_controller: %s.slx is not on the path', o.model);
end
here = fileparts(mfilename('fullpath'));
m = 'quad_sim';
load_system(fullfile(here, [m '.slx']));
load_system(o.model);
args = {};
try
    a = get_param(o.model, 'ParameterArgumentNames');
    if ~isempty(a), args = strtrim(strsplit(a, ',')); end
catch
end
hasArg = any(strcmp(args, o.arg));
if ~hasArg
    warning(['quad_set_controller: %s has no model argument "%s"; board_fast and board_circuit ' ...
        'will both reference it unchanged. Set its internal fast/circuit variant control yourself.'], ...
        o.model, o.arg);
end
vs = [m '/Controller'];
names = {'board_fast', 'board_circuit'};
vals  = {o.fast, o.circuit};
for i = 1:2
    p = [vs '/' names{i}];
    pos = get_param(p, 'Position');
    vc  = get_param(p, 'VariantControl');
    delete_block(p);
    add_block('simulink/Ports & Subsystems/Model', p, 'ModelName', o.model, ...
        'Position', pos, 'VariantControl', vc, 'DefaultDataLogging', 'on');
    if hasArg
        ip = get_param(p, 'InstanceParameters');
        for j = 1:numel(ip)
            if strcmp(ip(j).Name, o.arg), ip(j).Value = vals{i}; end
        end
        set_param(p, 'InstanceParameters', ip);
    end
end
save_system(m);
fprintf('Installed %s in %s (board_fast / board_circuit). Select with quad_set_controller(''board_fast'').\n', o.model, vs);
end
