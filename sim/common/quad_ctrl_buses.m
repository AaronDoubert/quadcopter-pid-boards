%QUAD_CTRL_BUSES Interface contract between the quad plant (model 1) and any controller.
%   Defines the Simulink.Bus object CtrlIn in the base workspace. Every
%   controller variant (ideal 5-loop PID, PID board fast, PID board circuit)
%   is a referenced model with exactly this interface:
%
%     Inport 1  "ctrl_in"  : bus CtrlIn
%     Outport 1 "esc_us"   : double [4x1], ESC pulse width in us (1000 = stop,
%                            2000 = full), in motor order M1..M4 below.
%
%   Conventions (same as sim/quad_plant/quad_params.m):
%     body axes x forward, y right, z down; angles roll phi, pitch theta, yaw psi
%     motor order (Betaflight Quad-X, seen from above):
%       M1 rear-right CW, M2 front-right CCW, M3 rear-left CCW, M4 front-left CW
%     sticks: roll/pitch/yaw in [-1 1], throttle in [0 1]. Like a radio:
%       roll +1 = stick right = roll right (phi > 0)
%       pitch +1 = stick forward = nose down (theta < 0)
%       yaw +1 = stick right = nose right (r > 0)
%
%   The plant runs continuous; controllers own their internal sample times.

el = @(name, dims, desc) local_el(name, dims, desc);

CtrlIn = Simulink.Bus;
CtrlIn.Description = 'Plant/radio -> controller';
CtrlIn.Elements = [ ...
    el('gyro_radps',   [3 1], 'IMU gyro body rates p q r (truth + sensor model from plant), rad/s')
    el('accel_mps2',   [3 1], 'IMU specific force in body axes, m/s^2')
    el('rates_true',   [3 1], 'true body rates p q r, rad/s (for ideal controllers / debugging)')
    el('euler_true',   [3 1], 'true roll pitch yaw, rad (for ideal controllers / debugging)')
    el('sticks',       [4 1], 'roll pitch yaw in [-1 1], throttle in [0 1]')
    el('arm',          [1 1], '1 = armed, 0 = disarmed')
    el('vbat_V',       [1 1], 'battery terminal voltage, V')];
assignin('base', 'CtrlIn', CtrlIn);
clear el

function e = local_el(name, dims, desc)
e = Simulink.BusElement;
e.Name = name;
e.Dimensions = dims;
e.DataType = 'double';
e.Description = desc;
end
