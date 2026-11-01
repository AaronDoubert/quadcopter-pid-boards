function [F_b, M_b, f_imu, T] = quad_fcn_forces(w, Qm, Vb, wb, DCM, Xe, Ve, eul, P)
%QUAD_FCN_FORCES Body forces/moments for the 6DOF block + IMU specific force.
%   Source of the "forces" MATLAB Function block in quad_sim.slx.
%   Inputs: rotor speeds w [4x1] rad/s, motor torques Qm [4x1] N m, body
%   velocity Vb, body rates wb, DCM_be (earth->body), position Xe (NED),
%   earth velocity Ve, Euler angles eul. All forces in body FRD axes, about
%   the CG. Includes: thrust kT*w^2 at each hub (r_motor), yaw reaction
%   -spin*Qm, rotor gyroscopic moment, linear + quadratic drag, rotational
%   damping, gravity, and a simple ground (spring-damper in z, damped
%   sliding, levelling torque) for z_e > 0.
%   f_imu = specific force at the IMU location (what an accelerometer reads).

w = w(:); Qm = Qm(:); Vb = Vb(:); wb = wb(:); Xe = Xe(:); Ve = Ve(:); eul = eul(:);
T = P.kT*w.^2;                                   % N per rotor, along -z_b


% Thrust and its moment about the CG: r x [0;0;-T]
F_thr = [0; 0; -sum(T)];
M_thr = zeros(3,1);
for i = 1:4
    r = P.r_motor(:,i);
    M_thr = M_thr + [-r(2)*T(i); r(1)*T(i); 0];
end
% Yaw: stator reaction to the motor torque (includes rotor acceleration)
M_yaw = [0; 0; -sum(P.spin(:).*Qm)];
% Rotor gyroscopic moment: h = J_r*sum(spin_i*w_i)*z_b
h = [0; 0; P.J_r*sum(P.spin(:).*w)];
M_gyro = -cross(wb, h);

% Aerodynamic drag and rotational damping (no wind: airspeed = Vb)
F_drag = -P.m*P.d_lin.*Vb - 0.5*P.rho*P.CdA.*abs(Vb).*Vb;
M_damp = -P.k_rot.*wb;

% Gravity
F_grav = DCM*[0; 0; P.m*P.g];

% Ground (z_e down, ground at z_e = 0)
F_gnd_e = zeros(3,1);
M_gnd = zeros(3,1);
if Xe(3) > 0
    Fz = -(P.gnd_k*Xe(3) + P.gnd_c*Ve(3));
    Fz = min(Fz, 0);                              % ground can only push up
    F_gnd_e = [-P.gnd_cxy*Ve(1); -P.gnd_cxy*Ve(2); Fz];
    M_gnd = -P.gnd_klev*[eul(1); eul(2); 0] - P.gnd_clev*wb;
end
F_gnd = DCM*F_gnd_e;

F_b = F_thr + F_drag + F_grav + F_gnd;
M_b = M_thr + M_yaw + M_gyro + M_damp + M_gnd;

% Specific force at the IMU: non-gravitational accel + lever-arm terms
dwb = P.Jinv*(M_b - cross(wb, P.J*wb));
f_imu = (F_b - F_grav)/P.m + cross(dwb, P.r_imu) + cross(wb, cross(wb, P.r_imu));
end
