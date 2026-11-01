function [dw, I_m, Qm, vbat, I_bus, dsoc] = quad_fcn_motor_batt(duty, w, soc, P)
%QUAD_FCN_MOTOR_BATT Four DC-equivalent BLDC motors + ESCs + 4S battery.
%   Source of the "motor_batt" MATLAB Function block in quad_sim.slx
%   (quad_build_models.m copies this code into the block).
%   duty [4x1] ESC duty 0..1, w [4x1] rotor speed rad/s (>= 0), soc 0..1.
%   Model (quad_params.m, motor + battery sections):
%     ESC = ideal buck: V_m = duty*V_bus, I_in = duty*I_m (damped light,
%           so the current may go negative = active braking); duty = 0
%           means FETs off, no current.
%     motor: I_m = (V_m - w/Kv)/R   (inductance neglected, L/R << 1 ms)
%            Q_m = Kt*(I_m - I0*tanh(w/w_I0))  (no-load loss as friction)
%            J_r*dw/dt = Q_m - kQ*w^2
%     battery: V_bus = cells*OCV(soc) - R_batt*I_bus, solved in closed form
%            (no algebraic loop), I_bus = sum(duty.*I_m) + I_avionics,
%            dsoc/dt = -I_bus/capacity.
%     ESC current limit quad.esc.I_limit per channel (30 A burst).
%   Outputs: dw [4x1], I_m [4x1] A, Qm [4x1] N m (motor shaft torque, the
%   body gets -spin*Qm), vbat V, I_bus A, dsoc 1/s.

w   = w(:);
d   = min(max(duty(:), 0), 1);
on  = double(d > 0);
s   = min(max(soc, 0), 1);
E   = P.cells*interp1(P.soc_tab, P.ocv_tab, s);   % open-circuit pack voltage
emf = w/P.Kv;                                      % back-EMF, V

% V = E - Rb*(sum(on.*d.*(d*V - emf))/R + Ia)  ->  solve for V
S1 = sum(on.*d.^2)/P.R;
S2 = sum(on.*d.*emf)/P.R;
vbat = (E + P.R_batt*(S2 - P.I_av))/(1 + P.R_batt*S1);
vbat = max(vbat, 0);

I_m = on.*(d*vbat - emf)/P.R;
% ESC per-channel current limit (+-I_lim): the ESC lowers its effective duty.
% A limited channel draws P = (emf + R*I_lim)*I_lim from the bus; re-solve
% the bus voltage once with those channels as constant-power loads.
lim = double(abs(I_m) > P.I_lim);
if any(lim)
    Ilim = sign(I_m)*P.I_lim;
    Plim = sum(lim.*(emf + P.R*Ilim).*Ilim);
    u = 1 - lim;
    S1u = sum(u.*on.*d.^2)/P.R;
    S2u = sum(u.*on.*d.*emf)/P.R;
    vbat = (E + P.R_batt*(S2u - P.I_av - Plim/max(vbat, 1)))/(1 + P.R_batt*S1u);
    vbat = max(vbat, 0);
    I_m = lim.*Ilim + u.*on.*(d*vbat - emf)/P.R;
    I_bus = sum(u.*d.*I_m) + Plim/max(vbat, 1) + P.I_av;
else
    I_bus = sum(d.*I_m) + P.I_av;
end
Qm    = P.Kt*(I_m - P.I0*tanh(w/P.w_I0));
dw    = (Qm - P.kQ*w.^2)/P.J_r;
dsoc  = -I_bus/P.capacity;
end
