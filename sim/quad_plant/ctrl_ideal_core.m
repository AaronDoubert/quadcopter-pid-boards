function [esc_us, dbg] = ctrl_ideal_core(ci, ideal)
%CTRL_IDEAL_CORE Ideal 5-loop cascaded PID + Quad-X mixer, called at ideal.Ts.
%   Source of the "ctrl_core" MATLAB Function block in ctrl_ideal.slx.
%   ci: bus CtrlIn (see sim/common/quad_ctrl_buses.m). ideal: gains
%   (ctrl_ideal_params.m). Structure mirrors the PID board:
%     roll/pitch angle P : rate_sp = Ka*(angle_cmd - angle)
%     rate PID (3 axes)  : e = rate_sp - gyro
%                          u = Kp*e + I + D, clipped to +-u_max   [us]
%                          I: dI/dt = Ki*e - wL*I, |I| <= I_max, held at 0
%                             while disarmed or throttle < i_hold_thr
%                          D: Kd*s/((1+s/w1)(1+s/w2)) on the ERROR (as on the
%                             board, so setpoint steps kick it)
%     mixer              : esc = thr_us + mix*u, clipped [us_idle, us_max];
%                          1000 us (us_off) while disarmed.
%   dbg = [rate_sp(3); e(3); u(3); I(3); D(3)].

persistent I x1 x2
if isempty(I)
    I = zeros(3,1); x1 = zeros(3,1); x2 = zeros(3,1);
end
Ts = ideal.Ts;

armed = ci.arm > 0.5;
st    = ci.sticks;
thr   = min(max(st(4), 0), 1);
rpy   = min(max(st(1:3), -1), 1);

% measurements
ang = ci.euler_true;
if ideal.use_true_rates > 0.5
    rates = ci.rates_true;
else
    rates = ci.gyro_radps;
end

% outer loops (angle P) and yaw rate command; stick pitch +1 = nose down
sp = [ ideal.roll.Ka *( ideal.max_angle*rpy(1) - ang(1));
       ideal.pitch.Ka*(-ideal.max_angle*rpy(2) - ang(2));
       ideal.max_yaw_rate*rpy(3) ];
sp = min(max(sp, -ideal.max_rate_sp), ideal.max_rate_sp);
e  = sp - rates;

Kp   = [ideal.roll.Kp;    ideal.pitch.Kp;    ideal.yaw.Kp];
Ki   = [ideal.roll.Ki;    ideal.pitch.Ki;    ideal.yaw.Ki];
Kd   = [ideal.roll.Kd;    ideal.pitch.Kd;    ideal.yaw.Kd];
wL   = [ideal.roll.wL;    ideal.pitch.wL;    ideal.yaw.wL];
Imax = [ideal.roll.I_max; ideal.pitch.I_max; ideal.yaw.I_max];
w1   = [ideal.roll.w1;    ideal.pitch.w1;    ideal.yaw.w1];
w2   = [ideal.roll.w2;    ideal.pitch.w2;    ideal.yaw.w2];

% band-limited derivative: HPF (Kd*s/(1+s/w1)) then LPF (1/(1+s/w2)), exact-ZOH poles
if ~armed
    x1 = e; x2 = zeros(3,1);
end
a1 = exp(-w1*Ts);
a2 = exp(-w2*Ts);
x1 = a1.*x1 + (1 - a1).*e;
d1 = Kd.*w1.*(e - x1);
x2 = a2.*x2 + (1 - a2).*d1;
D  = x2;

% leaky, clamped integrator with hold (board CD4066 I_HOLD)
if ~armed || thr < ideal.i_hold_thr
    I = zeros(3,1);
else
    I = I + Ts*(Ki.*e - wL.*I);
    I = min(max(I, -Imax), Imax);
end

u = Kp.*e + I + D;
u = min(max(u, -ideal.u_max), ideal.u_max);

thr_us = ideal.us_idle + thr*(ideal.us_max - ideal.us_idle);
esc_us = thr_us + ideal.mix*u;
esc_us = min(max(esc_us, ideal.us_idle), ideal.us_max);
if ~armed
    esc_us = ideal.us_off*ones(4,1);
end
dbg = [sp; e; u; I; D];
end
