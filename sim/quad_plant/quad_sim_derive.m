function qsim = quad_sim_derive(qsim, quad)
%QUAD_SIM_DERIVE Fill the derived fields of qsim from quad + qsim options.
%   qsim.p     flat numeric plant parameters (motor/battery/forces blocks)
%   qsim.imu   IMU filter/noise/quantisation constants
%   qsim.esc   ESC frame period and latency
%   qsim.init  initial states

%% ESC: PWM frame as a ZOH (period rounded to the solver step) + latency
fr = 1/quad.esc.update_hz;
qsim.esc.Ts    = max(1, round(fr/qsim.dt))*qsim.dt;          % 2.0 ms (490 Hz -> 500 Hz)
qsim.esc.delay = quad.esc.latency - 0.5/quad.esc.update_hz;   % s, pulse + ESC processing (2.2 ms)
qsim.esc.us_min = quad.esc.pwm_min_us;
qsim.esc.us_span = quad.esc.pwm_max_us - quad.esc.pwm_min_us;

%% Plant parameters (flat, numeric)
p = struct();
p.Kv = quad.motor.Kv;  p.Kt = quad.motor.Kt;  p.R = quad.motor.R;  p.I0 = quad.motor.I0;
p.I_lim = quad.esc.I_limit;                   % A per channel, ESC current limit
p.w_I0 = 20;                                  % rad/s, smoothing of the no-load friction near 0
p.J_r = quad.inertia.J_rotor;
p.kT = quad.prop.kT;  p.kQ = quad.prop.kQ;
p.spin = quad.motor.spin(:);
p.cells = quad.batt.cells;
p.soc_tab = quad.batt.soc_tab(:);  p.ocv_tab = quad.batt.ocv_tab(:);
p.R_batt = quad.batt.R_total;  p.I_av = quad.batt.I_avionics;
p.capacity = quad.batt.capacity;
p.m = quad.mass.total;  p.g = quad.g;  p.rho = quad.rho;
p.J = quad.inertia.J;  p.Jinv = quad.inertia.Jinv;
p.r_motor = quad.geom.r_motor;  p.r_imu = quad.geom.r_imu;
p.d_lin = quad.drag.d_lin;  p.CdA = quad.drag.CdA;  p.k_rot = quad.drag.k_rot;
p.gnd_k = qsim.gnd.k;  p.gnd_c = qsim.gnd.c;  p.gnd_cxy = qsim.gnd.cxy;
p.gnd_klev = qsim.gnd.klev;  p.gnd_clev = qsim.gnd.clev;
qsim.p = p;

%% IMU (MPU-6050)
im = quad.imu;
k  = im.dlpf_cfg_used + 1;
fs = 1/im.Ts;
qsim.imu.Ts   = im.Ts;
qsim.imu.a_g  = exp(-2*pi*im.dlpf.gyro_bw(k)*im.Ts);
qsim.imu.a_a  = exp(-2*pi*im.dlpf.acc_bw(k)*im.Ts);
qsim.imu.nd_g = min(7, max(0, round((im.dlpf.gyro_delay(k) - 1/(2*pi*im.dlpf.gyro_bw(k)))/im.Ts)));
qsim.imu.nd_a = min(7, max(0, round((im.dlpf.acc_delay(k)  - 1/(2*pi*im.dlpf.acc_bw(k)))/im.Ts)));
qsim.imu.sig_g = im.gyro.noise_density*sqrt(fs/2);
qsim.imu.sig_a = im.accel.noise_density*sqrt(fs/2);
qsim.imu.bias_g = im.gyro.bias_resid*qsim.imu.bias_g_pat;
qsim.imu.bias_a = [im.accel.bias_init_xy; im.accel.bias_init_xy; im.accel.bias_init_z].*qsim.imu.bias_a_pat;
qsim.imu.gsens = im.gyro.g_sens;
qsim.imu.g     = quad.g;
qsim.imu.sat_g = im.gyro.fs_sel_dps*pi/180;
qsim.imu.lsb_g = (pi/180)/im.gyro.lsb_per_dps([250 500 1000 2000] == im.gyro.fs_sel_dps);
qsim.imu.sat_a = im.accel.fs_g*quad.g;
qsim.imu.lsb_a = quad.g/im.accel.lsb_per_g([2 4 8 16] == im.accel.fs_g);

%% Initial states
if strcmpi(qsim.start, 'air')
    qsim.init.Xe0 = [0 0 -qsim.alt0];
    qsim.init.w0  = quad.checks.w_hover*ones(4,1);
    qsim.esc.us0  = quad.esc.pwm_min_us + quad.checks.hover_duty*qsim.esc.us_span;
else
    qsim.init.Xe0 = [0 0 quad.mass.total*quad.g/qsim.gnd.k];
    qsim.init.w0  = zeros(4,1);
    qsim.esc.us0  = quad.esc.pwm_min_us;
end
qsim.init.soc0 = qsim.soc0;
end
