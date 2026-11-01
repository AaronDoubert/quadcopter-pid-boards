function [gyro, accel] = quad_fcn_imu(wb, f, n, P)
%QUAD_FCN_IMU MPU-6050 model, discrete at quad.imu.Ts (1 kHz).
%   Source of the "imu" MATLAB Function block in quad_sim.slx.
%   wb true body rates [3x1] rad/s, f specific force at the IMU [3x1] m/s^2,
%   n [6x1] unit-variance white noise (Random Number block, seeded).
%   Per axis: scale error, bias (boot-calibration residual), white noise
%   (density * sqrt(fs/2)), g-sensitivity, DLPF = 1st-order low-pass at the
%   DLPF bandwidth + an integer sample delay so the total group delay matches
%   the datasheet DLPF delay, full-scale saturation, LSB quantisation.
%   Outputs gyro rad/s, accel m/s^2 (body FRD axes; IMU axes assumed aligned).

wb = wb(:); f = f(:); n = n(:);
persistent xg xa bg ba
if isempty(xg)
    xg = wb;  xa = f;
    bg = repmat(wb, 1, 8);  ba = repmat(f, 1, 8);
end

g_raw = (1 + P.scale_g).*wb + P.bias_on*P.bias_g + P.noise_on*P.sig_g*n(1:3) ...
        + P.gsens*f/P.g;
a_raw = (1 + P.scale_a).*f + P.bias_on*P.bias_a + P.noise_on*P.sig_a*n(4:6);

xg = P.a_g*xg + (1 - P.a_g)*g_raw;
xa = P.a_a*xa + (1 - P.a_a)*a_raw;

% integer delay line (column 1 = newest)
bg = [xg, bg(:,1:7)];
ba = [xa, ba(:,1:7)];
gd = bg(:, P.nd_g + 1);
ad = ba(:, P.nd_a + 1);

gyro  = P.lsb_g*round(min(max(gd, -P.sat_g), P.sat_g)/P.lsb_g);
accel = P.lsb_a*round(min(max(ad, -P.sat_a), P.sat_a)/P.lsb_a);
end
