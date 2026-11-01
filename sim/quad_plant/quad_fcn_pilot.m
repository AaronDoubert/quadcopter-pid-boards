function [sticks, arm, e_int] = quad_fcn_pilot(cmd, Xe, Ve, ih, P)
%QUAD_FCN_PILOT Radio commands + optional "auto pilot" on the throttle stick.
%   Source of the "pilot" MATLAB Function block in quad_sim.slx.
%   cmd = [roll pitch yaw thr arm alt_en alt_ref] from the scenario table
%   (qsim.cmd). Angle mode has no altitude loop, so to keep step tests and
%   long hovers at a constant height a simulated pilot can move the
%   throttle stick (alt_en = 1):
%       vz_cmd = sat(kh*(h_ref - h), +-vz_max)              climb-rate demand
%       thr    = thr_cmd + kvz*(vz_cmd - hdot) + ki*ih      ih = int(vz_cmd - hdot)
%   This is the PILOT, outside the controller interface (the controller just
%   sees a throttle stick). e_int feeds the external integrator (ih); it is
%   zero while alt_en = 0 or the stick is saturated (anti-windup).

cmd = cmd(:);
h = -Xe(3);  hdot = -Ve(3);          % NED -> altitude, climb rate
en = double(cmd(6) > 0.5);
vz_cmd = min(max(P.kh*(cmd(7) - h), -P.vz_max), P.vz_max);
ev = vz_cmd - hdot;
thr_raw = cmd(4) + en*(P.kvz*ev + P.ki*ih);
thr = min(max(thr_raw, 0.05), 1);
if en < 0.5, thr = min(max(cmd(4), 0), 1); end
sticks = [min(max(cmd(1:3), -1), 1); thr];
arm = double(cmd(5) > 0.5);
e_int = en*ev*double(thr == thr_raw);
end
