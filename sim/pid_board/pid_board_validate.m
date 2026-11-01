function res = pid_board_validate(parts)
%PID_BOARD_VALIDATE Checks of Model 2 (fast) of PID_Board rev B.
%   res = pid_board_validate            runs all parts
%   res = pid_board_validate('bcd')     runs a subset of parts:
%     a  roll path vs the legacy sim/roll_axis model (built into tempdir)
%     b  per-axis stage gains / poles vs the spec formulas (linearised)
%     c  rail saturation and I-clamp
%     d  I_HOLD (CD4066) integrator reset
%     e  disarmed / armed ESC outputs
%     f  closed loop: board + rigid-body attitude plant (J from quad_params)
%   Uses only the base variables pcb, CtrlIn, pbsim (pcb is restored at the end).

if nargin < 1, parts = 'abcdef'; end
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here, '..', 'common'));
hadPcb = evalin('base', 'exist(''pcb'', ''var'')');
pcbUser = [];
if hadPcb, pcbUser = evalin('base', 'pcb'); end
cleanup = onCleanup(@() restorePcb(hadPcb, pcbUser));
evalin('base', 'pid_board_params; pcb.circuit_enable = 0;');   % validate the fast variant only
if ~evalin('base', 'exist(''CtrlIn'', ''var'')'), evalin('base', 'quad_ctrl_buses;'); end
for m = {'pid_analog_fast', 'pid_board', 'pid_board_bench', 'pid_board_tb_analog', 'pid_board_cl'}
    if ~isfile(fullfile(here, [m{1} '.slx'])), error('%s.slx missing: run pid_board_build', m{1}); end
end
res = struct();
nfail = 0;
if any(parts == 'b'), [res.b, n] = partB(); nfail = nfail + n; end
if any(parts == 'c'), [res.c, n] = partCD(); nfail = nfail + n; res.d = res.c; end
if any(parts == 'd') && ~any(parts == 'c'), [res.d, n] = partCD(); nfail = nfail + n; end
if any(parts == 'e'), [res.e, n] = partE(); nfail = nfail + n; end
if any(parts == 'a'), [res.a, n] = partA(here); nfail = nfail + n; end
if any(parts == 'f'), [res.f, n] = partF(here); nfail = nfail + n; end
res.nfail = nfail;
fprintf('\n%s: %d check(s) failed\n', passStr(nfail == 0), nfail);
end

%% ---------------------------------------------------------------------
function [r, nfail] = partB()
fprintf('\n(b) Stage gains and filter poles vs spec formulas (linearised analog model, nominal pots)\n');
pcb = evalin('base', 'pcb');
setAnalogInput(constIn(pcb), 0.1);
mdl = 'pid_board_tb_analog';
load_system(mdl);
nfail = 0; r = struct(); tol = 0.01;
fprintf('   %-34s %10s %10s %8s\n', 'check', 'model', 'formula', 'err');
cells = {'roll', 'Roll cell U201 U202', [2 7 1]; 'pitch', 'Pitch cell U301 U302', [4 8 3]; ...
         'yaw', 'Yaw cell U401 U402', [5 6 0]};
for c = 1:3
    an = cells{c, 1}; cp = ['pid_analog_fast/' cells{c, 2} '/']; ix = cells{c, 3};
    ax = pcb.(an);
    G = gains_from_pots(pcb, an); g = G.(an);
    outs = {'rails E', 'rails P', 'rails I', 'rails D', 'rails U'};
    io = linio([mdl '/split'], ix(1), 'input');
    for k = 1:numel(outs), io(end+1) = linio([cp outs{k}], 1, 'output'); end %#ok<AGROW>
    H = linearize(mdl, io);
    w = @(f) 2*pi*f;
    fr = @(i, f) squeeze(freqresp(H(i, 1), w(f)));
    % E per gyro DAC volt: -g_fb/(1 + s tau_inv)
    fpole = 1/(2*pi*pcb.dac.tau_inv_leg);
    nfail = nfail + chk(r, sprintf('%s E/RATE DC (-g_fb)', an), real(fr(1, 1e-3)), -g.g_fb, tol);
    nfail = nfail + chk(r, sprintf('%s RATE RC pole %.0f Hz (|H|/DC)', an, fpole), ...
        abs(fr(1, fpole))/abs(fr(1, 1e-3)), 1/sqrt(2), tol);
    s1 = 1j*w(1);
    nfail = nfail + chk(r, sprintf('%s Kp (P/E)', an), abs(fr(2, 1)/fr(1, 1)), g.Kp, tol);
    for f = [0.1 1 10]
        s = 1j*w(f);
        nfail = nfail + chk(r, sprintf('%s |I/E| at %g Hz (Ki %.2f, wL %g)', an, f, g.Ki, g.wL), ...
            abs(fr(3, f)/fr(1, f)), abs(g.Ki/(s + g.wL)), tol);
    end
    Dform = @(s) g.Kd*s/((1 + s/g.w1)*(1 + s/g.w2));
    for f = [10 59 100]
        nfail = nfail + chk(r, sprintf('%s |D/E| at %g Hz (Kd %.1f ms)', an, f, 1e3*g.Kd), ...
            abs(fr(4, f)/fr(1, f)), abs(Dform(1j*w(f))), tol);
    end
    Uform = g.en(1)*g.Kp + g.en(2)*g.Ki/(s1 + g.wL) + g.en(3)*Dform(s1);
    nfail = nfail + chk(r, sprintf('%s |U/E| at 1 Hz (summer, en_D=%d)', an, g.en(3)), ...
        abs(fr(5, 1)/fr(1, 1)), abs(Uform), tol);
    if c < 3
        % angle stages: from ANGLE_CMD and ANGLE
        io = [linio([mdl '/split'], ix(2), 'input'), linio([cp 'rails E_A'], 1, 'output'), ...
              linio([cp 'rails RATE_SP'], 1, 'output'), linio([cp 'rails E'], 1, 'output')];
        Ha = linearize(mdl, io);
        fa = @(i, f) squeeze(freqresp(Ha(i, 1), w(f)));
        nfail = nfail + chk(r, sprintf('%s E_A/ANGLE_CMD DC (-g_ang)', an), real(fa(1, 1e-3)), -g.g_ang, tol);
        nfail = nfail + chk(r, sprintf('%s Go (RATE_SP/E_A)', an), abs(fa(2, 1)/fa(1, 1)), g.Go, tol);
        nfail = nfail + chk(r, sprintf('%s E/ANGLE_CMD DC (g_sp*Go*g_ang)', an), real(fa(3, 1e-3)), ...
            g.g_sp*g.Go*g.g_ang, tol);
        io = [linio([mdl '/split'], ix(3), 'input'), linio([cp 'rails E_A'], 1, 'output')];
        Hm = linearize(mdl, io);
        fpn = 1/(2*pi*pcb.dac.tau_ni_leg);
        h0 = squeeze(freqresp(Hm, w(1e-3)));
        nfail = nfail + chk(r, sprintf('%s E_A/ANGLE DC (+g_ang_meas)', an), real(h0), ax.g_ang_meas, tol);
        nfail = nfail + chk(r, sprintf('%s ANGLE RC pole %.0f Hz', an, fpn), ...
            abs(squeeze(freqresp(Hm, w(fpn))))/abs(h0), 1/sqrt(2), tol);
    else
        io = [linio([mdl '/split'], 6, 'input'), linio([cp 'rails E'], 1, 'output')];
        Hy = linearize(mdl, io);
        h0 = squeeze(freqresp(Hy, w(1e-3)));
        fpn = 1/(2*pi*pcb.dac.tau_ni_leg);
        nfail = nfail + chk(r, 'yaw E/YAW_RATE_CMD DC (+g_sp = 0.909)', real(h0), g.g_sp, tol);
        nfail = nfail + chk(r, sprintf('yaw RATE_CMD RC pole %.0f Hz', fpn), ...
            abs(squeeze(freqresp(Hy, w(fpn))))/abs(h0), 1/sqrt(2), tol);
    end
end
fprintf('   spec nominal: Kp 1.16, Ki 10 /s (yaw 15.4), Kd 9.7 ms, f1 59 Hz, f2 164 Hz, Go 1.75, Ka 7.0 /s\n');
end

%% ---------------------------------------------------------------------
function [r, nfail] = partCD()
fprintf('\n(c) Rail saturation / I clamp and (d) I_HOLD reset (analog model, large steps)\n');
pcb = evalin('base', 'pcb');
VG = pcb.VG;
t = [0 0.05 0.05 0.3 0.3 0.4 0.4 0.6]';
V = repmat(constIn(pcb), numel(t), 1);
step = (1:numel(t))' >= 3;
V(step, 2) = VG - 2.0;    % ROLL_RATE  -> E_RR = +1.82 V: P to the low rail, I to -clamp, U high
V(step, 4) = VG + 2.0;    % PITCH_RATE -> E = -1.82 V: U_PITCH to the low rail
V(step, 6) = VG + 1.0;    % YAW_RATE_CMD
V(5:6, 9) = pcb.VDD;      % I_HOLD high 0.3-0.4 s
setAnalogInput([t V], 0.6);
out = sim('pid_board_tb_analog');
L = out.logsout;
gv = @(n) local_ts(L, n);
[tP, P] = gv('roll_P'); [tI, I] = gv('roll_I'); [tU, U] = gv('roll_U');
[~, Up] = gv('pitch_U'); [~, Ip] = gv('pitch_I'); [tUa, Ua] = gv('U_PITCH');
nfail = 0; r = struct();
lo = pcb.V_rail_lo; hi = pcb.V_rail_hi; vc = pcb.roll.V_iclamp;
fprintf('   %-44s %10s %10s\n', 'check', 'model', 'expected');
nfail = nfail + chk(r, 'roll P_OUT min (low rail, V re VG)', min(P), lo, 1e-3);
nfail = nfail + chk(r, 'roll I_OUT at 0.29 s (-clamp)', interp1(tI, I, 0.29), -vc, 1e-3);
nfail = nfail + chk(r, 'max |I_OUT| roll (<= clamp)', max(abs(I)), vc, 1e-3);
nfail = nfail + chk(r, 'roll U at 0.29 s = -(P+I) = -lo + clamp', interp1(tU, U, 0.29), -lo + vc, 2e-3);
fprintf('   %-44s %10.3f %10s\n', 'roll U max (incl. D kick)', max(U), sprintf('<=%.3f', hi));
nfail = nfail + (max(U) > hi + 1e-9);
nfail = nfail + chk(r, 'pitch U at 0.29 s (low rail)', interp1(tU, Up, 0.29), lo, 1e-3);
nfail = nfail + chk(r, 'pitch I_OUT at 0.29 s (+clamp)', interp1(tI, Ip, 0.29), vc, 1e-3);
nfail = nfail + chk(r, 'U_PITCH absolute at 0.29 s (VOL 25 mV)', interp1(tUa, Ua, 0.29), 0.025, 1e-3);
fprintf('\n(d) I_HOLD high 0.30-0.40 s (CD4066 Ron %.0f ohm across 1 uF)\n', pcb.ihold.R_on);
k = tI >= 0.3 & tI <= 0.4;
ti = tI(k) - 0.3; Ii = I(k);
t10 = ti(find(abs(Ii) <= 0.1*vc, 1));
tau = pcb.ihold.R_on*pcb.roll.CI;
Iss = -(1/((pcb.roll.R_I_fixed + pcb.roll.RV_I)*pcb.roll.CI))*1.818/(1/(pcb.roll.RL*pcb.roll.CI) + 1/tau);
fprintf('   %-44s %10.2f %10.2f\n', 'time to 10 % of clamp (ms) vs Ron*CI*ln10', 1e3*t10, 1e3*tau*log(10));
nfail = nfail + (abs(t10 - tau*log(10)) > pcb.sim_dt + 0.1e-3);   % resolution = solver step
nfail = nfail + chk(r, 'I_OUT held at 0.39 s (V)', interp1(tI, I, 0.39), Iss, 0.5e-3);
nfail = nfail + chk(r, 'I recharges to clamp after release (0.59 s)', interp1(tI, I, 0.59), -vc, 1e-3);
r.out = out;
end

%% ---------------------------------------------------------------------
function [r, nfail] = partE()
fprintf('\n(e) ARM gating at control_io (pid_board_bench)\n');
nfail = 0; r = struct();
pid_board_bench_inputs('disarmed');
out = sim('pid_board_bench');
[~, e] = local_ts(out.logsout, 'esc_us_bench');
fprintf('   disarmed (wobble, 50 %% throttle): esc_us min/max %g / %g (expect 1000 / 1000)  %s\n', ...
    min(e(:)), max(e(:)), passStr(all(e(:) == 1000)));
nfail = nfail + ~all(e(:) == 1000);
[~, ih] = local_ts(out.logsout, 'I_HOLD');
fprintf('   disarmed: I_HOLD always high: %s\n', passStr(all(ih > 2.5)));
nfail = nfail + ~all(ih > 2.5);
pid_board_bench_inputs('wobble');
out = sim('pid_board_bench');
[te, e] = local_ts(out.logsout, 'esc_us_bench');
ok = all(e(:) >= 1050 & e(:) <= 2000);
fprintf('   armed (wobble, 30 %% throttle): esc_us range %g..%g (expect within 1050..2000)  %s\n', ...
    min(e(:)), max(e(:)), passStr(ok));
nfail = nfail + ~ok;
r.esc_armed = [te e];
end

%% ---------------------------------------------------------------------
function [r, nfail] = partA(here)
fprintf('\n(a) Roll path vs legacy sim/roll_axis (built into tempdir as pid_board_legacy_roll)\n');
pcb = evalin('base', 'pcb');
nfail = 0; r = struct();
tmp = fullfile(tempdir, 'pid_board_legacy');
if ~isfolder(tmp), mkdir(tmp); end
src = fileread(fullfile(here, '..', 'legacy_roll_axis', 'build_roll_model.m'));
src = strrep(src, 'mdl  = ''roll_axis'';', 'mdl  = ''pid_board_legacy_roll'';');
% LEGACY BUG: build_roll_model.m never wires the Command path inport to its Ktheta gain,
% so roll_axis.slx ignores theta_cmd (angle command is always 0). Fixed in the temp copy.
src = strrep(src, 'dacChain(cp, ''cmd'', ''Ktheta/1'', ''theta_cmd_V/1'');', ...
    'wire(cp, ''theta_cmd_deg/1'', ''Ktheta/1''); dacChain(cp, ''cmd'', ''Ktheta/1'', ''theta_cmd_V/1'');');
fprintf('   note: legacy build_roll_model.m leaves the theta_cmd inport unconnected (fixed in the temp copy)\n');
src = regexprep(src,', \.\.\.\s*''PreLoadFcn'', ''[^\n]*''\);', ');');
fid = fopen(fullfile(tmp, 'build_roll_model.m'), 'w'); fwrite(fid, src); fclose(fid);
copyfile(fullfile(here, '..', 'legacy_roll_axis', 'roll_params.m'), fullfile(tmp, 'roll_params.m'));
P = legacyBuild(tmp);              % builds in tmp, returns the roll_params variables
mdl = 'pid_board_legacy_roll';
mkin = @(vars) legacyInput(mdl, P, vars);

% A1: open-loop 10 deg angle-command step, airframe frozen (KF = 0)
tS = 0.05; th = 10;
out = sim(mkin({'KF', 0, 'theta_cmd_deg', th, 't_step', tS, 't_stop', 0.6, 't_dist', 10}));
tl = out.log_U.Time; Ul = squeeze(out.log_U.Data); Il = squeeze(out.log_I.Data); El = squeeze(out.log_E.Data);
Vc = round(pcb.scale.Ktheta*th/pcb.dac.lsb)*pcb.dac.lsb;
tt = [0 tS tS 0.6]';
V = repmat(constIn(pcb), 4, 1); V(3:4, 7) = pcb.VG + Vc;
setAnalogInput([tt V], 0.6);
on = sim('pid_board_tb_analog');
[tn, Un] = local_ts(on.logsout, 'roll_U'); [~, In] = local_ts(on.logsout, 'roll_I'); [~, En] = local_ts(on.logsout, 'roll_E');
tc = (0:1e-4:0.6)';
d = @(a, b) interp1(tl, a, tc) - interp1(tn, b, tc);
fprintf('   A1 10 deg angle-cmd step, no airframe: legacy vs new\n');
fprintf('      %-26s %10s %10s %12s\n', 'signal', 'legacy', 'new', 'max |diff|');
fprintf('      %-26s %10.4f %10.4f %12.4f\n', 'E_RR at 0.1 s (V)', interp1(tl, El, 0.1), interp1(tn, En, 0.1), max(abs(d(El, En))));
fprintf('      %-26s %10.4f %10.4f %12.4f\n', 'I_OUT at 0.6 s (V)', Il(end), In(end), max(abs(d(Il, In))));
fprintf('      %-26s %10.4f %10.4f %12.4f\n', 'U_ROLL at 0.6 s (V)', Ul(end), Un(end), max(abs(d(Ul, Un))));
r.A1 = struct('maxdiff_U', max(abs(d(Ul, Un))), 'U_end', [Ul(end) Un(end)]);
ok = max(abs(d(Ul, Un))) < 0.15 && abs(Ul(end) - Un(end)) < 2e-3 && abs(Il(end) - In(end)) < 2e-3;
fprintf(['      steady state equal (<2 mV); transient diff is the D kick (legacy 159 Hz RC + gain vs\n' ...
         '      loaded 175 Hz RC, ode45 vs ode4 at %.1f ms): %s\n'], 1e3*pcb.sim_dt, passStr(ok));
nfail = nfail + ~ok;

% A2: rate loop around the 22 ms bench integrator, 100 mV step
out = sim(mkin({'bench_mode', 1, 'angle_loop_on', 0, 'rate_sp_V', 0.1, 't_stop', 0.4, 't_step', tS}));
[rl, ol, fl] = stepMetrics(out.log_FB.Time, squeeze(out.log_FB.Data), tS);
pid_board_bench_inputs('bench_roll');
pb = evalin('base', 'pbsim'); pb.t_stop = 0.4; assignin('base', 'pbsim', pb);
pcb2 = evalin('base', 'pcb'); pcb2.bench.w1_amp = 0.1; pcb2.bench.w1_t0 = tS; assignin('base', 'pcb', pcb2);
ob = sim('pid_board_bench');
[tb, pl] = local_ts(ob.logsout, 'roll_PLANT');
[rn, on2, fn] = stepMetrics(tb, pl, tS);
fprintf('   A2 bench integrator (U202D 22 ms) rate loop, 100 mV step\n');
fprintf('      %-34s %10s %10s %10s\n', '', 'rise ms', 'OS %', 'final V');
fprintf('      %-34s %10.1f %10.1f %10.4f\n', 'legacy (SP 0.1 on rate_sp, FB=+int)', 1e3*rl, ol, fl);
fprintf('      %-34s %10.1f %10.1f %10.4f\n', 'new (W1 0.1 on EXT_FB, lead->EXT_SP)', 1e3*rn, on2, fn);
fprintf(['      expected final: legacy +0.1*g_sp/g_fb = %.4f; board +W1/g_sp with g_sp = 1.000 when\n' ...
         '      JP_FB = 2 (no 1k in the FB leg) -> +0.1000. U is inverted vs legacy (W1 enters the - leg).\n'], ...
    0.1*0.955/0.909);
okA2 = abs(fn - 0.1) < 2e-3 && abs(rn/rl - 1) < 0.15;
fprintf('      final = +W1 (spec section 5 says -1.05*W1), rise/OS within 15 %%: %s\n', passStr(okA2));
nfail = nfail + ~okA2;
evalin('base', 'pid_board_params; pcb.circuit_enable = 0;');
r.A2 = struct('legacy', [rl ol fl], 'new', [rn on2 fn]);
close_system(mdl, 0);
end

function P = legacyBuild(tmp)
% run the patched build script and roll_params in THIS function's workspace
run(fullfile(tmp, 'build_roll_model.m'));
clear tmp
run(fullfile(fileparts(which('pid_board_validate')), '..', 'legacy_roll_axis', 'roll_params.m'));
v = who;
P = struct();
for k = 1:numel(v)
    if ~any(strcmp(v{k}, {'P', 'k', 'v', 'here', 'mdl', 'slx', 'cp', 'sp', 'pc', 'mx', 'af', 'bp', 's'}))
        P.(v{k}) = eval(v{k});
    end
end
end

function in = legacyInput(mdl, P, vars)
in = Simulink.SimulationInput(mdl);
for k = 1:2:numel(vars), P.(vars{k}) = vars{k+1}; end   % overrides (setVariable keeps the first)
f = fieldnames(P);
for k = 1:numel(f), in = in.setVariable(f{k}, P.(f{k})); end
end

%% ---------------------------------------------------------------------
function [r, nfail] = partF(here)
fprintf('\n(f) Closed loop: pid_board + rigid-body attitude plant (quad_params J, nominal pots)\n');
quad = loadQuad(here);
pcb = evalin('base', 'pcb');
pl = plantFromQuad(quad);
pl.x0 = [0; 0; 0.3; 10*pi/180; -8*pi/180; 0];
thr = (pl.us_h - pcb.ctrl.us_off)/(pcb.ctrl.us_max - pcb.ctrl.us_off);
S = [0   0     0     0   thr 1;
     2   0.25  0     0   thr 1;
     3   0    -0.25  0   thr 1;
     4   0     0     0.2 thr 1;
     5   0     0     0   thr 1];
pb = evalin('base', 'pbsim');
pb.plant = pl; pb.cl_sticks = S; pb.t_stop = 6;
assignin('base', 'pbsim', pb);
fprintf('   hover %.0f us (thr stick %.3f), dT/dus %.2e N/us, K_rpy %s us/V\n', pl.us_h, thr, pl.dTdus, mat2str(pcb.ctrl.K_rpy));
Kw = pcb.scale.Kw*180/pi;
fprintf('   rate-loop plant gain U->gyro V: roll %.0f /s, pitch %.0f /s, yaw %.1f /s (legacy design 45.5 /s)\n', ...
    pl.Kloop(1)*Kw, pl.Kloop(2)*Kw, pl.Kloop(3)*Kw);
out = sim('pid_board_cl');
[te, eul] = local_ts(out.logsout, 'euler_rad'); eul = eul*180/pi;
[tw, om] = local_ts(out.logsout, 'omega_radps'); om = om*180/pi;
at = @(tq, c) interp1(te, eul(:, c), tq);
nfail = 0; r = struct('t', te, 'eul_deg', eul, 'omega_dps', om);
fprintf('   %-46s %9s %9s\n', 'check', 'model', 'target');
rows = {'roll after 1.9 s (initial 10 deg)', at(1.9, 1), 0, 2;
        'pitch after 1.9 s (initial -8 deg)', at(1.9, 2), 0, 2;
        'roll at 2.9 s (stick 0.25 -> 15 deg)', at(2.9, 1), 15, 2;
        'pitch at 3.9 s (stick -0.25 -> +15 deg nose up)', at(3.9, 2), 15, 2;
        'yaw rate at 4.9 s (stick 0.2 -> 50 deg/s)', interp1(tw, om(:, 3), 4.9), 50, 10;
        'roll at 5.9 s (back to level)', at(5.9, 1), 0, 2};
for k = 1:size(rows, 1)
    ok = abs(rows{k, 2} - rows{k, 3}) <= rows{k, 4};
    fprintf('   %-46s %9.2f %9.2f  %s\n', rows{k, 1}, rows{k, 2}, rows{k, 3}, passStr(ok));
    nfail = nfail + ~ok;
end
k = tw > 5.5;
rr = sqrt(mean(om(k, 1:2).^2));
fprintf('   %-46s %9.2f %9s  %s\n', 'rms roll/pitch rate 5.5-6 s (deg/s)', max(rr), '< 5', passStr(max(rr) < 5));
nfail = nfail + (max(rr) >= 5);
k2 = te >= 2 & te < 3;
os = max(eul(k2, 1)) - 15;
fprintf('   %-46s %9.2f\n', 'roll step overshoot (deg)', os);
r.overshoot_roll = os;
end

function quad = loadQuad(here)
f = fullfile(here, '..', 'quad_plant', 'quad_params.m');
run(f);
end

function pl = plantFromQuad(quad)
% Linearised motor/prop: T_i = T_h + dT/dus*(us - us_h), first-order lag, ESC latency
pl.g = quad.g;
pl.J = quad.inertia.J; pl.Jinv = inv(pl.J);
pl.rx = quad.geom.r_motor(1, :); pl.ry = quad.geom.r_motor(2, :);
pl.spin = quad.motor.spin;
pl.kQT = quad.prop.kQ/quad.prop.kT;
pl.krot = quad.drag.k_rot(:);
w = quad.checks.w_hover;
dVdw = 1/quad.motor.Kv + 2*quad.motor.R*quad.prop.kQ*w/quad.motor.Kt;
dwdduty = quad.checks.Vbus_hover/dVdw;
pl.T_h = quad.checks.T_hover;
pl.dTdus = 2*quad.prop.kT*w*dwdduty/(quad.esc.pwm_max_us - quad.esc.pwm_min_us);
pl.us_h = quad.esc.pwm_min_us + quad.checks.hover_duty*(quad.esc.pwm_max_us - quad.esc.pwm_min_us);
pl.T_max = quad.checks.T_max(2);
pl.tau = quad.motor.tau;
pl.t_lat = 2.2e-3;            % pulse + ESC processing; frame hold is in the board model
pl.vbat = quad.checks.Vbus_hover;
pl.x0 = zeros(6, 1);
pcb = evalin('base', 'pcb');
% rad/s^2 per volt of U (mixer columns, Betaflight pitch = nose down)
Kr = pcb.ctrl.K_rpy;
tx = sum(-pl.ry.*pcb.ctrl.mix(:, 2)')*Kr(1)*pl.dTdus;
ty = sum(pl.rx.*pcb.ctrl.mix(:, 3)')*Kr(2)*pl.dTdus;
tz = sum(-pl.spin.*pcb.ctrl.mix(:, 4)')*Kr(3)*pl.dTdus*pl.kQT;
pl.Kloop = abs([tx/pl.J(1, 1), ty/pl.J(2, 2), tz/pl.J(3, 3)]);   % rad/s^2 per V
end

%% ---------------------------------------------------------------------
function v = constIn(pcb)
v = pcb.VG*ones(1, 15); v(9) = 0;
end

function setAnalogInput(M, tStop)
if size(M, 2) == 15, M = [0 M; 1 M]; end
pb = struct();
if evalin('base', 'exist(''pbsim'', ''var'')'), pb = evalin('base', 'pbsim'); end
pb.an_in = M; pb.t_stop = tStop;
assignin('base', 'pbsim', pb);
end

function [t, y] = local_ts(L, name)
ts = L.get(name).Values;
t = ts.Time;
y = squeeze(ts.Data);
if size(y, 1) ~= numel(t), y = y.'; end
end

function [rise, os, yf] = stepMetrics(t, y, tStep)
keep = t >= tStep;
t = t(keep) - tStep; y = y(keep) - y(find(keep, 1));
yf = y(end);
S = stepinfo(y, t, yf, 'RiseTimeLimits', [0.1 0.9]);
rise = S.RiseTime; os = S.Overshoot;
end

function n = chk(~, name, model, expect, tol)
% tol < 0.005 -> absolute tolerance in volts, else relative tolerance
if tol < 0.005
    ok = abs(model - expect) <= tol;
else
    ok = abs(model/expect - 1) <= tol;
end
fprintf('   %-44s %10.4f %10.4f %+7.2f%%  %s\n', name, model, expect, 100*(model/expect - 1), passStr(ok));
n = ~ok;
end

function restorePcb(had, p)
if had, assignin('base', 'pcb', p); else, evalin('base', 'pid_board_params;'); end
end

function s = passStr(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end
