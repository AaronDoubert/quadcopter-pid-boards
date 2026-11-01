function R = pid_analog_circuit_test(varargin)
%PID_ANALOG_CIRCUIT_TEST Test harness for pid_analog_circuit.slx (Simscape circuit variant).
%   R = pid_analog_circuit_test()            all tests
%   R = pid_analog_circuit_test('steps','sweep','limits','ref','fast','speed')  a subset
%   R = pid_analog_circuit_test(..., 'Ts', 20e-6)   override pacirc.Ts for this run only
%
%   Drives the 15 inports (absolute DAC/TP volts, ANALOG_INTERFACE.md) and compares the
%   circuit against the analytic transfer functions of pid_board_spec.md section 4:
%     U - VG = [Kp + Ki/(s + wL) + Kd s/((1 + s/w1)(1 + s/w2))] * (g_sp SP - g_fb FB)
%   with the DAC RC (0.909 ms inverting legs, 0.952 ms non-inverting legs), the angle stage
%   (E_A = 0.909 (theta - theta_cmd), SP = -Go E_A) and the yaw cell (no D, g_sp = 0.909).
%     steps  : per-axis small steps; output and every stage (P, I, D, E, E_A, SP) vs analytic
%     sweep  : 10-tone multisine 0.4-300 Hz per axis; gain/phase vs analytic
%     limits : rail saturation, I-clamp level (1N4148 pairs), I_HOLD reset (CD4066)
%     ref    : VGND_A from the TL431/RV101 reference (trimmed and the 8.1k label value)
%     fast   : same inputs into pid_analog_fast.slx (if present): max error, gain/phase
%     speed  : wall time for 1 s and 10 s (open-loop drive) and, if available, closed loop
%   Results are printed and returned in R. Requires pcb (pid_board_params.m) and pacirc
%   (pid_analog_circuit_params.m) in the base workspace (loaded if missing).

here = fileparts(mfilename('fullpath'));
tests = varargin; TsOverride = [];
k = find(strcmp(tests, 'Ts'));
if ~isempty(k), TsOverride = tests{k + 1}; tests(k:k + 1) = []; end
if isempty(tests), tests = {'steps', 'sweep', 'limits', 'ref', 'fast', 'speed'}; end
evalin('base', sprintf('if ~exist(''pcb'',''var''), run(''%s''); end', fullfile(here, 'pid_board_params.m')));
evalin('base', sprintf('if ~exist(''pacirc'',''var''), run(''%s''); end', fullfile(here, 'pid_analog_circuit_params.m')));
if ~isempty(TsOverride)
    Ts0 = evalin('base', 'pacirc.Ts');
    evalin('base', sprintf('pacirc.Ts = %.12g;', TsOverride));
    cTs = onCleanup(@() evalin('base', sprintf('pacirc.Ts = %.12g;', Ts0)));
end
pcb = evalin('base', 'pcb');
mdl = 'pid_analog_circuit';
load_system(fullfile(here, [mdl '.slx']));
hasFast = isfile(fullfile(here, 'pid_analog_fast.slx'));
R = struct();
R.Ts = evalin('base', 'pacirc.Ts');
VG = pcb.VG;
base = [VG*ones(1, 8), 0, VG*ones(1, 6)];   % 15 inputs at rest (absolute volts)
C = cols();
A = analytic(pcb);
s = tf('s');

%% ---------------------------------------------------------------- steps
if any(strcmp(tests, 'steps')) || any(strcmp(tests, 'fast'))
    T = 0.4; t0 = 0.02;
    [t, U] = stepInput(base, T, t0, {C.ROLL_RATE, 0.05; C.PITCH_ANGLE_CMD, 0.04; C.YAW_RATE_CMD, 0.05});
    S = runsim(mdl, t, U, T, 1);
    R.steps.wall = S.wall;
    tg = (0:R.Ts:T)';
    u = @(c, a) a*(tg >= t0);
    % analytic outputs (re VG)
    Yr = lsim(A.roll.H * (-A.roll.g_fb) / (1 + s*pcb.dac.tau_inv_leg), u(0, 0.05), tg);
    Yp = lsim(A.pitch.H * A.pitch.g_sp * A.pitch.Go * A.pitch.g_ang_cmd / (1 + s*pcb.dac.tau_inv_leg), u(0, 0.04), tg);
    Yy = lsim(A.yaw.H * A.yaw.g_sp / (1 + s*pcb.dac.tau_ni_leg), u(0, 0.05), tg);
    yr = rs(S, 'U_ROLL', tg) - VG; yp = rs(S, 'U_PITCH', tg) - VG; yy = rs(S, 'U_YAW', tg) - VG;
    yr = yr - yr(1); yp = yp - yp(1); yy = yy - yy(1);   % remove static offset
    R.steps.U_err = [nerr(yr, Yr), nerr(yp, Yp), nerr(yy, Yy)];
    R.steps.U_final = [yr(end) Yr(end); yp(end) Yp(end); yy(end) Yy(end)];
    % stage-level (from logged test points, re VG)
    for ax = {'roll', 'pitch', 'yaw'}
        a = ax{1}; Ax = A.(a);
        E = tp(S, [a '_E'], tg); E = E - E(1);
        P = tp(S, [a '_P'], tg); P = P - P(1);
        I = tp(S, [a '_I'], tg); I = I - I(1);
        D = tp(S, [a '_D'], tg); D = D - D(1);
        st.Kp_meas = -P(end)/E(end); st.Kp = Ax.Kp;
        st.I_err = nerr(I, lsim(-Ax.Ki/(s + Ax.wL), E, tg));
        st.D_err = nerr(D, lsim(-Ax.Dtf, E, tg));
        st.P_err = nerr(P, -Ax.Kp*E);
        if ~strcmp(a, 'yaw')
            EA = tp(S, [a '_E_A'], tg); EA = EA - EA(1);
            SP = tp(S, [a '_RATE_SP'], tg); SP = SP - SP(1);
            if strcmp(a, 'pitch')
                st.Go_meas = -SP(end)/EA(end); st.Go = Ax.Go;
                st.gEA_meas = -EA(end)/0.04; st.gEA = Ax.g_ang_cmd;
                st.g_sp_meas = E(end)/SP(end); st.g_sp = Ax.g_sp;
            else
                st.g_fb_meas = -E(end)/0.05; st.g_fb = Ax.g_fb;
            end
        else
            st.g_sp_meas = E(end)/0.05; st.g_sp = Ax.g_sp;
        end
        R.steps.(a) = st; clear st
    end
    R.steps.data = struct('t', tg, 'U', [yr yp yy], 'U_ana', [Yr Yp Yy]);
    fprintf('\n== Step tests (roll FB +50 mV, pitch ANGLE_CMD +40 mV, yaw CMD +50 mV) ==\n');
    fprintf('U vs analytic, max |err| / max |U|: roll %.2f %%, pitch %.2f %%, yaw %.2f %%\n', 100*R.steps.U_err);
    fprintf('U final (circuit / analytic) re VG: roll %.4f/%.4f  pitch %.4f/%.4f  yaw %.4f/%.4f V\n', R.steps.U_final');
    for ax = {'roll', 'pitch', 'yaw'}
        st = R.steps.(ax{1});
        fprintf('%-5s Kp %.4f (ana %.4f)  P err %.2f %%  I err %.2f %%  D err %.2f %%', ax{1}, st.Kp_meas, st.Kp, ...
            100*st.P_err, 100*st.I_err, 100*st.D_err);
        if isfield(st, 'Go'), fprintf('  Go %.4f (%.4f) g_EA %.4f (%.4f) g_sp %.4f (%.4f)', st.Go_meas, st.Go, st.gEA_meas, st.gEA, st.g_sp_meas, st.g_sp); end
        if isfield(st, 'g_fb'), fprintf('  g_fb %.4f (%.4f)', st.g_fb_meas, st.g_fb); end
        if strcmp(ax{1}, 'yaw'), fprintf('  g_sp %.4f (%.4f)', st.g_sp_meas, st.g_sp); end
        fprintf('\n');
    end
    stepIn = struct('t', t, 'U', U, 'T', T, 'S', S);
end

%% ---------------------------------------------------------------- sweep
if any(strcmp(tests, 'sweep')) || any(strcmp(tests, 'fast'))
    f = [0.4 1 2 5 10 20 50 100 200 300]; amp = 4e-3; Tw = 5; T = 9;
    dt = 1e-4; t = (0:dt:T)'; rng(1); ph = 2*pi*rand(3, numel(f));
    ms = @(k) amp*sum(sin(2*pi*t*f + ph(k, :)), 2);
    U = repmat(base, numel(t), 1);
    U(:, C.ROLL_RATE) = VG + ms(1); U(:, C.PITCH_ANGLE) = VG + ms(2); U(:, C.YAW_RATE) = VG + ms(3);
    S = runsim(mdl, t, U, T, 1);
    R.sweep.wall = S.wall;
    tg = (0:R.Ts:T)'; x = [interp1(t, ms(1), tg), interp1(t, ms(2), tg), interp1(t, ms(3), tg)];
    G = {A.roll.H * (-A.roll.g_fb) / (1 + s*pcb.dac.tau_inv_leg), ...
         -A.pitch.H * A.pitch.g_sp * A.pitch.Go * A.pitch.g_ang_meas / (1 + s*pcb.dac.tau_ni_leg), ...
         A.yaw.H * (-A.yaw.g_fb) / (1 + s*pcb.dac.tau_inv_leg)};
    outs = {'U_ROLL', 'U_PITCH', 'U_YAW'};
    win = tg >= T - Tw;
    fprintf('\n== Multisine 0.4-300 Hz (4 mV/tone): circuit / analytic ==\n');
    for k = 1:3
        y = rs(S, outs{k}, tg) - VG; y = y - mean(y(tg < 0.01));
        ya = lsim(G{k}, x(:, k), tg);
        Hc = dft(y, tg, win, f) ./ dft(x(:, k), tg, win, f);
        Ha = dft(ya, tg, win, f) ./ dft(x(:, k), tg, win, f);
        Hs = squeeze(freqresp(G{k}, 2*pi*f)).';
        R.sweep.(outs{k}) = struct('f', f, 'H_circ', Hc, 'H_ana_sim', Ha, 'H_ana', Hs, ...
            'dB', 20*log10(abs(Hc./Ha)), 'dphase', angle(Hc./Ha)*180/pi);
        fprintf('%-8s f [Hz]   ', outs{k}); fprintf('%8.1f', f); fprintf('\n');
        fprintf('         |H| dB   '); fprintf('%8.2f', 20*log10(abs(Hc))); fprintf('\n');
        fprintf('         dGain dB '); fprintf('%8.2f', R.sweep.(outs{k}).dB); fprintf('\n');
        fprintf('         dPhase deg'); fprintf('%8.2f', R.sweep.(outs{k}).dphase); fprintf('\n');
    end
    sweepIn = struct('t', t, 'U', U, 'T', T, 'S', S, 'f', f, 'x', x, 'tg', tg, 'win', win);
end

%% ---------------------------------------------------------------- limits
if any(strcmp(tests, 'limits'))
    T = 1.5; dt = 1e-4; t = (0:dt:T)';
    U = repmat(base, numel(t), 1);
    on = t >= 0.02 & t < 1.3;
    U(on, C.ROLL_RATE) = 0;             % DAC code 0: E_RR = +1.86 V, P_OUT at -rail, D kick -> U_ROLL to +rail,
                                        % roll I_OUT to -clamp (U > +2.02 V re VG needs the D term: P_OUT >= -2.02 V)
    U(on, C.PITCH_RATE) = VG + 1.0;     % E_PR = -0.91 V -> U_PITCH to -rail, I_OUT to +clamp
    U(on, C.YAW_RATE) = VG + 0.3;       % E_YR = -0.27 V -> yaw I_OUT to +clamp
    U(t >= 1.0 & t < 1.4, C.I_HOLD) = pcb.VDD;   % CD4066 closes at 1.0 s
    S = runsim(mdl, t, U, T);
    tg = (0:R.Ts:T)';
    L.U_ROLL_max = max(rs(S, 'U_ROLL', tg)); L.U_PITCH_min = min(rs(S, 'U_PITCH', tg));
    L.rail = [pcb.VG + pcb.V_rail_lo, pcb.VG + pcb.V_rail_hi];
    for ax = {'roll', 'pitch', 'yaw'}
        I = tp(S, [ax{1} '_I'], tg);
        L.([ax{1} '_Iclamp']) = mean(I(tg > 0.9 & tg < 0.99));
        L.([ax{1} '_Ihold']) = mean(I(tg > 1.2 & tg < 1.29));
        k0 = find(tg >= 1.0, 1); I0 = I(k0) - L.([ax{1} '_Ihold']);
        k1 = find(abs(I(k0:end) - L.([ax{1} '_Ihold'])) < 0.1*abs(I0), 1);
        L.([ax{1} '_t10']) = (k1 - 1)*R.Ts;   % s, time to 10 % after I_HOLD
    end
    E = tp(S, 'roll_E', tg); L.roll_E = mean(E(tg > 0.9 & tg < 0.99));
    R.limits = L;
    fprintf('\n== Limits ==\n');
    fprintf('Rails (abs): U_ROLL max %.4f V, U_PITCH min %.4f V (pcb rails %.3f / %.3f V)\n', ...
        L.U_ROLL_max, L.U_PITCH_min, L.rail(2), L.rail(1));
    fprintf('I clamp (re VG): roll %.3f V, pitch %.3f V, yaw %.3f V (pcb.V_iclamp %.2f V)\n', ...
        L.roll_Iclamp, L.pitch_Iclamp, L.yaw_Iclamp, pcb.roll.V_iclamp);
    fprintf('I_HOLD: residual roll %.1f mV, pitch %.1f mV, yaw %.1f mV; 90 %% discharge in %.2f / %.2f / %.2f ms (Ron*CI = %.2f ms)\n', ...
        1e3*[L.roll_Ihold L.pitch_Ihold L.yaw_Ihold], 1e3*[L.roll_t10 L.pitch_t10 L.yaw_t10], 1e3*pcb.ihold.R_on*pcb.roll.CI);
end

%% ---------------------------------------------------------------- ref
if any(strcmp(tests, 'ref'))
    T = 0.02; t = [0; T]; U = repmat(base, 2, 1);
    S = runsim(mdl, t, U, T);
    R.ref.VGND_A_trim = S.VGND_A(end);
    wg = evalin('base', 'pacirc.ref.RV_wg');
    evalin('base', 'pacirc.ref.RV_wg = pcb.ref.RV_set_wiper_gnd;');
    c = onCleanup(@() evalin('base', sprintf('pacirc.ref.RV_wg = %.12g;', wg)));
    S = runsim(mdl, t, U, T);
    R.ref.VGND_A_label = S.VGND_A(end);
    clear c
    fprintf('\n== Reference ==\nVGND_A trimmed: %.4f V (target %.4f); with RV101 label 8.1k: %.4f V\n', ...
        R.ref.VGND_A_trim, VG, R.ref.VGND_A_label);
end

%% ---------------------------------------------------------------- fast
if any(strcmp(tests, 'fast'))
    if ~hasFast
        fprintf('\npid_analog_fast.slx not found: skipping circuit vs fast.\n');
    else
        load_system(fullfile(here, 'pid_analog_fast.slx'));
        outs = {'U_ROLL', 'U_PITCH', 'U_YAW', 'VGND_A'};
        F = runsim('pid_analog_fast', stepIn.t, stepIn.U, stepIn.T);
        tg = (0:R.Ts:stepIn.T)';
        fprintf('\n== Circuit vs fast (steps) ==\n');
        for k = 1:4
            yc = rs(stepIn.S, outs{k}, tg); yf = rs(F, outs{k}, tg);
            R.fast.step.(outs{k}) = struct('max_abs_err', max(abs(yc - yf)), ...
                'max_err_offset_removed', max(abs((yc - yc(1)) - (yf - yf(1)))), 'static', yc(1) - yf(1));
            fprintf('%-8s max |circ-fast| %.2f mV (offset removed %.2f mV, static offset %.2f mV)\n', outs{k}, ...
                1e3*R.fast.step.(outs{k}).max_abs_err, 1e3*R.fast.step.(outs{k}).max_err_offset_removed, 1e3*R.fast.step.(outs{k}).static);
        end
        F = runsim('pid_analog_fast', sweepIn.t, sweepIn.U, sweepIn.T);
        fprintf('== Circuit vs fast (multisine), circ/fast ==\n');
        tg = sweepIn.tg; f = sweepIn.f;
        for k = 1:3
            yc = rs(sweepIn.S, outs{k}, tg); yf = rs(F, outs{k}, tg);
            Hc = dft(yc - mean(yc), tg, sweepIn.win, f); Hf = dft(yf - mean(yf), tg, sweepIn.win, f);
            R.fast.sweep.(outs{k}) = struct('f', f, 'dB', 20*log10(abs(Hc./Hf)), 'dphase', angle(Hc./Hf)*180/pi, ...
                'max_abs_err', max(abs((yc - yc(1)) - (yf - yf(1)))));
            fprintf('%-8s dGain dB  ', outs{k}); fprintf('%8.2f', R.fast.sweep.(outs{k}).dB); fprintf('\n');
            fprintf('         dPhase deg'); fprintf('%8.2f', R.fast.sweep.(outs{k}).dphase);
            fprintf('   (max |err| %.2f mV)\n', 1e3*R.fast.sweep.(outs{k}).max_abs_err);
        end
    end
end

%% ---------------------------------------------------------------- speed
if any(strcmp(tests, 'speed'))
    fprintf('\n== Speed (open-loop drive: roll/pitch/yaw sines on all 8 DAC inputs) ==\n');
    for T = [1 10]
        t = (0:1e-3:T)'; U = repmat(base, numel(t), 1);
        U(:, 1:8) = VG + 0.05*sin(2*pi*(1:8).*t*0.7);
        U(:, C.I_HOLD) = pcb.VDD*(t < 0.1);
        S = runsim(mdl, t, U, T);
        R.speed.(sprintf('T%d', T)) = S.timing;
        fprintf('%2d s simulated: init %.1f s, run %.1f s, total %.1f s (Ts = %g us)\n', T, ...
            S.timing.InitializationElapsedWallTime, S.timing.ExecutionElapsedWallTime, S.timing.TotalElapsedWallTime, 1e6*R.Ts);
    end
end
end

%% ======================================================================
function C = cols()
n = {'ROLL_ANGLE', 'ROLL_RATE', 'PITCH_ANGLE', 'PITCH_RATE', 'YAW_RATE', 'YAW_RATE_CMD', ...
    'ROLL_ANGLE_CMD', 'PITCH_ANGLE_CMD', 'I_HOLD', 'EXT_SP_ROLL', 'EXT_FB_ROLL', ...
    'EXT_SP_PITCH', 'EXT_FB_PITCH', 'EXT_SP_YAW', 'EXT_FB_YAW'};
for k = 1:numel(n), C.(n{k}) = k; end
end

function A = analytic(pcb)
s = tf('s');
for ax = {'roll', 'pitch', 'yaw'}
    p = pcb.(ax{1});
    a.Kp = (p.R_P_fixed + p.RV_P)/p.R_P_in;
    a.Ki = 1/((p.R_I_fixed + p.RV_I)*p.CI);
    a.wL = 1/(p.RL*p.CI);
    Rd = p.Rd_fixed + p.RV_D;
    a.Kd = Rd*p.Cd;
    a.Dtf = s*Rd*p.Cd/((1 + s*p.Rs*p.Cd)*(1 + s*Rd*p.Cf));
    a.H = p.en_P*a.Kp + p.en_I*a.Ki/(s + a.wL) + p.en_D*a.Dtf;
    a.g_sp = p.g_sp; a.g_fb = p.g_fb;
    if isfield(p, 'RV_A') && ~strcmp(ax{1}, 'yaw')
        a.Go = (p.R_A_fixed + p.RV_A)/p.R_A_in;
        a.g_ang_cmd = p.g_ang_cmd; a.g_ang_meas = p.g_ang_meas;
    end
    A.(ax{1}) = a; clear a
end
end

function [t, U] = stepInput(base, T, t0, steps)
t = [0; t0 - 1e-9; t0; T];
U = repmat(base, 4, 1);
for k = 1:size(steps, 1)
    U(3:4, steps{k, 1}) = U(3:4, steps{k, 1}) + steps{k, 2};
end
end

function S = runsim(mdl, t, U, T, baseline)
% baseline = 1: also simulate the rest input (all inputs at their t = 0 values) and
% subtract it from every output and test point, which removes the slow drift of the
% leaky integrators / bench plants from the sub-mV static offsets (VSET trim, VGND_B).
if nargin < 5, baseline = 0; end
mk = @(tt, UU) Simulink.SimulationInput(mdl).setExternalInput([tt UU]).setModelParameter( ...
    'StopTime', num2str(T), 'SaveOutput', 'on', ...
    'SaveFormat', 'StructureWithTime', 'SignalLogging', 'on', 'SignalLoggingName', 'logsout', ...
    'ReturnWorkspaceOutputs', 'on', 'SaveTime', 'on');
in = mk(t, U);
if strcmp(mdl, 'pid_analog_fast')
    % the fast model is Fixed-step ode4 'auto' (StopTime/50): use the board rate instead
    in = in.setModelParameter('FixedStep', '5e-4');
end
if baseline, in(2) = mk([0; T], repmat(U(1, :), 2, 1)); end
tic;
if baseline
    outs = sim(in, 'UseFastRestart', 'on', 'ShowProgress', 'off', 'ShowSimulationManager', 'off');
else
    outs = sim(in);
end
S.wall = toc;
S = unpack(outs(1), S);
if baseline
    B = unpack(outs(2), struct());
    nm = fieldnames(S);
    for k = 1:numel(nm)
        if isnumeric(S.(nm{k})) && isfield(B, nm{k}) && numel(S.(nm{k})) == numel(S.t) && ~strcmp(nm{k}, 't')
            S.(nm{k}) = S.(nm{k}) - interp1(B.t, B.(nm{k}), S.t, 'linear', 'extrap') + B.(nm{k})(1);
        end
    end
    nm = fieldnames(S.tp);
    for k = 1:numel(nm)
        v = S.tp.(nm{k}); b = B.tp.(nm{k});
        S.tp.(nm{k}) = timeseries(v.Data - interp1(b.Time, double(b.Data), v.Time, 'linear', 'extrap') ...
            + double(b.Data(1)), v.Time);
    end
end
end

function S = unpack(out, S)
S.timing = out.SimulationMetadata.TimingInfo;
y = out.yout; S.t = y.time;
for k = 1:numel(y.signals)
    nm = y.signals(k).blockName; nm = nm(find(nm == '/', 1, 'last') + 1:end);
    S.(nm) = y.signals(k).values;
end
S.tp = struct();
L = out.logsout;
if ~isempty(L)
    for k = 1:L.numElements
        e = L.getElement(k); nm = e.Name;
        if ~isempty(nm) && isvarname(nm), S.tp.(nm) = e.Values; end
    end
end
end

function y = rs(S, name, tg)
[tt, iu] = unique(S.t, 'last');
y = interp1(tt, S.(name)(iu), tg, 'linear', 'extrap');
end

function y = tp(S, name, tg)
v = S.tp.(name);
[tt, iu] = unique(v.Time, 'last');
y = interp1(tt, double(v.Data(iu)), tg, 'linear', 'extrap');
end

function e = nerr(y, ya)
e = max(abs(y(:) - ya(:)))/max(abs(ya(:)));
end

function X = dft(y, t, win, f)
tw = t(win); yw = y(win);
X = arrayfun(@(fk) sum(yw.*exp(-1i*2*pi*fk*tw)), f);
end
