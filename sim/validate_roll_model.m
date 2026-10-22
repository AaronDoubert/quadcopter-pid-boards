%VALIDATE_ROLL_MODEL Check roll_axis.slx against the briefing before using it.
%   1. Guide checks of the P, I and D stages (open-loop gain, V/V).
%   2. Rate loop closed around the 22 ms bench integrator: rise and overshoot.
%   3. Loop budget table: crossover, phase and gain margin per case.
%   Each check passes within 5%. Checks 1-3 use the briefing's ideal
%   diff-amp gains (1.0); check 2 and 3 also print the board's
%   source-impedance gains (0.955 / 0.909) for information.

here = fileparts(mfilename('fullpath'));
addpath(here);
mdl = 'roll_axis';
if ~isfile(fullfile(here, [mdl '.slx'])), build_roll_model; end
roll_params;
load_system(mdl);
tol = 0.05;
nfail = 0;

%% 1. Guide checks: stage gains linearised from the model
[sl, pt] = roll_linearizer(mdl);
checks = { 'P', 1,              1.16;
           'I', [0.1 1 10],     [8.5 1.57 0.16];
           'D', [10 59 100],    [0.60 2.39 2.64] };
fprintf('\n1. Controller guide checks (nominal pots)\n');
fprintf('   %-5s %8s %10s %10s %8s\n', 'Stage', 'f (Hz)', 'model', 'guide', 'error');
for k = 1:size(checks, 1)
    H = getIOTransfer(sl, pt.E, pt.(checks{k, 1}), pt.E);
    f = checks{k, 2};
    g = abs(squeeze(freqresp(H, 2*pi*f)))';
    for n = 1:numel(f)
        err = g(n)/checks{k, 3}(n) - 1;
        ok  = abs(err) <= tol;
        nfail = nfail + ~ok;
        fprintf('   %-5s %8.2f %10.3f %10.2f %+7.1f%%  %s\n', checks{k, 1}, f(n), g(n), ...
            checks{k, 3}(n), 100*err, passStr(ok));
    end
end

%% 2. Rate loop around the bench integrator, 100 mV setpoint step
fprintf('\n2. Closed around the 22 ms bench integrator (100 mV rate step)\n');
cases = {'guide (ideal legs 1.0 / 1.0)', 1, 1, true;
         'bench wiring (SP 0.909 / FB 0.955)', 0.909, 0.955, false};
fprintf('   %-36s %10s %10s\n', 'Diff-amp gains', 'rise (ms)', 'OS (%)');
for k = 1:size(cases, 1)
    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('bench_mode', 1).setVariable('angle_loop_on', 0) ...
           .setVariable('rate_sp_V', 0.1).setVariable('g_sp', cases{k, 2}) ...
           .setVariable('g_fb', cases{k, 3}).setVariable('t_stop', 0.4);
    out = sim(in);
    % integrating plant: FB settles at SP*g_sp/g_fb
    [rise, os] = stepMetrics(out.log_FB, t_step, 0.1*cases{k, 2}/cases{k, 3});
    if cases{k, 4}
        ok = abs(rise/38e-3 - 1) <= tol && abs(os/8 - 1) <= tol;
        nfail = nfail + ~ok;
        fprintf('   %-36s %10.1f %10.1f   guide 38 ms / 8%%  %s\n', cases{k, 1}, 1e3*rise, os, passStr(ok));
    else
        fprintf('   %-36s %10.1f %10.1f   (info)\n', cases{k, 1}, 1e3*rise, os);
    end
end

%% 3. Loop budget, cumulative cases at nominal pots
fprintf('\n3. Loop budget (loop broken at U, plant gain 45.5 /s)\n');
%           name                              bench  t_dlpf  tau_m  fc    PM   GM
budget = {'Ideal integrator (bench plant)',     1,  0,      0,     9.9, 108, Inf;
          '+ RC filter, 1 kHz firmware hold',   0,  0,      0,     9.9, 103, 19;
          '+ IMU DLPF 98 Hz (3.3 ms total)',    0,  2.8e-3, 0,     9.9,  93,  9;
          '+ motor lag 20 ms',                  0,  2.8e-3, 0.02,  6.8,  47, 16;
          '+ motor lag 40 ms',                  0,  2.8e-3, 0.04,  5.3,  30, 20;
          '+ motor lag 80 ms',                  0,  2.8e-3, 0.08,  4.0,  13, 25};
mustPass = [1 4 5];   % the task's 108 / 47 / 30 deg cases
fgrid = logspace(-2, log10(500), 4000);
for gains = [1 1; g_sp g_fb]'
    fprintf('   Diff-amp gains SP %.3f / FB %.3f\n', gains(1), gains(2));
    fprintf('   %-34s %13s %13s %13s\n', 'Case', 'fc (Hz)', 'PM (deg)', 'GM (dB)');
    for k = 1:size(budget, 1)
        assignin('base', 'g_sp', gains(1));
        assignin('base', 'g_fb', gains(2));
        assignin('base', 'bench_mode', budget{k, 2});
        assignin('base', 't_dlpf', budget{k, 3});
        assignin('base', 'tau_m', budget{k, 4});
        assignin('base', 'angle_loop_on', 0);
        refresh(sl);
        Lw = squeeze(freqresp(getLoopTransfer(sl, pt.U, -1), 2*pi*fgrid));
        [fc, pm, gm] = loop_margins(Lw, fgrid);
        line = sprintf('   %-34s %5.1f (%4.1f) %5.0f (%4.0f) %5.1f (%4.0f)', budget{k, 1}, ...
            fc, budget{k, 5}, pm, budget{k, 6}, gm, budget{k, 7});
        if gains(2) == 1 && ismember(k, mustPass)
            ok = abs(pm/budget{k, 6} - 1) <= tol;
            nfail = nfail + ~ok;
            line = [line '  ' passStr(ok)]; %#ok<AGROW>
        end
        fprintf('%s\n', line);
    end
end
fprintf('   (briefing values in brackets)\n');
roll_params;

fprintf('\n%s: %d check(s) failed\n', passStr(nfail == 0), nfail);

%% Local helpers
function [rise, os] = stepMetrics(ts, tStep, yf)
% 10-90% rise time and overshoot of a step that starts at tStep
t = ts.Time;
y = squeeze(ts.Data);
keep = t >= tStep;
S = stepinfo(y(keep), t(keep) - tStep, yf, 'RiseTimeLimits', [0.1 0.9]);
rise = S.RiseTime;
os = S.Overshoot;
end

function s = passStr(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end
