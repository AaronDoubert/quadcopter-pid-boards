function make_flight_gif(scenario, gif_file, varargin)
%MAKE_FLIGHT_GIF Animate a quad_sim scenario: 3-D quad plus attitude/motor traces.
%   make_flight_gif('roll_step', 'img/flight_roll_step.gif')
%   make_flight_gif(..., 'ctrl', 2, 'title', 'PID board twin', 'fps', 25, 'speed', 0.5)
%   Uses the controller selected by 'ctrl' (qsim.ctrl: 1 ideal, 2 board fast,
%   3 board circuit). 'speed' < 1 plays in slow motion.

o = struct('ctrl', [], 'title', '', 'fps', 20, 'speed', 1, 'tspan', []);
for k = 1:2:numel(varargin), o.(varargin{k}) = varargin{k+1}; end
here = fileparts(mfilename('fullpath'));
qp = fullfile(here, '..', 'quad_plant');
addpath(qp);
if ~evalin('base', 'exist(''qsim'',''var'')')
    evalin('base', sprintf('run(''%s'')', fullfile(qp, 'quad_sim_params.m')));
end
if ~isempty(o.ctrl)
    q0 = evalin('base', 'qsim');
    cleanup = onCleanup(@() assignin('base', 'qsim', q0));
    q = q0; q.ctrl = o.ctrl; assignin('base', 'qsim', q);
end
M = run_quad_scenarios({scenario}, 'plot', false, 'save', false);
S = M.(matlab.lang.makeValidName(scenario)).sig;
quad = evalin('base', 'quad');
if isempty(o.title), o.title = strrep(scenario, '_', ' '); end
if ~isempty(o.tspan), k = S.t >= o.tspan(1) & S.t <= o.tspan(2); else, k = true(size(S.t)); end

t = S.t(k); eul = rad2deg(S.euler(k,:)); Xe = S.Xe(k,:); rpm = S.motor_rpm(k,:);
pqr = rad2deg(S.pqr(k,:)); sticks = S.sticks(k,:);
fr = (t(1):o.speed/o.fps:t(end))';

% Geometry in body axes (x fwd, y right, z down), drawn with z up
rm = quad.geom.p_motor_frame;  R = 0.0635;               % 5" prop radius
ang = linspace(0, 2*pi, 30);

fig = figure('Color', 'w', 'Position', [100 100 1100 520], 'Visible', 'off');
tl = tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, o.title, 'FontWeight', 'bold');
ax3 = nexttile(tl, 1, [3 1]); hold(ax3, 'on'); axis(ax3, 'equal'); grid(ax3, 'on');
view(ax3, 35, 20); xlabel(ax3, 'x fwd'); ylabel(ax3, 'y left'); zlabel(ax3, 'up');
lim = 0.22; axis(ax3, [-lim lim -lim lim -lim lim]);
hArm = gobjects(4,1); hProp = gobjects(4,1); hNose = plot3(ax3, nan, nan, nan, 'r-', 'LineWidth', 3);
cols = [0.85 0.33 0.10; 0 0.45 0.74; 0 0.45 0.74; 0.85 0.33 0.10];
for i = 1:4
    hArm(i) = plot3(ax3, nan, nan, nan, 'k-', 'LineWidth', 2.5);
    hProp(i) = patch(ax3, nan, nan, nan, cols(i,:), 'FaceAlpha', 0.35, 'EdgeColor', cols(i,:));
end
hTxt = text(ax3, 0.01, 0.99, '', 'Units', 'normalized', 'FontName', 'Consolas', 'FontSize', 8, 'VerticalAlignment', 'top', 'BackgroundColor', 'w');

axA = nexttile(tl, 2); hold(axA, 'on'); grid(axA, 'on'); ylabel(axA, 'angle [deg]');
plot(axA, t, eul(:,1), 'Color', [0.85 0.85 0.85]); plot(axA, t, eul(:,2), 'Color', [0.85 0.85 0.85]);
hR = plot(axA, nan, nan, 'LineWidth', 1.5); hP = plot(axA, nan, nan, 'LineWidth', 1.5);
legend(axA, [hR hP], {'roll', 'pitch'}, 'Location', 'northeast'); xlim(axA, t([1 end]));
axY = nexttile(tl, 4); hold(axY, 'on'); grid(axY, 'on'); ylabel(axY, 'yaw rate [deg/s]');
plot(axY, t, pqr(:,3), 'Color', [0.85 0.85 0.85]); hY = plot(axY, nan, nan, 'LineWidth', 1.5);
xlim(axY, t([1 end]));
axM = nexttile(tl, 6); hold(axM, 'on'); grid(axM, 'on'); ylabel(axM, 'motor [krpm]'); xlabel(axM, 't [s]');
hM = gobjects(4,1);
for i = 1:4, hM(i) = plot(axM, nan, nan); end
legend(axM, hM, {'M1 RR','M2 FR','M3 RL','M4 FL'}, 'Location', 'eastoutside'); xlim(axM, t([1 end]));
ylim(axM, [0 max(rpm(:))/1e3*1.05 + 0.1]);

if isfile(gif_file), delete(gif_file); end
for f = 1:numel(fr)
    j = find(t <= fr(f), 1, 'last');
    Rb = local_dcm(deg2rad(eul(j,:)));                   % body -> NED
    toUp = diag([1 -1 -1]);                              % NED -> x fwd, y left, z up for drawing
    c = toUp*[0; 0; Xe(j,3) + 1]*0;                      % keep the quad centred
    for i = 1:4
        p = toUp*Rb*rm(:,i);
        set(hArm(i), 'XData', [c(1) p(1)], 'YData', [c(2) p(2)], 'ZData', [c(3) p(3)]);
        disk = toUp*Rb*(rm(:,i) + R*[cos(ang); sin(ang); zeros(size(ang))]);
        set(hProp(i), 'XData', disk(1,:), 'YData', disk(2,:), 'ZData', disk(3,:), ...
            'FaceAlpha', 0.15 + 0.6*rpm(j,i)/max(rpm(:)));
    end
    n = toUp*Rb*[0.12; 0; 0];
    set(hNose, 'XData', [0 n(1)], 'YData', [0 n(2)], 'ZData', [0 n(3)]);
    set(hTxt, 'String', sprintf('t = %5.2f s\nroll  %6.1f deg\npitch %6.1f deg\nalt   %6.2f m\nsticks r%+.2f p%+.2f y%+.2f T%.2f', ...
        t(j), eul(j,1), eul(j,2), -Xe(j,3), sticks(j,:)));
    set(hR, 'XData', t(1:j), 'YData', eul(1:j,1)); set(hP, 'XData', t(1:j), 'YData', eul(1:j,2));
    set(hY, 'XData', t(1:j), 'YData', pqr(1:j,3));
    for i = 1:4, set(hM(i), 'XData', t(1:j), 'YData', rpm(1:j,i)/1e3); end
    drawnow;
    exportgraphics(fig, gif_file, 'Append', f > 1, 'Resolution', 70);
end
close(fig);
fprintf('wrote %s (%d frames)\n', gif_file, numel(fr));
end

function R = local_dcm(e)
% body -> NED from roll, pitch, yaw (ZYX)
cr = cos(e(1)); sr = sin(e(1)); cp = cos(e(2)); sp = sin(e(2)); cy = cos(e(3)); sy = sin(e(3));
R = [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr;
     sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr;
     -sp,   cp*sr,            cp*cr];
end
