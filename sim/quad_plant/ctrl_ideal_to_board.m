function B = ctrl_ideal_to_board(ideal, pcb, Kmix)
%CTRL_IDEAL_TO_BOARD Map ideal-controller gains onto PID-board trimpot settings.
%   B = ctrl_ideal_to_board(ideal, pcb)        Kmix = 200 us/V (pid_board_spec placeholder)
%   B = ctrl_ideal_to_board(ideal, pcb, Kmix)
%   pcb = struct from sim/pid_board/pid_board_params.m. Prints a table and
%   flags values outside the pots' ranges. Mapping (see ctrl_ideal_params.m):
%     Kp_b = Kp/(Kmix*Kw*g_fb),  Ki_b = Ki/(Kmix*Kw*g_fb),  Kd_b = Kd/(Kmix*Kw*g_fb)
%       Kw = pcb.scale.Kw*180/pi  [V per rad/s]
%     Go   = Ka/(g_ang*(Ktheta/Kw_deg)*(g_sp/g_fb))
%     RV_P = Kp_b*R_P_in - R_P_fixed           Kp_b = (4.7k + RV_P)/47k
%     RV_I = 1/(Ki_b*CI) - R_I_fixed           Ki_b = 1/((R_I_fixed + RV_I)*CI)
%     RV_D = Kd_b/Cd - Rd_fixed                Kd_b = (Rd_fixed + RV_D)*Cd
%     RV_A = Go*R_A_in - R_A_fixed             Go   = (10k + RV_A)/20k
%   Note the board yaw cell uses R407 = 15k and does not sum D.

if nargin < 3, Kmix = 200; end
Kw = pcb.scale.Kw*180/pi;                       % V per rad/s
axs = {'roll','pitch','yaw'};
fprintf('\nideal -> board (Kmix = %g us/V, Kw = %g V/(deg/s))\n', Kmix, pcb.scale.Kw);
fprintf('%-6s %7s %7s %8s %6s | %8s %8s %8s %8s\n', 'axis', 'Kp_b', 'Ki_b', 'Kd_b ms', 'Go', 'RV_P', 'RV_I', 'RV_D', 'RV_A');
for a = 1:3
    ax = axs{a}; g = ideal.(ax); c = pcb.(ax);
    if strcmp(ax, 'yaw')
        RIf = 15e3; c = pcb.roll; c.RV_I_max = pcb.yaw.RV_I_max;   % yaw cell: R407 15k, same other parts
    else
        RIf = c.R_I_fixed;
    end
    gk = Kmix*Kw*c.g_fb;                       % gyro leg of the diff amp (1k + 10k) has gain 0.909
    b.Kp = g.Kp/gk; b.Ki = g.Ki/gk; b.Kd = g.Kd/gk;
    b.RV_P = b.Kp*c.R_P_in - c.R_P_fixed;
    b.RV_I = 1/(b.Ki*c.CI) - RIf;
    b.RV_D = b.Kd/c.Cd - c.Rd_fixed;
    if strcmp(ax, 'yaw') && g.Kd == 0, b.RV_D = NaN; end   % D not used on yaw
    b.Go = NaN; b.RV_A = NaN;
    if a < 3
        b.Go = g.Ka/(c.g_ang_meas*(pcb.scale.Ktheta/pcb.scale.Kw)*(c.g_sp/c.g_fb));
        b.RV_A = b.Go*c.R_A_in - c.R_A_fixed;
    end
    rng = {'RV_P', c.RV_P_max; 'RV_I', c.RV_I_max; 'RV_D', c.RV_D_max};
    if a < 3, rng(end+1,:) = {'RV_A', c.RV_A_max}; end
    b.out_of_range = {};
    for k = 1:size(rng, 1)
        v = b.(rng{k,1});
        if ~isnan(v) && (v < 0 || v > rng{k,2}), b.out_of_range{end+1} = rng{k,1}; end
    end
    if strcmp(ax, 'yaw') && g.Kd ~= 0, b.out_of_range{end+1} = 'yaw D not summed on board'; end
    B.(ax) = b;
    fprintf('%-6s %7.3f %7.2f %8.2f %6.2f | %7.1fk %7.1fk %7.1fk %7.1fk  %s\n', ax, b.Kp, b.Ki, ...
        1e3*b.Kd, b.Go, b.RV_P/1e3, b.RV_I/1e3, b.RV_D/1e3, b.RV_A/1e3, strjoin(b.out_of_range, ' '));
end
end
