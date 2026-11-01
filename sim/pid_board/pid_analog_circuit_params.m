%PID_ANALOG_CIRCUIT_PARAMS Extra parameters of the Simscape circuit variant.
%   Defines the struct pacirc used by pid_analog_circuit.slx (component-level
%   Simscape Electrical model of the PID_Board rev B analog section). Every
%   board value (resistors, caps, trimpots, jumpers, rails) is read from the
%   struct pcb (pid_board_params.m) directly inside the model, so a pot or
%   jumper change in pcb takes effect on the next simulation. pacirc only
%   holds part models (op-amp, diode, switch), solver options and initial
%   conditions that pcb does not describe.
%
%   Run order: run pid_board_params.m first (this file runs it if pcb is
%   missing from the caller workspace).

if ~exist('pcb','var')
    run(fullfile(fileparts(mfilename('fullpath')),'pid_board_params.m'));
end

pacirc = struct();

%% Solver (Simscape local solver, fixed step, Backward Euler)
% The Solver Configuration block has UseLocalSolver = on, Backward Euler
% (L-stable: damps the MHz op-amp loop poles), fixed cost. To let the
% parent's variable-step solver integrate the network instead:
%   set_param('pid_analog_circuit/Solver_Configuration','UseLocalSolver','off')
% Ts must be an integer multiple of the parent's fixed step: the board top model
% (Model 1) runs Fixed-step ode4 at 0.5 ms, so the default is 0.5 ms. Accuracy at
% 0.5 ms: < 0.5 dB / 3 deg up to 50 Hz, ~-1.5 dB / +10..25 deg at 200-300 Hz (D term);
% use 20e-6 (variable-step parents, or a parent fixed step <= 20 us) for full fidelity.
pacirc.Ts         = 0.5e-3;       % s, local solver step
pacirc.nl_iter    = 10;           % max Newton iterations per step (diodes, rail limits); fixed cost off

%% MCP6004 op-amp (ee_lib Band-Limited Op-Amp)
% Open loop A/(1 + s/(2*pi*f)), f = GBW/A, hard output limits Vmin/Vmax
% (absolute, referred to GND), slew-rate limit, output resistance Rout.
pacirc.opamp.A     = 10^(90/20);              % 90 dB typ (MCP6004 AOL, datasheet 88-112 dB)
pacirc.opamp.f     = pcb.opamp.GBW/pacirc.opamp.A;  % Hz, dominant pole -> GBW = 1 MHz
pacirc.opamp.Rin   = 1e12;                    % ohm, CMOS input (datasheet 1e13)
pacirc.opamp.Rout  = 50;                      % ohm, open-loop output resistance (assumed)
pacirc.opamp.Vmin  = pcb.VG + pcb.V_rail_lo;  % V abs, 0.025 V
pacirc.opamp.Vmax  = pcb.VG + pcb.V_rail_hi;  % V abs, 4.975 V
pacirc.opamp.SR    = 1e9;                     % V/s. Slew limit OFF: MCP6004 0.6 V/us (pcb.opamp.SR) swings
%                     rail to rail in 8 us, far below the solver step; enabling it with a 0.5 ms
%                     step makes the op-amp state diverge on large steps.
pacirc.opamp.V0    = pcb.VG;                  % V abs, initial internal output state

%% 1N4148 (ee_lib Diode, exponential model, SPICE: IS 2.52n, N 1.752, RS 0.568)
pacirc.diode.IS = 2.52e-9;        % A
pacirc.diode.N  = 1.752;          % emission coefficient
pacirc.diode.RS = 0.568;          % ohm
pacirc.diode.CJ = 0;              % pF, junction capacitance ignored (4 pF << 1 uF)

%% CD4066 U9 switches (Simscape Switch, PS control = I_HOLD volts)
pacirc.sw.R_on   = pcb.ihold.R_on;   % ohm, 470 typ at VDD = 5 V
pacirc.sw.G_off  = 1e-11;            % S, off leakage
pacirc.sw.Vth    = pcb.VDD/2;        % V, logic threshold on I_HOLD

%% Jumpers (shunt fitted / open)
pacirc.jp.R_on  = 1e-3;              % ohm
pacirc.jp.R_off = 1e10;              % ohm
pacirc.R_dnp    = 1e10;              % ohm, "DNP" summer resistor (en_* = 0)

%% Reference (sheet PID_Reference): TL431 + RV101 divider
pacirc.ref.R_tl431 = 0.2;            % ohm, TL431 dynamic impedance (datasheet |z_KA| 0.2)
% RV101 wiper-GND resistance. Default: trimmed so VSET = pcb.VG with the
% nominal TL431 (the schematic note: trim until VGND_A = DAC midscale).
% Use pcb.ref.RV_set_wiper_gnd (8.1k label) to see the untrimmed 2.021 V.
pacirc.ref.V_node  = pcb.ref.V_tl431 + pacirc.ref.R_tl431*(pcb.VDD - pcb.ref.V_tl431)/pcb.ref.R_bias; % V, TL431 cathode under 2.5 mA
pacirc.ref.RV_wg   = pcb.ref.RV_set*pcb.VG/pacirc.ref.V_node;   % 8.207k

%% Initial conditions (absolute volts): every node at rest at VG
pacirc.V0_dac  = pcb.VG;             % DAC RC caps C15-C22
pacirc.V0_rail = pcb.VG;             % VGND rail caps C101-C103
