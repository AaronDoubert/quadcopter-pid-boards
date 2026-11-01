%PID_BOARD_PARAMS Parameters of the whole PID_Board rev B (digital twin).
%   Defines ONE struct, pcb, with every value a Simulink model of the board
%   needs: shim (ATtiny U1 + MPU-6050 + 5x MCP4822), reference and virtual
%   grounds, the five analog loops (roll angle -> roll rate, pitch angle ->
%   pitch rate, yaw rate) and control_io (ATtiny U3 mixer + ESC outputs).
%   Source of truth: PID_Board.kicad_pro rev B netlist (2026-10-31). Values
%   tagged "doc" come from the Analog PID Wiring Guide (2026-10-19), the
%   schematic notes or the breadboard sketch mpu6050_5ch_mcp4822_138.ino;
%   "assumed" values have no source yet (see pid_board_spec.md, open
%   questions). Every controller voltage is measured from VG unless the
%   comment says "absolute". Trimpots are rheostats (wiper tied to one end),
%   value = resistance in circuit. Derived gains are NOT stored: compute
%   them from the RV_* fields in the model so a pot change takes effect.
%   Re-running this file restores every default.

pcb = struct();

%% Supply and op-amps (from schematic: all op-amps MCP6004 on +5V/GND)
pcb.VDD       = 5.0;       % V, +5V plane from U10 buck module (absolute)
pcb.VG        = 2.048;     % V, virtual ground = MCP4822 midscale (absolute)
pcb.V_rail_lo = 0.025 - 2.048;  % V, MCP6004 VOL ~25 mV  -> -2.023 V re VG
pcb.V_rail_hi = 4.975 - 2.048;  % V, MCP6004 VOH ~4.975 V -> +2.927 V re VG
pcb.opamp.GBW    = 1e6;    % Hz, MCP6004 gain-bandwidth (doc: datasheet)
pcb.opamp.SR     = 0.6e6;  % V/s, slew rate
pcb.opamp.Vos    = 4.5e-3; % V, max input offset (worst case, per stage)

%% Reference and virtual grounds (sheet PID_Reference, U101 + U102)
pcb.ref.R_bias   = 1e3;    % R101, +5V -> TL431 cathode (I_KA ~2.5 mA)
pcb.ref.V_tl431  = 2.495;  % V, U102 TL431ACLPG shunt (REF=K), +-1 % (absolute)
pcb.ref.V_buf_meas = 2.53; % V, VREF_BUF measured on the breadboard (doc)
pcb.ref.RV_set   = 10e3;   % RV101 total, VREF_BUF -> GND divider
pcb.ref.RV_set_wiper_gnd = 8.1e3;  % RV101 wiper-GND (label); trim so VSET = 2.048 V
pcb.ref.R_iso    = 100;    % R102/R103/R104, follower -> VGND_A/B/C
pcb.ref.C_rail   = 470e-9; % C101/C102/C103 film, VGND rail caps (fc 3.4 kHz)
% Rail users (from netlist):
%   VGND_A: + inputs of U201B-D, U202A/C, U301B-D, U302A/C; U3 PA7 (ADC)
%   VGND_B: diff-amp ref legs R204 R218 R304 R318 R404; all U401 + inputs; U402A
%   VGND_C: bench plants U202D/U302D; U402B-D spare followers
pcb.ref.ideal    = 1;      % 1: VGND_A = VGND_B = VGND_C = VG exactly

%% DACs (root sheet, U4-U8 MCP4822, LDAC tied GND, gain 2x)
pcb.dac.bits   = 12;
pcb.dac.lsb    = 1e-3;     % V, 2.048 V ref x2 / 4096
pcb.dac.code0  = 2048;     % code for 0 V re VG
pcb.dac.lo     = -2.048;   % V re VG (code 0)
pcb.dac.hi     = 2.047;    % V re VG (code 4095)
pcb.dac.t_settle = 4.5e-6; % s, MCP4822 settling (negligible)
pcb.dac.R_f    = 1e3;      % R5-R14, series filter resistor on every output
pcb.dac.C_f    = 1e-6;     % C15-C24, 1uF to GND on every output
% Channel map (from netlist): chip (decoder output) VA / VB
%   U4 (Y0): ROLL_ANGLE      / ROLL_RATE
%   U5 (Y1): PITCH_ANGLE     / PITCH_RATE
%   U6 (Y2): YAW_RATE        / YAW_RATE_CMD
%   U7 (Y3): ROLL_ANGLE_CMD  / PITCH_ANGLE_CMD
%   U8 (Y4): THROTTLE        / DAC_SPARE (J1 pin 5 only)
pcb.dac.order = {'ROLL_ANGLE','ROLL_RATE','PITCH_ANGLE','PITCH_RATE', ...
    'YAW_RATE','YAW_RATE_CMD','ROLL_ANGLE_CMD','PITCH_ANGLE_CMD', ...
    'THROTTLE','DAC_SPARE'};
% Loaded RC time constants (the 1uF node also drives the diff-amp resistor)
pcb.dac.tau_unloaded = pcb.dac.R_f*pcb.dac.C_f;               % s, 1.00 ms (159 Hz): THROTTLE, DAC_SPARE
pcb.dac.tau_inv_leg  = pcb.dac.C_f/(1/pcb.dac.R_f + 1/10e3);  % s, 0.909 ms (175 Hz): gyro rate, angle cmd
pcb.dac.tau_ni_leg   = pcb.dac.C_f/(1/pcb.dac.R_f + 1/20e3);  % s, 0.952 ms (167 Hz): angle meas, yaw cmd

%% Signal scales (doc: wiring guide; firmware not written yet)
pcb.scale.Kw      = 6e-3;    % V per deg/s, gyro rate and rate commands
pcb.scale.Ktheta  = 25e-3;   % V per deg, fused angle and angle commands
pcb.scale.Kw_sketch = 2.048/250; % V per deg/s in mpu6050_5ch_mcp4822_138.ino (8.19 mV) - NOT the guide value
pcb.scale.thr_V0  = 0;       % V absolute on THROTTLE at 0 % (assumed)
pcb.scale.thr_V1  = 4.095;   % V absolute on THROTTLE at 100 % (assumed, full DAC code)

%% Shim firmware, ATtiny1614 U1 (root sheet) - no PCB firmware exists yet
pcb.shim.f_clk     = 20e6;   % Hz, CPU clock (assumed)
pcb.shim.Ts        = 1e-3;   % s, IMU read + 10 DAC writes per frame (assumed, briefing)
pcb.shim.Ts_sketch = 5e-3;   % s, breadboard sketch value (SAMPLE_MS 5, 200 Hz) - doc
pcb.shim.i2c_Hz    = 400e3;  % Hz, Wire.setClock (doc: sketch)
pcb.shim.spi_Hz    = 5e6;    % Hz, 8 MHz requested -> 5 MHz at 20 MHz clock (assumed)
pcb.shim.t_dac_skew = 100e-6; % s, first-to-last DAC update within a frame (assumed)
pcb.shim.gyro_fs   = 250;    % deg/s, MPU-6050 FS_SEL=0 (doc: sketch default)
pcb.shim.acc_fs    = 2;      % g, AFS_SEL=0 (doc: sketch default)
pcb.shim.t_dlpf    = 2.8e-3; % s, DLPF_CFG=2 (98 Hz) as a pure delay (assumed, briefing)
pcb.shim.t_dlpf_cfg0 = 0.98e-3; % s, delay if DLPF is left at reset (CFG=0, 256 Hz): sketch
pcb.shim.cf_alpha  = 0.98;   % complementary-filter gyro weight per frame (assumed)
pcb.shim.gyro_nd   = 0.005;  % deg/s/sqrt(Hz), MPU-6050 rate noise density
pcb.shim.angle_max = 60;     % deg, stick full scale -> +-1.5 V (assumed, guide FS)
pcb.shim.yaw_rate_max = 250; % deg/s, stick full scale -> +-1.5 V (assumed)
pcb.shim.crsf_baud = 420e3;  % baud, USART0 RX on PB3 via Q3 (from schematic note)
pcb.shim.crsf_Ts   = 4e-3;   % s, ELRS packet rate 250 Hz (assumed)
pcb.shim.crsf_min  = 172;    % CRSF channel value at 988 us
pcb.shim.crsf_mid  = 992;    % CRSF channel value at 1500 us
pcb.shim.crsf_max  = 1811;   % CRSF channel value at 2012 us
pcb.shim.failsafe_t = 0.25;  % s, link loss -> disarm (assumed)
% Pins (from netlist): PA1 MOSI, PA3 SCK, PA4-PA6 74HC138 A0-A2 (E3=+5V, always
% enabled), PA2 ARM, PA7 I_HOLD, PB0 SCL, PB1 SDA, PB3 CRSF, PB2 n.c.

%% Integrator hold, U9 CD4066 (sheet PID_Roll) driven by I_HOLD (U1 PA7)
pcb.ihold.R_on      = 470;   % ohm, switch on-resistance at VDD = 5 V (typ; max ~1.05k)
pcb.ihold.R_pd      = 10e3;  % R25 pulldown: I_HOLD low (integrators FREE) while U1 boots
pcb.arm.R_pd        = 10e3;  % R24 pulldown on ARM (U1 PA2 -> U3 PB3): disarmed at boot
pcb.ihold.t_release = 0;     % s, I_HOLD high (integrators shorted) until this time (sim input)
% Switches: A (pins 1/2) roll C201, B (3/4) pitch C301, C (8/9) yaw C401; D spare (ctrl=GND)

%% Roll axis (sheet PID_Roll: U201, U202) - trimpots (ohms), the values you tune
pcb.roll.RV_P = 50e3;     % RV201, 100k: Kp = (4.7k + RV_P)/47k          (1.16 nominal)
pcb.roll.RV_I = 67e3;     % RV202, 100k: Ki = 1/((33k + RV_I)*1 uF)      (10 /s nominal)
pcb.roll.RV_D = 50e3;     % RV203, 100k: Kd = (47k + RV_D)*100 nF       (9.7 ms nominal)
pcb.roll.RV_A = 25e3;     % RV204,  50k: Go = (10k + RV_A)/20k          (1.75 nominal)
pcb.roll.RV_P_max = 100e3;
pcb.roll.RV_I_max = 100e3;
pcb.roll.RV_D_max = 100e3;
pcb.roll.RV_A_max = 50e3;

% Term enables (1 = summer input resistor fitted)
pcb.roll.en_P = 1;        % R211
pcb.roll.en_I = 1;        % R212
pcb.roll.en_D = 1;        % R213

% Jumpers: 1 = pins 1-2 (flight), 2 = pins 2-3 (bench / external)
pcb.roll.JP_SP = 1;       % JP201 LOOP_SP: 1 ROLL_RATE_SP (angle loop), 2 EXT_SP (TP209)
pcb.roll.JP_FB = 1;       % JP202 LOOP_FB: 1 ROLL_RATE (gyro DAC), 2 EXT_FB (TP210)

% Stage 1, U201A rate diff amp: E_RR = g_sp*SP - g_fb*FB
pcb.roll.R_dfb_in  = 10e3; % R201, FB_SEL -> U201A-
pcb.roll.R_dfb_f   = 10e3; % R202, E_RR -> U201A-
pcb.roll.R_dsp_in  = 10e3; % R203, SP_SEL -> U201A+
pcb.roll.R_dsp_ref = 10e3; % R204, U201A+ -> VGND_B
% Stage 2, U201B P (inverting): P_OUT = -Kp*E_RR
pcb.roll.R_P_in    = 47e3;  % R205
pcb.roll.R_P_fixed = 4.7e3; % R206, in series with RV201
% Stage 3, U201C leaky I (inverting) + clamp + reset: I_OUT
pcb.roll.R_I_fixed = 33e3;  % R207, in series with RV202
pcb.roll.CI        = 1e-6;  % C201 film
pcb.roll.RL        = 1e6;   % R208 leak -> wL = 1 rad/s
pcb.roll.n_clamp   = 2;     % D201+D202 / D203+D204: two 1N4148 in series per direction
pcb.roll.V_iclamp  = 0.75;  % V, clamp level (0.6-1.0 V with current; guide 0.6-0.9)
% Stage 4, U201D band-limited D (inverting): D_OUT
pcb.roll.Rs        = 27e3;   % R209, series with Cd -> f1 = 59 Hz
pcb.roll.Cd        = 100e-9; % C202 film
pcb.roll.Rd_fixed  = 47e3;   % R210, in series with RV203
pcb.roll.Cf        = 10e-9;  % C203 C0G, across Rd -> f2 = 108-339 Hz
% Stage 5, U202A summer: U_ROLL = -(P + I + D)
pcb.roll.R_sum     = 10e3;   % R211 (P), R212 (I), R213 (D), R214 (feedback)
% Stage 6, U202B angle diff amp, inputs swapped: E_A = g_ang*(ANGLE - ANGLE_CMD)
pcb.roll.R_acmd_in  = 10e3;  % R215, ROLL_ANGLE_CMD -> U202B-
pcb.roll.R_a_f      = 10e3;  % R216, E_A -> U202B-
pcb.roll.R_ameas_in = 10e3;  % R217, ROLL_ANGLE -> U202B+
pcb.roll.R_a_ref    = 10e3;  % R218, U202B+ -> VGND_B
% Stage 7, U202C angle P (inverting): ROLL_RATE_SP = -Go*E_A
pcb.roll.R_A_in    = 20e3;   % R219
pcb.roll.R_A_fixed = 10e3;   % R220, in series with RV204
% Stage 8, U202D bench plant: PLANT_OUT = -(1/(R221*C204 s))*U_ROLL, no leak, no reset
pcb.roll.R_plant   = 22e3;   % R221
pcb.roll.C_plant   = 1e-6;   % C204 film -> tau_p = 22 ms
pcb.roll.plant_ref = 'VGND_C';
% Source impedances seen by the diff-amp legs
pcb.roll.R_src_sp  = 0;            % ROLL_RATE_SP is an op-amp output (U202C)
pcb.roll.R_src_fb  = pcb.dac.R_f;  % ROLL_RATE via R6 1k / C16
pcb.roll.R_src_ang = pcb.dac.R_f;  % ROLL_ANGLE (R5/C15), ROLL_ANGLE_CMD (R11/C21)
% Leg gains (DC), derived from the fixed resistors above
pcb.roll.g_sp  = pcb.roll.R_dsp_ref/(pcb.roll.R_dsp_in + pcb.roll.R_src_sp + pcb.roll.R_dsp_ref) ...
    *(1 + pcb.roll.R_dfb_f/(pcb.roll.R_dfb_in + pcb.roll.R_src_fb));          % 0.955
pcb.roll.g_fb  = pcb.roll.R_dfb_f/(pcb.roll.R_dfb_in + pcb.roll.R_src_fb);  % 0.909
pcb.roll.g_ang_cmd  = pcb.roll.R_a_f/(pcb.roll.R_acmd_in + pcb.roll.R_src_ang);  % 0.909
pcb.roll.g_ang_meas = pcb.roll.R_a_ref/(pcb.roll.R_ameas_in + pcb.roll.R_src_ang + pcb.roll.R_a_ref) ...
    *(1 + pcb.roll.R_a_f/(pcb.roll.R_acmd_in + pcb.roll.R_src_ang));          % 0.909
% Bench mode (JP_SP = JP_FB = 2): PLANT_OUT wired to EXT_SP (0 ohm source),
% generator into EXT_FB (no 1k): E = g_sp*PLANT_OUT - 1.0*V_gen
pcb.roll.g_fb_ext = pcb.roll.R_dfb_f/pcb.roll.R_dfb_in;                      % 1.000
pcb.roll.ref = struct('U', {{'U201','U202'}}, 'RV_P','RV201', 'RV_I','RV202', ...
    'RV_D','RV203', 'RV_A','RV204', 'JP_SP','JP201', 'JP_FB','JP202', ...
    'TP', {{'TP201 E_RR','TP202 P_OUT','TP203 I_OUT','TP204 D_OUT','TP205 U_ROLL', ...
            'TP206 E_A','TP207 ROLL_RATE_SP','TP208 PLANT_OUT','TP209 EXT_SP','TP210 EXT_FB'}});

%% Pitch axis (sheet PID_Pitch: U301, U302) - pin-for-pin copy of roll, refdes +100
pcb.pitch = pcb.roll;
pcb.pitch.RV_P = 50e3;    % RV301, 100k: Kp = (4.7k + RV_P)/47k          (1.16 nominal)
pcb.pitch.RV_I = 67e3;    % RV302, 100k: Ki = 1/((33k + RV_I)*1 uF)      (10 /s nominal)
pcb.pitch.RV_D = 50e3;    % RV303, 100k: Kd = (47k + RV_D)*100 nF       (9.7 ms nominal)
pcb.pitch.RV_A = 25e3;    % RV304,  50k: Go = (10k + RV_A)/20k          (1.75 nominal)
pcb.pitch.JP_SP = 1;      % JP301
pcb.pitch.JP_FB = 1;      % JP302
% Fixed parts R301-R321, C301-C304, D301-D304 have the roll values; sources
% PITCH_RATE (R8/C18), PITCH_ANGLE (R7/C17), PITCH_ANGLE_CMD (R12/C22)
pcb.pitch.ref = struct('U', {{'U301','U302'}}, 'RV_P','RV301', 'RV_I','RV302', ...
    'RV_D','RV303', 'RV_A','RV304', 'JP_SP','JP301', 'JP_FB','JP302', ...
    'TP', {{'TP301 E_PR','TP302 P_OUT','TP303 I_OUT','TP304 D_OUT','TP305 U_PITCH', ...
            'TP306 E_A','TP307 PITCH_RATE_SP','TP308 PLANT_OUT','TP309 EXT_SP','TP310 EXT_FB'}});

%% Yaw axis (sheet PID_Yaw: U401, U402A) - rate loop only, all refs on VGND_B
pcb.yaw.RV_P = 50e3;      % RV401, 100k: Kp = (4.7k + RV_P)/47k          (1.16 nominal)
pcb.yaw.RV_I = 50e3;      % RV402, 100k "set mid": Ki = 1/((15k + RV_I)*1 uF) (15.4 /s)
pcb.yaw.RV_D = 50e3;      % RV403, 100k: Kd = (47k + RV_D)*100 nF       (9.7 ms, not summed)
pcb.yaw.RV_P_max = 100e3;
pcb.yaw.RV_I_max = 100e3;
pcb.yaw.RV_D_max = 100e3;

pcb.yaw.en_P = 1;         % R411
pcb.yaw.en_I = 1;         % R412
pcb.yaw.en_D = 0;         % R413 10k is DNP: D_OUT (TP404) is built but not summed

pcb.yaw.JP_SP = 1;        % JP401 LOOP_SP: 1 YAW_RATE_CMD (DAC), 2 EXT_SP (TP406)
pcb.yaw.JP_FB = 1;        % JP402 LOOP_FB: 1 YAW_RATE (DAC), 2 EXT_FB (TP407)

pcb.yaw.R_dfb_in  = 10e3; % R401
pcb.yaw.R_dfb_f   = 10e3; % R402
pcb.yaw.R_dsp_in  = 10e3; % R403
pcb.yaw.R_dsp_ref = 10e3; % R404 -> VGND_B
pcb.yaw.R_P_in    = 47e3;  % R405
pcb.yaw.R_P_fixed = 4.7e3; % R406
pcb.yaw.R_I_fixed = 15e3;  % R407
pcb.yaw.CI        = 1e-6;  % C401 film
pcb.yaw.RL        = 1e6;   % R408
pcb.yaw.n_clamp   = 2;     % D401-D404
pcb.yaw.V_iclamp  = 0.75;  % V
pcb.yaw.Rs        = 27e3;   % R409
pcb.yaw.Cd        = 100e-9; % C402 film
pcb.yaw.Rd_fixed  = 47e3;   % R410
pcb.yaw.Cf        = 10e-9;  % C403 C0G
pcb.yaw.R_sum     = 10e3;   % R411, R412, (R413 DNP), R414
pcb.yaw.R_src_sp  = pcb.dac.R_f;  % YAW_RATE_CMD via R10 1k / C20 (a DAC, not an op-amp)
pcb.yaw.R_src_fb  = pcb.dac.R_f;  % YAW_RATE via R9 1k / C19
pcb.yaw.g_sp = pcb.yaw.R_dsp_ref/(pcb.yaw.R_dsp_in + pcb.yaw.R_src_sp + pcb.yaw.R_dsp_ref) ...
    *(1 + pcb.yaw.R_dfb_f/(pcb.yaw.R_dfb_in + pcb.yaw.R_src_fb));             % 0.909
pcb.yaw.g_fb = pcb.yaw.R_dfb_f/(pcb.yaw.R_dfb_in + pcb.yaw.R_src_fb);       % 0.909
pcb.yaw.g_fb_ext = pcb.yaw.R_dfb_f/pcb.yaw.R_dfb_in;                         % 1.000
pcb.yaw.ref = struct('U', {{'U401','U402'}}, 'RV_P','RV401', 'RV_I','RV402', ...
    'RV_D','RV403', 'JP_SP','JP401', 'JP_FB','JP402', ...
    'TP', {{'TP401 E_YR','TP402 P_OUT','TP403 I_OUT','TP404 D_OUT','TP405 U_YAW', ...
            'TP406 EXT_SP','TP407 EXT_FB'}});

%% Control I/O, ATtiny1614 U3 (sheet Control_IO) - no firmware exists yet
% ADC inputs (from netlist, no divider, no filter): PA1 U_ROLL, PA2 U_PITCH,
% PA3 U_YAW, PA6 THROTTLE (1k/1uF from DAC), PA7 VGND_A. Firmware subtracts VGND_A.
pcb.ctrl.adc_bits = 10;
pcb.ctrl.adc_Vref = pcb.VDD;   % V, VDD reference (assumed; 4.3 V ref would clip U > +2.25 V)
pcb.ctrl.adc_lsb  = pcb.VDD/1024;  % V, 4.88 mV
pcb.ctrl.Ts       = 2.5e-3;    % s, ADC + mix + ESC frame (assumed, 400 Hz PWM)
pcb.ctrl.t_compute = 0.2e-3;   % s, 5 conversions + mixer (assumed)
% Quad-X mixer, Betaflight motor order (assumed): columns [thr roll pitch yaw]
%   M1 rear-right (PB0), M2 front-right (PB1), M3 rear-left (PB2), M4 front-left (PA4)
pcb.ctrl.mix = [1 -1  1 -1;    % M1 RR
                1 -1 -1  1;    % M2 FR
                1  1  1  1;    % M3 RL
                1  1 -1 -1];   % M4 FL
pcb.ctrl.K_rpy  = [200 200 200]; % us per V of U_ROLL/U_PITCH/U_YAW (assumed placeholder)
pcb.ctrl.us_off    = 1000;     % us, disarmed output (schematic note: hold 1000 us)
pcb.ctrl.us_idle   = 1050;     % us, armed minimum (note: run 1050-2000 us)
pcb.ctrl.us_max    = 2000;     % us, full throttle
pcb.ctrl.us_res    = 1;        % us, pulse resolution (assumed; 8-bit TCA split mode would be ~10 us)
% ESC outputs: U3 pin -> 1k series (R15/R17/R19/R21) -> J5 3-6, 2.2k to GND (R16/R18/R20/R22)
pcb.ctrl.R_esc_series = 1e3;
pcb.ctrl.R_esc_shunt  = 2.2e3;
pcb.ctrl.V_esc_high   = pcb.VDD*2.2e3/(1e3 + 2.2e3);   % V, 3.44 V logic high at the ESC

%% Power (sheet Control_IO)
pcb.pwr.VBAT_range = [7.4 16.8]; % V, 2S-4S via J5 pin 2 (not measured by any ADC)
pcb.pwr.Vf_D1      = 0.35;       % V, D1 1N5817 at ~0.2 A (doc: datasheet typ)
pcb.pwr.C_in       = 47e-6;      % C25, 25 V
pcb.pwr.Vin_min    = 5.5;        % V, U10 mini buck minimum input
