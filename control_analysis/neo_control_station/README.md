NEO / SPARK MAX - Data prepared for process identification

SOURCE
comandos.csv, resumen.csv, status_0.csv, status_2.csv supplied by the user.
Run completed: 132 s; steps 0,5,...,100 percent, then 0 percent, 6 s per level.
Original STATUS_0 and STATUS_2 each contain 6600 received frames.
Measured STATUS_2 mean interval: 19.9995958583 ms (~50.0010 Hz).

FILES AND COLUMN MAPPING
Each dataset is provided as:
- .txt: ASCII, tab-separated, NO header, decimal point.
- .csv: comma-separated, ONE header line, decimal point.
The numeric data in each pair are identical.
Column 1 = Time, in seconds.
Column 2 = CO / MV / input: APPLIED duty in PERCENT (0 to 100).
Column 3 = PV / output: RPM in velocity files; relative motor revolutions
           in position_full. This is not a setpoint column.
No SP column is required for this open-loop identification experiment.

velocity_full: full run, 6599 rows, timestamps 0.00 to 131.96 s.
position_full: same input and grid, measured encoder position minus initial value.
velocity_step_45_to_50: source times 57.00 to 65.98 s. Local t=0 at source 57 s.
velocity_step_70_to_75: source times 87.00 to 95.98 s. Local t=0 at source 87 s.
Local step files contain a nonzero initial velocity and initial input.
Configure initial steady state/operating point accordingly when fitting them;
do not assume zero initial conditions for these windows.

IMPORT
Use the data import/model-fitting function of Control Station.
First try the TXT variant: tab separator, no header, Time=1, CO/MV=2, PV=3.
If using CSV: comma separator, skip one header line.
Time unit is SECONDS; nominal record interval is 0.020 seconds (50 Hz).
CO is percent. PV is RPM or revolutions, NOT percent of an arbitrary range.
If the application requires signal ranges, preserve these engineering units
and record any normalization it applies before exporting model/controller gains.
I could not verify the exact legacy v3.5 import dialog or its file-extension
requirements. These are plain numeric import files, not a native project format.
If rejected, the import dialog or its example file is needed to adapt syntax.

PREPROCESSING (NOT NEW MEASUREMENTS)
Original time origin: resumen.csv t0_unix_s = 1789785044.527487.
Common uniform grid in ORIGINAL t_rx_s: 0.02 to 131.98 s, step 0.020 s.
Full files subtract 0.02 s from BOTH input/output times to start at zero.
Applied duty: previous received STATUS_0 value (zero-order hold).
Velocity and position: linear interpolation between adjacent STATUS_2 samples.
No extrapolation, extra smoothing, outlier removal, or artificial high-rate data.
Tiny timing jitter is regularized; original raw files remain authoritative.
There are 6599 uniform rows rather than 6600 raw frames because the grid must
remain inside both records' observed time ranges.
STATUS_0/2 are separate messages, not guaranteed simultaneous sensor samples.
Their timestamps are host CAN reception times, not internal sensor sample times.
The first observed new duty bounds its timing only to approximately one 20 ms frame.
Do not interpret a fitted sub-sample delay as an accurately measured physical delay.
50 received frames/s does not guarantee 50 independent sensor updates/s.

IDENTIFICATION AND POSITION CONTROL
Use velocity data first to compare FOPDT and SOPDT fits. Do not force a second
pole if it is unresolved at 20 ms or fails validation on another transition.
Start with a local step window and validate on another; a global 0-100% fit can
hide local dynamics, friction, saturation or the different dynamics of stopping.
The final 100 -> 0% stop is in the full file; inspect it separately before using
it to fit a single model to the accelerating steps.

If G_n_percent(s) = N_RPM(s) / U_percent(s), then, for compatible measurement
paths and zero/deviation initial conditions:
  G_position_percent(s) = G_n_percent(s) / (60*s)       [rev / percent duty]
  G_position_fraction(s) = 100*G_n_percent(s) / (60*s)  [rev / duty fraction]
A velocity $SOPDT K*exp(-L*s)/((tau1*s+1)*(tau2*s+1))$ therefore becomes a
position model with an EXTRA INTEGRATOR, not an ordinary self-regulating SOPDT.
The rational part of the position plant has poles 0, -1/tau1, -1/tau2.
The delay remains and must be handled in the control design.
Because reported velocity can be filtered differently than position, validate
the derived model against measured position; do not blindly integrate velocity
measurement filtering into the position plant.
Position PID gains must be designed for the POSITION model. Velocity PID gains
are not directly transferable. Controller form, units, discretization, output
limits and SPARK implementation must be matched before entering gains.
An alternative is direct identification of an integrating position model in
MATLAB (e.g. idproc/procest family P1DI or P2DI, where I denotes an integrator).
See https://www.mathworks.com/help/ident/ref/idproc.html

The identification record period (20 ms) is distinct from the internal SPARK
control period. Digital pole mapping uses the actual controller period.
Motor position is not automatically joint position. An ideal gear ratio scales
angle, but attaching the gearbox/load changes the plant and requires validation.
