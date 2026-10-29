# UAV Simulink Simulation Framework

[![CI](https://github.com/Qxy661/Simulink-UAV-Dynamics-Sim/actions/workflows/ci.yml/badge.svg)](https://github.com/Qxy661/Simulink-UAV-Dynamics-Sim/actions/workflows/ci.yml)
[![license](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2024a-orange.svg)](https://www.mathworks.com)

English | [中文](README.md)

Closed-loop simulation of a quadrotor, a fixed-wing aircraft and a tilt-rotor, implemented **twice** — once as a MATLAB package and once as Simulink charts generated from the same equations — and cross-checked against each other. The core loops can also be generated to C and run with nothing but a C compiler.

| Quadrotor position step | Fixed-wing climb / turn / accelerate | Tilt-rotor transition |
| --- | --- | --- |
| ![](figures/demo_step.png) | ![](figures/demo_fw.png) | ![](figures/demo_tilt.png) |

## Why it is worth a look

1. **Two implementations, independently written.** The Simulink charts are generated with every parameter inlined as a literal, so they share no code with the MATLAB package. They agree to 1e-15 … 1e-12. The one time they disagreed by 12 mm it was a real bug — the order of actuator update versus integration — documented in [docs/validation.md](docs/validation.md).
2. **Verify without MATLAB.** `codegen/` holds generated C, `c/` builds three runners with a Makefile, and CI compares them against committed reference trajectories, failing above 1e-9. Clone, `make`, `make run`: no license needed.
3. **The tilt-rotor conversion is characterised, not hand-waved.** The stable corridor ends at β = 57°, 10 m/s; from 12 m/s the rotor vertical component plus wing lift cannot hold altitude, and the failure data is published too — see [docs/tiltrotor.md](docs/tiltrotor.md).

## Status

| Item | Result |
| --- | --- |
| Unit and closed-loop tests | 49 passing |
| Quadrotor position step | 0.003 m final error, 30.5° peak tilt |
| Fixed-wing mission | climb 30 m, turn 60°, accelerate to 22 m/s; final 130.01 m, 60.0°, 22.02 m/s |
| Tilt-rotor transition | β 0→57.3°, airspeed 0→9.99 m/s, altitude error 0.33 m, final horizontal error 1.08 m |
| Simulink cross-check | 1.1e-15 (quad, 14 s), 1.3e-12 (fixed wing, 60 s), 6.65e-12 (tilt rotor, 45 s, all 21 states) |
| Generated C | 5.9e-14 / 9.1e-13 / 8.4e-12 against the MATLAB trajectories |
| Controller benchmark | PID 4.19 s / LQR 4.47 s / MPC 5.99 s to within 5 cm; 35 % thrust loss recovers to < 1 cm |
| Tutorials | 61 chapters in 11 parts, Chinese, in [tutorials/](tutorials/README.md) |

## Quick start

```
git clone https://github.com/Qxy661/Simulink-UAV-Dynamics-Sim.git
cd Simulink-UAV-Dynamics-Sim
```

```matlab
addpath(pwd);
uav.demo('step');          % quadrotor position step
uav.demo('hover');
uav.demo('yaw');
uav.demo('fw');            % fixed wing: climb, turn, accelerate
uav.demo('tilt');          % tilt rotor: hover, nacelle tilt, cruise
runtests('tests')          % 49 tests, about 40 s
```

Everything at once — tests, three model cross-checks, C runner cross-check:

```matlab
cd scripts
verify_all
```

Build and validate the Simulink models:

```matlab
cd scripts
build_quad_model;  validate_model
build_fw_model;    validate_fw_model
build_tilt_model;  validate_tilt_model
```

## Without MATLAB

```
cd c
make                       # mingw32-make on Windows
make run                   # writes out/quad.csv, out/fw.csv, out/tilt.csv
build/uav_quad_runner      # or run one configuration
```

`codegen/` contains the generated sources, `c/tmwtypes.h` is a small shim so that only a C compiler is required.

## Conventions

State vector, identical for quadrotor and fixed wing:

```
x = [pn; pe; pd; u; v; w; qw; qx; qy; qz; p; q; r]
```

- World frame NED, body frame FRD, quaternion maps body to NED.
- Euler angles in ZYX order; positive pitch is nose up.
- Motors 1-4: front-right, rear-right, rear-left, front-left; 1 and 3 counter-clockwise, `a = arm/sqrt(2)`.
- Mixer `B` maps squared rotor speeds to `[T; Mx; My; Mz]`; the controller uses `inv(B)`.
- Tilt rotor adds 8 states: nacelle angle, four rotor speeds, three surfaces (21 total). β = 0 thrust up, β = 90° thrust forward.

Full derivations: [docs/architecture.md](docs/architecture.md), [docs/fixedwing.md](docs/fixedwing.md), [docs/tiltrotor.md](docs/tiltrotor.md), [docs/codegen.md](docs/codegen.md), [docs/validation.md](docs/validation.md).

## Known limitations

- Linear aerodynamics only: no stall, unsteady effects, blade flapping, ground effect or wind field.
- Motors and servos are first-order lags with saturation; no current loop, backlash or deadband.
- The fixed-wing controller is tuned at 18 m/s; no gain scheduling at high angle of attack or low speed.
- The tilt-rotor conversion is validated only up to β = 57°; decelerating back to hover and the full 90° conversion are not solved yet.
- A fully stopped motor (η = 0) makes the vehicle spin on reaction-torque imbalance; only a 35 % thrust loss is demonstrated.

## License

MIT, see [LICENSE](LICENSE).
