# Elantra GT experimental longitudinal control

Branch `2026-custom-0.10.1-experimental-long` adds an opt-in longitudinal port for
the recorded 2018 Elantra GT Sport configuration. It retains the existing custom
UI, chimes, Cancel behavior, temperatures, and boost display.

**Factory AEB and factory forward collision warning are disabled while this
longitudinal configuration is running, including with cruise disengaged.**
Sunnypilot's own collision warning remains available; it does not replace AEB.
The car's 32 mph steering-assist cutoff still applies.

## Hardware scope

The option is exposed only for `HYUNDAI_ELANTRA_GT_I30` with the exact recorded
radar `96400-G3100` and ABS `58920-G3350` firmware responses and the required
stock SCC/FCA messages on bus 0. ESCC configurations are excluded. Unknown
firmware retains stock cruise and legacy safety.

Enabling the option selects the existing Hyundai longitudinal controller and
normal Hyundai safety checks. It does not remove acceleration limits, brake or
Cancel disengagement, checksum checks, or stock-SCC conflict detection. Startup
requires one second of live chassis traffic without radar SCC/FCA output. If
verification fails, initialization stops before sending replacement commands.

## Settings and recovery

With the vehicle parked, use Settings > Developer > **openpilot Longitudinal
Control (Alpha)**. Accept its AEB warning, then power-cycle the vehicle and let
sunnypilot start the next drive. Once longitudinal control is active, the existing
**Experimental Mode** toggle selects model-based longitudinal planning. This
branch does not set either option for you; existing saved values are retained.

Turning the Alpha setting off requires another startup to return to stock cruise;
it does not immediately restore the radar during the current drive. Power-cycle
the vehicle when reverting or recovering from an initialization failure, and
verify the factory systems have resumed normal operation. A full car-process
restart while the radar is already silent can fail the stock-message eligibility
check and require a vehicle restart.

## Validation limits

Offline tests cover hardware eligibility, message encoding, radar-disable
verification, restart handling, longitudinal safety limits, disengagement, radar
conflicts, and telemetry access. They do not establish the brake controller's
acceptance of replacement messages or the vehicle's acceleration, stopping,
holding, and restart behavior. This branch is a development port requiring
controlled vehicle validation before road use.

The supplied stock-ACC log keeps the second acceleration field and SCC14 mode
field at encoded zero even with cruise active. This differs from the generic
Hyundai encoder. This port retains that encoder and its safety limits; the
differences remain a vehicle-validation item.

Relevant tests are `opendbc_repo/opendbc/car/hyundai/tests/test_elantra_longitudinal.py`,
`opendbc_repo/opendbc/sunnypilot/car/hyundai/tests/test_elantra_radar_disable.py`,
and `opendbc_repo/opendbc/safety/tests/test_elantra_longitudinal.c`.
