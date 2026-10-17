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
stock SCC/FCA messages on bus 0. ESCC configurations and layouts containing
FCA12 or FRT_RADAR11 on bus 0 are excluded. Unknown firmware retains stock cruise
and legacy safety.

Enabling the option selects the Hyundai longitudinal controller with the
recorded legacy FCA11 status-message format and normal Hyundai safety checks.
It does not remove acceleration limits, brake or
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
field at encoded zero even with cruise active. This differs from both the generic
Hyundai encoder and the Circuit-Pro radar-disable implementation. This port
retains the generic SCC12/SCC14 encoding and its acceleration limits; the
differences remain a vehicle-validation item. An older Hyundai DBC names the
acceleration fields maximum/minimum instead of raw/value. That is a clue to
possible older semantics, not proof of this firmware's interpretation. An
encoded-zero field must not be interpreted as an observed extreme braking demand.

## Radar-disable implementation comparison

Reviewed Circuit-Pro's implementation at immutable commit
[`e0c40ea0f8e8aa01124e23bc6685285d6ca49a89`](https://github.com/Circuit-Pro/openpilot/tree/e0c40ea0f8e8aa01124e23bc6685285d6ca49a89),
the `stable` branch named by its installation instructions. Its `main` branch
contains documentation rather than the implementation. The code includes the
Elantra GT 2018 DCT platform, but does not establish tested success with the exact
G3100 radar/G3350 ESC combination.

| Area | Comparison and decision |
| --- | --- |
| Diagnostic radar disable | Same extended session `10 03`, communication control `28 83 01`, address `0x7D0`, bus 0. Our startup additionally verifies silence while chassis messages remain live. |
| Session keepalive | Same suppressed-response tester-present `3E 80` once per second during operation. |
| SCC11/SCC12/SCC14 | Same 50 Hz schedule and basic acceleration/mode encoding. Retained pending vehicle-response evidence. |
| FCA11 | Corrected to the recorded legacy modulo-15 counter and nibble checksum. The fork's additional speculative counter pattern is not used. |
| Optional radar messages | FCA12 and FRT_RADAR11 are absent from both provided stock-message inventories and are omitted for this gated variant. SCC13 remains at 5 Hz with the recorded options. |
| Safety | Retained bounded Hyundai LONG safety. The fork's broad community safety implementation is not transplanted. |

Primary source files:
[initialization](https://github.com/Circuit-Pro/openpilot/blob/e0c40ea0f8e8aa01124e23bc6685285d6ca49a89/selfdrive/car/hyundai/interface.py#L551-L581),
[CAN generation](https://github.com/Circuit-Pro/openpilot/blob/e0c40ea0f8e8aa01124e23bc6685285d6ca49a89/selfdrive/car/hyundai/hyundaican.py#L127-L210),
[scheduling](https://github.com/Circuit-Pro/openpilot/blob/e0c40ea0f8e8aa01124e23bc6685285d6ca49a89/selfdrive/car/hyundai/carcontroller.py#L232-L267).

## Recorded-message correction and offline evidence

All 3,000 FCA11 frames in the supplied ACC segment match the legacy nibble
checksum, and none match the modern CRC8. Their byte-7 low nibble cycles through
0..14; the counter location used by the newer DBC remains zero. Preserved samples
from the earlier cruise-off report corroborate this format. Upstream's
[FCA11 checksum discussion](https://github.com/commaai/opendbc/pull/656#issuecomment-1266292167)
also acknowledges that older vehicles may use this layout.

The recorded FCA11 body is `00 00 48 00 00 00 00`. Replacement messages use
`00 00 44 00 00 00 00`, changing FCA_Status from enabled (2) to disabled (1),
then append the counter and nibble checksum. AEB actuation fields remain zero.
This correction applies only to the active, firmware-gated port; another Hyundai
regression test verifies unchanged modern CRC8 encoding.

The offline replay removes radar-origin SCC/FCA messages from the recorded input,
then runs the real CarInterface, CAN parsers, CarController, and native Panda
safety hooks. Each main-cruise-toggle configuration processed 147,165 input frames,
6,000 vehicle-state updates, and 18,360 generated messages. After startup there
were no invalid CAN states, timeouts, ACC faults, unhealthy safety checks, or
rejected receive/transmit frames. All 3,000 generated FCA11 messages passed
independent legacy checksum/counter checks and contained no AEB actuation.

This replay retains the recorded chassis ECU responses; it cannot predict their
response to actual radar disable. Its bounded command probe derives enable from
native safety permission, not the complete selfdrived state machine. Separate
regressions cover runtime Main/Set/Resume/Cancel handling, pedal states, fault
reporting, missing radar messages, command scheduling, and existing custom
notifications. No physical vehicle test or complete comma device build was run.

Private analysis remains under `build/elantra_message_variant_analysis.json` and
`build/elantra_long_candidate_replay.json`, with reproducing scripts beside them.
The scripts assert expected results and exit unsuccessfully on a mismatch.
Raw logs and private report artifacts are not included in the published source.

Relevant tests are `opendbc_repo/opendbc/car/hyundai/tests/test_elantra_longitudinal.py`,
`opendbc_repo/opendbc/sunnypilot/car/hyundai/tests/test_elantra_radar_disable.py`,
and `opendbc_repo/opendbc/safety/tests/test_elantra_longitudinal.c`.
