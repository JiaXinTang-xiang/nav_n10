# CHANGELOG.md

This file is the **cross-domain engineering communication log** for `ros2-stm32-autonomous-robot`.

It is intentionally **English-only**.

Record only changes that the other Codex domain must know about, shared-interface changes, safety/architecture changes, or project milestones. Do not record trivial local implementation details.

## 2026-09-13T14:19 — Native DWB thin-wall navigation accepted

Domain: ROS
Impact: The thin-wall / multi-turn navigation blocker is closed. Firmware action NONE.
Protocol 1.0, CAN, safety gate, planner, BT, footprint and inflation are unchanged.

Changed only FollowPath.forward_prune_distance from Humble's 2.0 m default to 0.4 m
in dwb_core::DWBLocalPlanner. The old recorded path retained the correct wall detour,
but its local scoring endpoint lay across the wall and DWB favored stationary
turning. The wall remained present in 158/158 inspected local costmaps; published
footprint matched the frozen polygon and padding. Shortening the local plan keeps
its scoring endpoint on the current approach to the corner.

Status: PASS. Operator confirmed multiple full-map and thin-wall runs all succeeded
without the previous hesitation/heading oscillation. Runtime parameter readback
confirmed 0.4 m. The bounded bag independently confirms six explicit NavigateToPose
goal UUIDs all SUCCEEDED, no Failed to make progress, and final GOAL_REACHED with
authority withdrawn. Evidence: /data/projects/robot_navigation/
thin_wall_20260913_HHzOMH/navigation_test/. English/Chinese operation docs updated.
No rebuild or additional run was needed. Separate obstacle/remove/continue and
integrated safety acceptance remain outside this result; peer action is not required.

## 2026-09-10T19:55 — Native navigation startup and RViz defaults prepared

Domain: ROS
Impact: Explicit RViz goals no longer fail a custom cross-machine timestamp check;
host STOP/fault persistence supports the requested boot service. Firmware action NONE.
Protocol 1.0, drivetrain safety, odometry/localization/TF ownership are unchanged.

The earlier FollowPath preemption bug caused 35 unnecessary withdrawals at a median
1.030 s interval and 0.308 s pause. The boundary now keeps authority while any
replacement FollowPath is executing; action status never refreshes the command
deadline. One 4.569 s isolated check passed. Operator confirmed the real H-corner /
wall-adjacent turn passed, and subsequent native runs logged four goal successes.
Long multi-turn runs also logged native SimpleProgressChecker failures and Wait
recoveries; the operator reported pushing the robot. These are not evidence of
unassisted multi-turn acceptance. Root cause needs a stopped-scene capture.

Removed the host-only goal timestamp age/ordering rejection, matching native Nav2
map-goal ingress. It had rejected valid VM goals with only 77–85 ms clock lead.
Finite planar map-goal validation, volatile single-goal ingress, fresh controller
commands, scan/TF/bridge health and actual fault/STOP latches remain. Goal dispatch
uses the host timestamp. The boot service persists host STOP/faults in
/data/projects/robot_navigation/stop_latch, cleared only by healthy manual reset;
it never stores or replays goals and has no automatic fault-restart loop.

Added robot-navigation.service after existing bridge/localization services, waiting
for valid map -> base_link before Nav2 activation. Added the missing RViz defaults
in robot_navigation/config/navigation.rviz and a normal-user VM installer for
default.rviz, a launcher and desktop-login autostart. Displays use /map, /scan,
/odom, /plan and the native footprint; tools use /initialpose and /goal_pose.

Only robot_navigation was rebuilt. One 5.158 s isolated boundary check passed
remote-clock goal ingress, preemption continuity, existing health/command/STOP
latches and STOP persistence across process restart. YAML/Python/shell checks passed;
systemd verification encountered unrelated vendor-unit access warnings. The local,
runtime-source and installed navigation_v1.yaml hashes remain identical:
06da87f5b5d80308386883afc44dc3315d8ec989e52818f742f5b47af12e7698.
No Nav2 numeric parameters were tuned. Existing project-specific geometry/limits
remain; this does not claim every value is an upstream default.

Status: IN PROGRESS. Package installed. The operator corrected a pasted filename
line break; robot-navigation.service is now loaded, enabled and active. All four
managed Nav2 nodes report active [3]; the motion gate is idle awaiting an explicit
goal. This verifies service activation, not a subsequent boot cycle. The VM installer and
launcher succeeded according to operator output. The operator confirmed visible
map, live scan and robot pose after initial localization, with scan/wall alignment;
desktop-login autostart remains unobserved. Bundle: /data/projects/robot-rviz-setup.tar.gz.
Variable-position startup retains one explicit initial localization pose.
Remaining real work: capture the multi-turn blocker without pushing, and finish
native obstacle/remove/continue with the original goal retained through SUCCESS.
Evidence: /data/logs/robot_navigation/native_startup_* and native_preemption_*.

## 2026-09-10T13:29 — Native Nav2 obstacle ownership; physical acceptance pending

Domain: ROS
Impact: User superseded the custom obstacle-stop requirement with native Nav2 ownership.
Firmware action: NONE. Protocol 1.0 and accepted odometry/localization/TF are unchanged.

Removed navigation clearance thresholds, NORMAL_OBSTACLE_STOP, ObstacleWait BT plugin,
private obstacle wait/resume services and their DemoStatus handshake fields. The
standalone straight obstacle-stop commissioning demo retains its accepted implementation.
Production navigation uses a separate motion boundary with standard Nav2 planning,
FollowPath and Wait recovery (5 s, six retries); no reverse/spin recovery is enabled.

The boundary accepts one explicit map goal, validates health and forwards fresh Twist
only during an explicitly executing FollowPath action. Child action termination zeroes
output and withdraws authority without canceling NavigateToPose; a new action needs
confirmed disarm and fresh commands to reauthorize. Missing communication is never
classified as waiting. Active command freshness remains 250 ms; scan/TF/bridge,
navigation feedback, authority, true faults and operator STOP remain supervised/latched.
The gate status topic now uses std_msgs/String; detailed health remains BridgeStatus.
Bridge heartbeat and firmware safety supervision continue while disarmed.

Only robot_navigation and robot_stm32_bridge were built on Orange Pi. One isolated
3.845 s mock boundary check passed same-goal retention, fresh-command reauthorization,
success/disarm, true-fault and active-command-timeout latches and operator STOP.
Initial startup exposed Humble's default NavigateThroughPoses BT requiring disabled
spin recovery. Its default_nav_through_poses_bt_xml now uses native Wait-only recovery
as well. No controller, costmap, inflation, speed, map or localization tuning was made.

Status: IN PROGRESS. One real H-corner/wall-adjacent goal and one obstacle/remove/
continue goal remain required. Earlier custom-stack physical success is historical
evidence only and does not establish native-stack PASS. Evidence directory:
/data/logs/robot_navigation/ (native_* build, install, check and runtime logs).

Final review retained all-invalid LaserScan as a latched sensor fault without a
sector/distance rule. The corresponding 4.124 s isolated check passed after fixing
a test-only race between action cancellation and the 100 ms status publication.
Final gate source SHA256: c1efc8b5ad82d3feab48c5c5a4987cd60156367c40a4ae3da7f1cac1e5ad7106.
Runtime: native_nav_runtime_final_20260910.log. No physical goal has yet been sent
for this native-only acceptance; no physical PASS is claimed.

## 2026-09-10T12:41 — Same-goal normal obstacle wait and automatic arrival verified

Domain: ROS
Impact: The Humble blocked-path compatibility fix passed the real drivetrain run.
The full 0.10 m LiDAR-trigger/wall-turn milestone remains PARTIAL; firmware action NONE.

One explicit goal (0.232935, -1.312477), yaw -1.424828, retained UUID
32424382b1334ced3b1665ab31f3a890 throughout EXECUTING -> SUCCEEDED, with no cancellation
or second goal. Costmap-confirmed blockage caused NORMAL_OBSTACLE_STOP, zero output
and authority withdrawal. The stopped interval lasted about 183.72 s while the
operator held the obstacle. No nonzero /cmd_vel was recorded during the confirmed
wait. On removal, a fresh plan and fresh /cmd_vel_nav preceded reauthorization.
Original-goal success followed about 18.62 s after resume handshake; final errors
were 0.1043 m position and 0.1249 rad yaw. STM32 fault_flags stayed zero; final
GOAL_REACHED was disarmed. Operator confirmed automatic physical arrival.

Limits: this stop was caused by path blockage, not the 0.10 m front trigger.
Minimum recorded projected front clearance across the run was 0.3219 m; at stop it
was about 0.3577 m (nearest front return, not identified as the placed obstacle).
Operator explicitly confirmed no wall-adjacent turn was exercised. Do not claim
the complete requested demo PASS. Next controlled goal only needs to close the
0.10 m LiDAR trigger and wall-adjacent turn evidence gaps.

Evidence: /data/logs/robot_navigation/normal_stop_physical_20260910_123232/,
including acceptance_partial.json, and normal_stop_runtime_retry_20260910.log.
Recording finalized; no safety, speed, firmware, wire protocol or map changes.

## 2026-09-10T12:26 — Humble blocked-path wait compatibility fixed; physical retry pending

Domain: ROS
Impact: The first real full-map obstacle run exposed a BT compatibility defect.
The fix preserves explicit obstacle waiting without changing true fault latches.
Firmware and Protocol 1.0 action: NONE.

Observed on 2026-09-09: goal (0.259980, -1.627830), yaw -1.397693, started near
(-3.35, -0.99). Replanning failed after about 10.34 s with the test obstacle present.
The previous BT incorrectly required nonempty invalid_pose_indices; actual Humble
1.1.20 returns is_valid=false with an EMPTY array for a known occupied path.
That check aborted NavigateToPose and correctly triggered the existing fault latch.
No 0.10 m LiDAR stop occurred; minimum recorded projected front clearance was
0.2343 m. Firmware fault_flags stayed zero. After the operator removed the obstacle
without moving the robot, a planning-only request to the original goal succeeded
with 109 poses. Map-size changes were observed but not established as the cause;
no map/localization configuration was changed.

Changed: ObstacleWait now uses the received is_valid=false response for a nonempty
current-goal path; the mock service now matches Humble's empty array. Missing service,
response timeout, valid-path navigation failure and gate/system faults still fail
safely and latch. Targeted CMake build passed. The isolated real BT/mock-actuator
check passed wait/replan/resume with the same NavigateToPose UUID, success and disarm
using the newly built library. An initial check loaded the old installed library;
the corrected run selected the new build explicitly.

Deployment: after the operator reboot, installed the verified new library (SHA256
f1689a02a0398cdad1650bc25897dbe0de539118d2c0e97a3f3aea56df400246).
No repeated build or physical test was run. Bridge startup before STM32 supply
produced ENOBUFS and inhibited reconnect. Once STM32 was powered, CAN RX resumed,
but the bridge required an explicit service restart. Approved sudo restart could
not execute without the operator's local password; the operator completed it.
Bridge 0.5.5 communication is restored, fault_flags=0, authority_armed=false,
zero fresh-session transport/protocol errors. New explicit localization trajectory 1
is active. Updated Nav2 managed nodes are active after a one-time lifecycle-response
startup retry; startup STOP was explicitly reset only after health recovery, without
motion. Hidden action topics are recording in normal_stop_physical_20260910_123232.
Waiting for the operator's new navigation goal. Status remains PARTIAL.

Evidence: /data/logs/robot_navigation/normal_stop_physical_20260909_203318/,
normal_stop_runtime_20260909.log, humble_wait_fix_build_20260909.log and
humble_wait_fix_check_built_20260909.log. The first bag lacks hidden action topics;
use --include-hidden-topics for the physical retry's goal-UUID evidence.
Next: restore bridge health, initialize localization, launch the updated Nav2 gate,
then one new explicit goal for physical stop/resume/arrival and wall-turn acceptance.

## 2026-09-09T20:29 — Approved normal LiDAR stop retains navigation goal

Domain: ROS
Impact: The operator approved sufficient verified stopping margin at the present
navigation speed for front 30-degree / <=0.10 m body clearance. This revision
supersedes the preceding navigation emergency-latch interpretation. Physical demo
acceptance remains IN PROGRESS; firmware and Protocol 1.0 require no action.

Changed:
- Goal-driven LiDAR stopping now enters NORMAL_OBSTACLE_STOP, immediately publishes
  zero and requests disarm, retaining the SAME NavigateToPose action. Existing BT
  FollowPath pause/replan/resume and DemoStatus.normal_obstacle_wait are reused.
- Removed the unconditional 0.40 m forward wait and 0.60 m raw resume rule that can
  block wall-adjacent turning. The configured body-clearance condition must remain
  clear for 1 s, with healthy scan/TF/bridge/system, confirmed disarm, fresh plan
  and a new controller command before reauthorization. Nav2 path blockage remains
  a separate explicit wait cause; missing communication never creates a wait.
- True fault and operator STOP latches, bridge heartbeat, STM32 safety, speed caps,
  watchdog/timeouts, standalone obstacle latch and shared wire protocol are unchanged.

Validation: targeted gate CMake build on Orange Pi passed. One 9.922 s isolated
mock-actuator check passed same-goal obstacle resume, rejection of old commands,
removal of the old 0.40 m trigger, real STM32 fault during an obstacle stop remaining
locked after health returns, command loss, scan loss and operator STOP.
Build/check logs: /data/logs/robot_navigation/normal_stop_{build,gate_check}_20260909.log.
Production Nav2 was restarted with startup_locked:=true to preserve the observed
operator STOP. Managed nodes are active. No real goal or physical motion was issued
in this stage. Physical obstacle distance, retained-goal resume, wall turn and final
arrival remain unmeasured pending the operator-assisted run.
English/Chinese ROS and RViz documentation updated. Firmware action: NONE.

## 2026-09-09T10:44 — Explicit normal navigation wait separated from emergency latch

Domain: ROS
Impact: Operator-approved normal obstacle waiting now retains the SAME NavigateToPose
action while zeroing velocity and withdrawing Motion Authority. Physical acceptance
is IN PROGRESS; firmware and frozen Protocol 1.0 require no changes.

Changed:
- The existing gate enters NORMAL_OBSTACLE_WAIT on a forward obstacle at 0.40 m
  projected front clearance, or on an explicit BT request after costmap-confirmed
  invalid path points. Command silence is permitted only inside this explicit wait;
  scan, TF, bridge/firmware and navigation feedback health remain supervised.
- A plugin inside the existing bt_navigator halts FollowPath, not NavigateToPose.
  Clearance >0.60 m raw front range for 1 s, confirmed disarm and a new valid plan
  precede its resume handshake. Old commands are discarded; only fresh controller
  commands and healthy state allow rearm. No path means remaining stopped.
- Per the operator's latest explicit boundary, the independent 0.10 m emergency
  trigger locks, cancels/discards the goal and requires manual reset/new goal.
  This supersedes the prior navigation-mode auto-resume at that emergency threshold.
  Actual faults and operator STOP never auto-clear. Standalone latch, speed caps,
  STM32 safety, watchdog and timeout values are unchanged.
- DemoStatus adds three explicit navigation-state booleans; the gate adds private
  wait_for_obstacle and resume_navigation Trigger services. No runtime node, TF
  publisher or additional cmd_vel authority is introduced. Rebuild ROS consumers
  of DemoStatus when deploying this message update.

Validation: targeted Orange Pi builds passed. One 9.341 s isolated gate check passed
normal wait/command silence, same-goal fresh-command resume, emergency latch, command
loss outside wait, scan loss inside wait and STOP. Real bt_navigator + gate with mock
planner/controller also passed invalid-path waiting, failed recovery plans, same goal
UUID through resume, SUCCEEDED and disarm. No physical movement for these checks.
Logs: /data/logs/robot_navigation/wait_bt_test_20260909.log.
Firmware action: NONE. ROS next boundary: one controlled real obstacle/clearance goal.

## 2026-09-08T19:33 — Real Nav2 point-to-point goal accepted on firmware 0.5.5

Domain: ROS
Impact: The previous COMMAND_TIMEOUT integration blocker is cleared for the bounded
real navigation demo. No firmware, protocol, TF, speed or safety behavior changed.

After the operator restarted the bridge to recover its inhibited SocketCAN connection,
live status confirmed 0.5.5, healthy transport and no fault. Existing map 0 localization
was initialized explicitly in RViz. Existing Nav2 ran with the independent safety gate.
The operator selected (-2.761498, -2.089801) m, yaw 1.719410 rad from approximately
(-2.40, -3.79) m. Path planning, autonomous physical motion and Goal succeeded completed
in 10.62 s. At success, gate error was 0.0950 m and 0.1586 rad. No COMMAND_TIMEOUT or
health stop occurred; bridge error/reject counters remained zero. Final state was
GOAL_REACHED, zero wheel-odometry twist and authority_armed=false.

The operator's subsequent return goal (-2.401510, -3.597371) m, yaw -1.393068 rad
also succeeded in 13.02 s, with 0.0798 m position and 0.1397 rad yaw error.
Post-return state remained disarmed/fault-free with zero bridge error/reject counters.
This completes the demonstrated outbound/return point-to-point milestone; bilingual
project overviews now describe the accepted demo and its validation limits.

Cartographer remains the map -> odom authority, the bridge owns odom -> base_link,
and existing sensor static TF is unchanged. Only navigation_safety_gate publishes
/cmd_vel to the bridge. LiDAR protection remained active; this goal did not deliberately
exercise obstacle stopping/resumption or establish physical braking clearance.
Explicit close initial pose remains required; errors are localization estimates.
Evidence: /data/logs/robot_navigation/2026-09-08-19-14-03-014946-robot-core-5435/.
ROS source/configuration changes: NONE this stage; English/Chinese usage docs updated.
Action required: Firmware NONE. No further motion/tuning is required for this goal stage.

## 2026-09-08T18:48 — Firmware 0.5.5 flashed; CAN health blocks deployment acceptance

Domain: Firmware
Impact: Supplied Debug HEX programmed and verified using existing J-Link SWD
(STM32G474RE, 4000 kHz, probe 602710753), then reset/run. Deployment is PARTIAL.

VERIFIED by target SWD readback: deployed version string 0.5.5, initialization complete,
READY, fault_flags=0, motor authorization=false, STBY LOW, both PWM compares zero.
STBY LOW/PWM zero also confirmed before flashing. No actuation or safety/option-byte
changes. HEX SHA256: 9A7EC1FC58AAC2C70381FA7EB64F26DF18A7C639A7CF468F0D2DF83149C7BE91.

CAN3 was ERROR-PASSIVE before and after flashing (TX error counter 128, RX 0);
no system-status frames captured. Deployed STM32 reports CAN bus-off, no session,
and no armed authority. COM3 was silent before and after; SWD supplied identity/state.
Physical communication cause remains unresolved; no system configuration was changed.

Action: keep Nav2 motion on hold. Resolve existing CAN/UART connectivity and pass
the bounded idle CAN check before one ROS short navigation recheck. Protocol 1.0,
timeout values, watchdog, authority, Supervisor and fault-latch behavior are unchanged.

## 2026-09-07T21:26 — Firmware 0.5.5 candidate fixes stale-time command expiry

Domain: Firmware
Impact: A reproduced firmware defect explains how fresh accepted commands can raise
COMMAND_TIMEOUT. Hardware deployment/recheck remains IN PROGRESS.

CommunicationTask captured time before RX processing, but command submission stamped a
later HAL tick. Unsigned age subtraction could underflow and trip the existing 250 ms
predicate. CAN health now samples time after RX draining; command snapshots sample time
inside their existing IRQ lock. Version is 0.5.5 for deployment identification.
Protocol 1.0, 250/500 ms thresholds, fault latches, explicit recovery and motor logic
are unchanged. No ROS workaround or shared-interface change is required.

Validation: deterministic production command/safety C test failed before and passed after
the fix, covering fresh commands with stale caller time, 250/251 ms expiry, wraparound,
safe output request and no automatic fault recovery. Executed with actuator/clock stubs
on the Orange Pi CPU, not on STM32. Debug build passed (FLASH 105552 B, RAM 42848 B).
The supplied raw CAN fault window was read and confirms accepted sequence 789 and fault
0x00000002; it does not record the internal instruction/tick ordering.

Action: deploy the candidate safely and confirm version 0.5.5 before one ROS short-distance
navigation recheck. No flashing, CAN injection, ROS runtime changes or physical motion
was performed. English/Chinese firmware documentation records evidence and limitations.

## 2026-09-07T21:01 — Navigation blocked by contradictory STM32 command-timeout telemetry

Domain: ROS observation; firmware investigation required
Impact: One real Nav2 goal reached successfully, but two subsequent physical goals
were stopped by the existing safety path when STM32 reported `COMMAND_TIMEOUT`.
The second failure has complete CAN evidence that contradicts Protocol 1.0 freshness
semantics, so ROS must not mask it by relaxing health gates or automatically resetting.

Evidence:
- Successful goal: Nav2 reported `Goal succeeded`; gate measured 0.0990 m position
  error and 0.1553 rad yaw error, then sent zero and withdrew authority.
- Failing goal at host timestamp 1788785844.424: `0x081` BODY_VELOCITY command
  sequence 789, `0x080` ARMED sequence 55259 and `0x082` heartbeat sequence 55253
  were transmitted. At 1788785844.454 (about 30 ms later), `0x180` timestamp
  5551387 reported state FAULT, `fault_flags=0x00000002` (`COMMAND_TIMEOUT`) and
  `last_command_sequence=789`, proving the latest command had been accepted.
- Across the bounded capture, maximum observed gaps were 30.011 ms for `0x081`,
  102.616 ms for `0x080`, and 102.607 ms for `0x082`, all inside the frozen 250 ms
  command and 500 ms authority/heartbeat deadlines. CAN3 remained ERROR-ACTIVE
  with zero error counters; bridge TX failures and protocol rejects remained zero.
- Scan, bridge-status receipt and localization TF were fresh at the stop. The ROS
  gate canceled Nav2, sent zero/disarm and remained manually latched as designed.
  Raw evidence is `/data/logs/robot_navigation/can_command_timeout_retry2_20260907.log`.

Action required:
- Firmware domain: inspect why firmware 0.5.4 raises `COMMAND_TIMEOUT` despite its
  own `last_command_sequence` confirming a command accepted about 30 ms earlier and
  concurrently fresh authority/heartbeat traffic. Determine the actual timeout
  predicate/source before proposing any firmware change.
- ROS: no timeout relaxation or workaround. Nav2 was shut down and STM32 authority
  confirmed false. Boot localization remains enabled/active and safe for VM viewing.

## 2026-09-07T19:43 — Front-bumper clearance threshold and navigation speed cap

Domain: ROS
Impact: At the operator's explicit request, production navigation uses 0.10 m
front-bumper clearance, not 0.10 m raw LiDAR range, and a 0.20 m/s linear speed cap.

Changed:
- In the existing 30-degree front sector, project each valid return onto vehicle
  forward and subtract 0.097 m (front x=0.140 minus laser x=0.043). At/below 0.10 m
  clearance, issue zero and withdraw authority through the existing obstacle path.
  Straight-ahead trigger range is 0.197 m. Default standalone raw-range mode remains
  unchanged; DemoStatus.minimum_front_range_m retains its raw-range meaning.
- Nav2 maximum linear speed and gate linear clamp are both 0.20 m/s. Resume still
  requires raw range >0.60 m for 1,000 ms and the existing health/cancellation checks.
- STOP/fault locks and STM32 safety semantics remain unchanged. Maintenance launches
  can preserve a fault lock with startup_locked:=true. Health-stop logs now include
  scan/bridge/TF ages to identify the earlier unresolved transient health failure.

Validation: Orange Pi gate build passed. One 7.371 s isolated mock check passed:
0.200 m straight return (.103 m clearance) did not stop; the same return at 14 degrees
off-axis (.097 m clearance) stopped/canceled. A .30 m/s mock command was capped at
.20 m/s. Pause/resume, success/disarm and manual-reset-only fault/STOP paths passed.
Installed production YAML was read back successfully. These are not physical braking
measurements. This trigger does not guarantee 10 cm residual clearance after stopping.

Runtime: Orange Pi had rebooted; only the bridge was running among the checked
navigation/sensor components. It reported authority_armed=false, arm_requested=false
and fault_flags=0. No real goal, arm request or motor actuation was performed. Real
navigation and the new physical obstacle threshold remain IN PROGRESS.

Action required: Firmware domain NONE; no protocol, firmware or TF changes. ROS must
restore the existing runtime with operator pose initialization before real validation.

## 2026-09-06T16:57 — Operator-approved ROS goal startup and obstacle pause/resume

Domain: ROS
Impact: The operator explicitly approved replacing the navigation demo's separate START
with an explicit RViz goal and allowing obstacle-only pauses to resume. This is a ROS
navigation-mode change; the standalone latched stop demo remains the default. AGENTS.md,
Protocol 1.0, STM32 watchdog/fault/authority semantics and firmware remain unchanged.

Changed:
- The existing navigation safety gate owns `/goal_pose` and invokes the existing Nav2 action.
  No additional runtime node or TF publisher is introduced. Goal acceptance alone never
  bypasses fresh scan, localization, controller-command or healthy bridge requirements.
- At/below 0.40 m in the existing front sector, zero and disarm precede cancellation.
  The destination is retained only for obstacle pauses; clearance above 0.60 m for 1,000 ms,
  healthy runtime and confirmed disarm/cancellation allow replanning and fresh authorization.
- Operator STOP, invalid/stale sensors/TF/commands, bridge faults and unexpected navigation
  failure discard the goal and remain locked. Restored data/new goals cannot unlock them.
  Explicit `~/reset` requires restored health and does not move; a new goal is then required.
- Goal success withdraws authority and waits for a new goal. Restart never restores a goal.
- Nav2's action acknowledgement budget is 1,000 ms after an observed 20 ms timeout aborted
  a valid path; the independent 250 ms command-freshness limit is unchanged.

Validation: gate compiled on Orange Pi; one isolated mock-action/actuator check passed in
7.034 s, covering automatic start, pause/resume hysteresis, success and non-resuming fault/STOP
latches. Production nodes are active without motion. Real goal/obstacle-resume acceptance
remains IN PROGRESS; clearance settings are commissioning values, not stopping-distance proof.

Action required:
- Firmware domain: NONE. No firmware work or interface extension is requested.
- ROS: finish the bounded real goal and obstacle-only pause/resume acceptance with an operator.

## 2026-09-04T23:42 — Real Cartographer localization against map 0 accepted

Domain: ROS
Impact: The navigation dependency advances from the accepted map to verified real localization.
Protocol 1.0, firmware, drivetrain calibration, and safety behavior are unchanged; the firmware
domain does not need to act.

Changed:
- The production Cartographer pure-localization path loads
  `/data/projects/maps/cartographer_lidar_slam_0.pbstream` and starts from an explicit map-frame
  pose converted to Cartographer's frozen-trajectory-relative convention.
- Ambiguous full-map fallback matching is disabled for the repeated H geometry. Real DISARMED
  manual movement retained continuous scan/map alignment with local constraint scores of roughly
  65-80%, correct observed left-turn direction, no branch jump or gross divergence, and a stable
  pose after stopping.
- `/cartographer_node` is the sole `map -> odom` authority; `/robot_stm32_bridge` remains the sole
  `/odom` publisher and `odom -> base_link` authority. The existing static sensor TF is unchanged.

Action required:
- Firmware domain: none. The next ROS stage is Nav2 point-to-point real-hardware navigation using
  map 0, this localization path, and the existing independent safety architecture.

## 2026-09-04T23:07 — Production H-venue map 0 accepted

Domain: ROS
Impact: The production localization input advances from the noisy v2 map to the clearer map 0.
Protocol 1.0, firmware, drivetrain calibration, TF ownership, and safety behavior are unchanged;
the firmware domain does not need to act.

Changed:
- A real DISARMED manual traversal covered the complete H venue and produced 858 trajectory
  nodes in 25 Cartographer submaps without observed gross duplication or divergence.
- The accepted state and 0.05 m/cell occupancy map are now
  `/data/projects/maps/cartographer_lidar_slam_0.{pbstream,pgm,yaml}`. The grid is 152 x 150 cells;
  the 3.4 MiB pbstream was read successfully to generate the PGM/YAML artifacts.
- The large sunlight-induced fan visible in v2 is absent. A small amount of sparse endpoint noise
  remains, and v2 is retained only as a rollback map.

Action required:
- Firmware domain: none. ROS localization should use map 0 and preserve Cartographer as the sole
  `map -> odom` authority and `/robot_stm32_bridge` as the sole `odom -> base_link` authority.

## 2026-09-04T10:02 — Real wheel-odometry-assisted LiDAR map accepted

Domain: ROS
Impact: The ROS navigation dependency advances from accepted wheel odometry to a saved real
Cartographer map. Protocol 1.0, firmware, drivetrain calibration, and safety behavior are
unchanged; the firmware domain does not need to act.

Changed:
- The existing Cartographer 2D path now consumes the production `/odom` together with the real
  RPLIDAR A1 `/scan`. Cartographer is the sole `map -> odom` authority while
  `/robot_stm32_bridge` remains the sole `odom -> base_link` authority.
- Real DISARMED manual mapping covered the complete H-shaped fixed venue. The primary wall
  structure was coherent without immediate divergence or gross duplication.
- The accepted finished state and 0.05 m/cell occupancy map were saved as
  `/data/projects/maps/cartographer_lidar_slam_v2.{pbstream,pgm,yaml}`. Re-reading the pbstream
  with Cartographer regenerated a valid 266 x 301 PGM/YAML map.
- A direct-sunlight segment produced a visible fan-shaped sparse/noisy artifact outside one end
  of the H; this is a known map limitation for the localization stage.

Action required:
- Firmware domain: none. The next ROS stage is localization against the saved map.

## 2026-09-04T08:23 — Real ROS wheel odometry accepted

Domain: ROS
Impact: The ROS navigation dependency advances from accepted `0x181` telemetry to verified
`/odom` and a single `odom -> base_link` TF authority. Protocol 1.0 and firmware are unchanged;
the firmware domain does not need to act.

Changed:
- The production `robot_stm32_bridge` now integrates only valid cumulative logical left/right
  wheel positions from `0x181`, preserving the independent 0.0001362305 and 0.0001363976 m/count
  scales and the 0.125 m track width.
- Invalid position samples and STM32 timestamp/reset discontinuities re-baseline without a false
  pose jump. Timestamp/sequence wrap is supported, and sequence gaps retain distance through the
  cumulative positions.
- The bridge publishes `nav_msgs/msg/Odometry` on `/odom` and is the sole dynamic
  `odom -> base_link` TF authority in the production runtime.
- Real DISARMED manual-motion validation passed: an approximately 1.9 m forward push produced
  approximately 1.89 m positive odometry; a left turn produced approximately +1.28 rad yaw.
  CAN3 remained Error Active with zero TX/RX error counters and no protocol rejection.

Action required:
- Firmware domain: none. The next ROS stage is real LiDAR mapping using the accepted wheel
  odometry; effective wheel/track calibration remains a later navigation-driven refinement.

## Entry format

```text
## YYYY-MM-DDTHH:MM — Title

Domain: Firmware | ROS | Interface | System
Impact: <what the other domain must know>

Changed:
- ...

Action required:
- None
```

---

## 2026-08-31T22:38:53+08:00 — Closed-loop drivetrain demo accepted and public repository identity updated

Domain: System
Impact: The closed-loop drivetrain and obstacle-stop integration is complete on real hardware. Protocol 1.0 and all firmware/ROS runtime identifiers are unchanged; the public GitHub repository identity is now `LingZhen07/ros2-stm32-autonomous-robot`.

Changed:
- Firmware `0.5.4`, BODY_COMMAND_READY, Motion Authority, closed-loop wheel control, and the production CAN3 physical link passed the final integrated demo.
- The accepted behavior is explicit START, 0.30 m/s straight closed-loop motion, RPLIDAR frontal obstacle detection in a 30° sector at 0.60 m, zero velocity plus authority withdrawal, STM32 safe stop, and latched STOPPED until a new explicit START.
- Observed obstacle-detection intervals were approximately 0.075 ms to the zero velocity command, 1.276 ms to Motion Authority withdrawal, and 30.8 ms to STM32 stop confirmation. These are measured demo observations, not guaranteed worst-case safety limits.
- The public repository name, bilingual documentation, and MIT licensing were updated without changing the frozen Protocol 1.0 contract or internal package/module names.

Action required:
- None

## 2026-08-31T09:02:05+08:00 — Orange Pi CAN3 physical link accepted and productionized

Domain: System
Impact: Protocol 1.0 is unchanged. The ROS production transport is now frozen on Orange Pi CAN3 / SocketCAN `can3`; CAN2 is historical and must not be used as a fallback.

Changed:
- The installed and boot-verified path is `822d0000.mttcan`, `mttcan-id=3`, GPIO2_17/CAN_TX3, and GPIO2_18/CAN_RX3.
- Real bidirectional CAN FD+BRS Protocol 1.0 traffic passed: host `0x080`/`0x082` and STM32 `0x180`/`0x181`/`0x182`/`0x183`, with the robot DISARMED.
- Linux production timing is explicitly fixed at 500 kbit/s with 80.0% sample point and 2 Mbit/s with 82.5% sample point. This corrected the earlier Linux default sample-point mismatch.
- Accepted health evidence is Error Active, TX error 0, RX error 0, bus-errors 0, and bus-off 0.
- `robot_stm32_bridge` now defaults to `can3`. Version-controlled systemd units configure CAN3 idempotently before starting the DISARMED bridge; unarmed startup does not emit `0x081`.

Action required:
- Deploy and enable the version-controlled `robot-can3.service` and `robot-stm32-bridge.service` on the Orange Pi after building the updated package.
- The next safety-gated milestone is the controlled straight-drive and latched LiDAR obstacle-stop demo.

## 2026-08-29T17:53:32+08:00 — BODY_VELOCITY drivetrain handoff ready in firmware 0.5.4

Domain: Firmware
Impact: Protocol 1.0 is unchanged. The ROS bridge may now treat STM32 BODY_COMMAND_READY as true when the existing session, communication, runtime-health, and safety gates are also valid; physical straight-drive acceptance is still required before the `/scan` demo.

Changed:
- Verified production mapping is Motor A -> right wheel and Motor B -> left wheel, with logical-forward motor sign -1 on both channels. Encoder 1/right/+1 raw and Encoder 2/left/-1 raw normalization is unchanged.
- Firmware now initializes the independent left/right wheel PI controllers and complete drivetrain configuration. BODY_COMMAND_READY requires valid mapping, geometry/scales, finite limits, and both configured controllers.
- The commissioning envelope is body linear 0.30 m/s, body angular 1.50 rad/s, wheel rate 3000 count/s, wheel-target slew 4400 count/s^2, and normalized motor output +/-0.60. Initial per-side gains are Kp 0.00020, Ki 0.00060, Kd 0, with a 700 count-second integrator limit.
- Authority withdrawal, disarm, timeout, bus-off, invalid command, fault, and reset continue to bypass the command ramp and converge through the existing motor-safe path. A new session/authority and fresh command are required after stop.
- The 0.30 m/s value is a commissioning limit, not a verified hardware maximum. Installed-motor loaded/stall current, attainable loaded wheel speed, and TB6612 carrier/module thermal margin remain unmeasured.

Action required:
- ROS domain: no serializer or Protocol 1.0 change. Keep the first straight-drive acceptance at 0.05 m/s, require BODY_COMMAND_READY plus all existing gates, and do not raise speed until real closed-loop tracking and authority-withdrawal stopping are accepted.
- Cross-domain acceptance: after the controlled straight-drive run passes, connect that accepted path to the existing explicit-START, latched-STOPPED `/scan` obstacle-stop flow.

## 2026-08-29T11:42:02+08:00 — Encoder validity semantics corrected in firmware 0.5.3

Domain: Firmware
Impact: Protocol 1.0 is unchanged. System-status fault bit 4 now reports encoder acquisition validity only, so the ROS domain must not interpret incomplete drivetrain/controller commissioning as an encoder failure.

Changed:
- `ENCODER_VALIDITY` now clears automatically only after both encoder timers initialized successfully, both latest samples are valid, and both local sample ages are at most 50 ms; real initialization/stale-sample failures still assert it.
- Unknown Motor A/B mapping or polarity, unset controller gains/speed limits, and `BODY_COMMAND_READY=false` do not assert encoder fault bit 4.
- A DISABLED, unarmed command snapshot remains intentionally non-timed-out. Existing timeout protection for an established motion/control session is unchanged.
- The local USART2 console was reduced to a quiet essential commissioning command set with optional fixed 1 Hz watch output; this does not alter CAN IDs, layouts, rates, scaling, session, sequence, authority, or timeout contracts.

Action required:
- ROS domain: no serializer or bridge change; consume Protocol 1.0 as before and treat bit 4 strictly as STM32 encoder acquisition health.
- Hardware acceptance: after flashing firmware 0.5.3, confirm `encoder` reports both sides valid and `fault` keeps bit 4 clear.

## 2026-08-29T00:39:38+08:00 — Physical wheel encoder mapping and independent scales verified

Domain: System
Impact: Firmware 0.5.2 now normalizes verified physical wheel encoders and uses independent directly measured output-wheel scales. Protocol 1.0 wire layouts and behavior are unchanged; ROS odometry must preserve the independent commissioning scales.

Changed:
- Ten complete forward wheel revolutions measured Encoder 1 at +10,595 counts and Encoder 2 at -10,608 counts.
- Encoder 1 is the right wheel with forward raw sign +1 and 1059.5 decoded counts/wheel-rev; Encoder 2 is the left wheel with forward raw sign -1 and 1060.8 decoded counts/wheel-rev.
- Logical wheel convention remains forward-positive: right logical count/rate is `+right raw`, left logical count/rate is `-left raw`. Raw Encoder 1/2 UART diagnostics and frozen Protocol `0x181` raw diagnostic fields remain unchanged.
- With commissioning radius 0.023 m, the derived right scale is approximately 0.0001363976 m/count and 0.005930331 rad/count; the left scale is approximately 0.0001362305 m/count and 0.005923063 rad/count. The approximately 0.123% difference is not averaged.
- BODY_COMMAND_READY remains false pending measured Motor A/B wheel ownership/polarity, accepted operating limits, and commissioned left/right controller gains.

Action required:
- ROS domain: interpret Protocol `0x181` left/right wheel fields as logical forward-positive values and retain independent right/left commissioning scales for odometry; do not average them or treat them as final effective calibration.
- Firmware commissioning: identify Motor A/B ownership and forward polarity with separately gated low-effort tests before selecting operating limits or tuning controllers.

## 2026-08-28T16:12:51+08:00 — Commissioning wheel geometry measured

Domain: System
Impact: Firmware 0.5.1 now uses measured commissioning wheel geometry for future BODY_VELOCITY conversion. The values are not final effective calibration and Protocol 1.0 is unchanged.

Changed:
- User-measured wheel radius is 0.023 m and wheel track is 0.125 m; derived half track is 0.0625 m and geometric circumference is approximately 0.1445 m.
- Differential-drive commissioning conversion is `v_left = v - omega * 0.0625` and `v_right = v + omega * 0.0625`.
- The verified encoder convention remains left-forward raw CPS negative, right-forward raw CPS positive, and logical forward positive on both wheels.
- BODY_COMMAND_READY remains false because decoded encoder scale, mappings, motor polarity, operating limits, and controller gains are not yet verified/configured.

Action required:
- Firmware commissioning: directly measure decoded encoder counts over multiple complete wheel revolutions and divide by the revolution count. Do not infer encoder scale from product specifications.
- ROS domain: treat radius/track as `MEASURED / COMMISSIONING`, not final odometry calibration.

## 2026-08-28T15:50:39+08:00 — Physical CAN wiring confirmed

Domain: System
Impact: The Orange Pi/STM32 CAN physical wiring topology is now confirmed, but real CAN FD traffic and controller error state are not yet accepted.

Changed:
- CANH is connected to CANH, CANL to CANL, and both nodes share a common ground.
- One 120 ohm termination is installed at each physical end of the bus.
- Protocol 1.0 and all CAN nominal/data timing, IDs, layouts, rates, and safety semantics remain unchanged.

Action required:
- Cross-domain integration: observe real bidirectional FD+BRS frames and Error Active state before declaring the physical CAN link PASS.
- Firmware commissioning: retain BODY_VELOCITY gating until measured drivetrain mapping, geometry, encoder scale, operating limits, and controller gains are configured.

## 2026-08-24T20:45:25+08:00 — STM32 body-velocity control firmware ready for cross-domain acceptance

Domain: Firmware
Impact: The production STM32 firmware is ready for physical Orange Pi/STM32 CAN FD integration against unchanged Protocol 1.0. Closed-loop BODY_VELOCITY execution remains deliberately gated by real drivetrain measurements and controller commissioning.

Changed:
- Firmware identity is 0.5.0; Protocol 1.0 CAN IDs, layouts, rates, scaling, session, sequence, authority, and timeout semantics are unchanged.
- CAN BODY_VELOCITY commands are validated against controller configuration, drivetrain readiness, and local operating limits before entering the shared command model; invalid active commands converge through the existing Safety Supervisor.
- Permanent USART2 diagnostics now expose FDCAN state, counters, session/authority/freshness ages, safety/fault state, and bounded commissioning controls. The console is quiet by default and does not change the shared CAN interface.
- Debug and Release clean builds pass with zero compiler/linker warnings. Physical CAN traffic and the straight-drive obstacle-stop demo have not yet been accepted.

Action required:
- ROS domain: proceed with physical Protocol 1.0 integration. Do not expect STM32 ACTIVE/BODY_VELOCITY until wheel/motor mapping, motor polarity, wheel radius, track, encoder scale, gear ratio, operating limits, and wheel-controller gains are measured/configured.

## 2026-08-24T13:38:04+08:00 — Communication Protocol v1 frozen and STM32 CAN FD transport complete

Domain: Interface
Impact: STM32 peripheral, sensor, motor, and safety bring-up passed physical acceptance, and the stable CAN FD boundary is ready for the Orange Pi ROS bridge implementation. The ROS domain can implement entirely from `interfaces/protocol_v1.md`, `interfaces/protocol_v1.yaml`, and the golden frames.

Changed:
- Real STM32 execution, USART2, encoder acquisition, ICM-42688-P, battery ADC, TIM1 PWM, TB6612 control, and motor rotation are accepted hardware facts.
- Verified encoder convention is left-forward raw CPS negative and right-forward raw CPS positive; normalized logical forward is positive on both wheels. Encoder 1/2 and Motor A/B wheel ownership remain TBD.
- Protocol version 1.0 is frozen: Standard 11-bit CAN IDs, CAN FD+BRS, little-endian explicit serialization, and CAN FD link-layer CRC only.
- Transport timing is nominal 500 kbit/s (prescaler 17, SEG1 15, SEG2 4, SJW 4, 80% sample point) and data 2 Mbit/s (prescaler 5, SEG1 13, SEG2 3, SJW 3, 82.3529% sample point).
- Host IDs are `0x080` motion authority at 10 Hz, `0x081` body motion command at 50 Hz, and `0x082` heartbeat at 10 Hz. STM32 IDs are `0x180` system at 10 Hz, `0x181` wheel at 50 Hz, `0x182` IMU at 100 Hz, and `0x183` battery at 2 Hz.
- Body command scaling is signed 1 mm/s and 1 mrad/s. Wheel rate is signed counts/s, IMU scaling is 0.0001 m/s^2 and 0.00001 rad/s, battery is 1 mV, and STM32 timestamps are unsigned monotonic milliseconds.
- STM32 enforces a 250 ms command timeout and 500 ms host-heartbeat/authority timeouts using local reception time. Startup/reconnect requires heartbeat, explicit DISARMED, ARMED, then a fresh body command; old motion is never restored.
- STM32 firmware now includes bounded interrupt-driven FDCAN RX, task-context protocol validation, centralized command/safety integration, CAN health/error handling, and deterministic telemetry serialization.

Action required:
- ROS domain: inspect the real Orange Pi CAN capability, configure SocketCAN FD for 500 kbit/s / 2 Mbit/s, implement the bridge exactly against Protocol v1, and preserve the required authority/session/freshness behavior.
- Cross-domain integration: verify the exact transceiver module, termination, logic levels, and real bidirectional CAN FD frames before claiming physical CAN acceptance.

## 2026-08-24T11:11:42+08:00 — STM32 peripheral, RTOS, and safety firmware ready for hardware acceptance

Domain: Firmware
Impact: The STM32 production firmware foundation now builds locally and is ready for first physical acceptance. The ROS domain must not assume hardware acceptance or a finalized CAN application protocol.

Changed:
- Integrated MCU/peripheral, FreeRTOS, safety, watchdog, motor, sensor, wheel-control, diagnostics, command, and telemetry foundations are present in one firmware project.
- TIM2 and TIM3 are both configured and handled as 16-bit encoder counters.
- Debug and Release firmware images build with zero warnings; no MCU flashing or motor actuation was performed.
- Final CAN IDs, payloads, heartbeat semantics, and ROS bridge contracts remain intentionally undefined.

Action required:
- None for the ROS domain until STM32 hardware acceptance succeeds and the shared transport protocol stage begins.

## 2026-08-23T20:25:00+08:00 — STM32 real-time control phase established

Domain: System
Impact: The high-computing perception/SLAM/Navigation v1 stack is frozen. Active development moves to the STM32 real-time control domain.

Changed:
- STM32G474RET6 / DeveBox STM32G474R Ver:20 selected as the real-time controller.
- Pin Allocation v1 is frozen.
- HSE baseline is 8 MHz and SYSCLK target is 170 MHz.
- TIM1 motor PWM target is 10 kHz.
- TIM2 and TIM3 are reserved for hardware quadrature encoders.
- SPI1 is reserved for ICM-42688-P.
- ADC1_IN6 on PC0 is reserved for battery measurement.
- USART2 is reserved for debug.
- FDCAN1 on PA11/PA12 is the CAN controller with 500 kbit/s nominal bring-up bitrate.
- Safe startup requires MOTOR_STBY LOW, direction GPIO LOW, and IMU_CS HIGH.

Action required:
- Firmware domain: complete CubeMX baseline and safe MCU bring-up.
- ROS domain: remain on the frozen high-computing baseline until interface/bridge integration begins.
