# AGENTS.md — RobotProject Final Demo Contract

> Mandatory for every Codex session. Read first. Follow exactly.
> Modify this file only when the user explicitly requests a policy update.

## 1. Mission and Work Rule

RobotProject is in the **final demo phase**. The foundation is already accepted:

- STM32 closed-loop differential-drive control and actuator safety;
- Protocol 1.0 over CAN FD `can3`;
- real wheel odometry and sole `odom -> base_link` authority;
- RPLIDAR mapping and saved-map localization;
- real full-map Nav2 autonomous navigation;
- DWB thin-wall/corner navigation fix.

Do not reopen completed bring-up unless a new reproducible failure points there.

The only target is the final industrial material-transport AMR demo:

```text
one mission command
-> HOME to requested MATERIAL
-> RGB-D precision docking
-> wait 3 s
-> requested STATION
-> delivery complete
-> return HOME
-> RGB-D precision docking
-> MISSION COMPLETE
-> IDLE
```

Work only like this:

```text
inspect minimum needed
-> reuse existing system
-> smallest valid change
-> build only if needed
-> one focused validation
-> fix observed failure only
-> document accepted result
-> next dependency
```

Prefer configuration, reuse, or deletion over new infrastructure.

Do not run broad regressions, speculative tuning, unrelated refactors, exhaustive diagnostics, benchmark campaigns, or repeated acceptance tests.

---

## 2. Ownership and Frozen Baseline

```text
firmware/    STM32 domain
ros2_ws/     ROS / Orange Pi domain
interfaces/  shared frozen protocol
docs/        persistent engineering knowledge
```

### Firmware Codex

- Owns `firmware/` and firmware-specific docs.
- Never inspect/build/modify `ros2_ws/`.
- Final-demo default: **NO CHANGE**.
- Protocol/safety changes require a demonstrated blocker and user decision.

### ROS Codex

- Owns `ros2_ws/`, ROS configuration, final-demo integration and ROS docs.
- Never inspect/build/modify firmware source.
- Consume firmware only through `interfaces/` and accepted docs.
- Final-demo default: **ROS Codex only**.

Do not run both roles unless a real cross-domain dependency requires it.

### Accepted production baseline

```text
STM32 firmware                0.5.5
Protocol                      1.0 FROZEN
SocketCAN                     can3
ROS                           Humble
Controller                    dwb_core::DWBLocalPlanner
FollowPath.forward_prune_distance  0.4 m
navigation linear cap         0.20 m/s
```

Accepted TF ownership:

```text
map -> odom        Cartographer localization
odom -> base_link  bridge / wheel odometry
base_link -> laser static TF
```

Exactly one `odom -> base_link` publisher is allowed.

The accepted full-map/thin-wall navigation result is closed. Do not retune DWB, footprint, inflation, planner, odometry, CAN, firmware, or localization without new evidence.

Detailed CAN IDs, timeouts, wheel signs/scales/geometry and wire layouts belong in `interfaces/` and technical docs, not this file.

Initial pose may be provided during system setup if required by the accepted localization workflow. After mission acceptance, nominal operation must not require another navigation goal or manual recovery.

---

## 3. Final Demo Contract

Physical locations:

```text
HOME
WAREHOUSE: MATERIAL_1, MATERIAL_2, MATERIAL_3
STATION_1
STATION_2
```

Precision docking is required only at:

```text
HOME
MATERIAL_1
MATERIAL_2
MATERIAL_3
```

`STATION_1` and `STATION_2` use normal Nav2 arrival only.

The user sends one mission, for example:

```text
STATION_1 needs MATERIAL_2
```

The system must derive both targets and complete the mission without another user navigation goal.

Mission lifecycle:

```text
IDLE
-> MISSION_RECEIVED
-> NAV_TO_MATERIAL
-> MATERIAL_DOCKING
-> WAIT_3_SECONDS
-> NAV_TO_STATION
-> DELIVERY_COMPLETE
-> RETURN_HOME
-> HOME_DOCKING
-> MISSION_COMPLETE
-> IDLE
```

Mission lifetime is higher than any individual Nav2 action. Normal planner/controller wait or recovery must not discard the mission.

---

## 4. Responsibility Split

### Mission Manager

Owns only high-level sequencing:

- accept one mission;
- choose material and station;
- invoke Nav2 phases;
- invoke docking phases;
- wait 3 seconds after material docking;
- retain mission through normal Nav2 wait/recovery;
- complete delivery, return HOME, then IDLE.

It must not implement local planning, obstacle-distance rules, costmaps, or actuator safety.

### Nav2

Owns all ordinary navigation behavior:

- global planning;
- local control;
- 2D obstacle handling;
- dynamic costmaps;
- replanning;
- native wait/recovery;
- continuation toward the same goal.

Do **not** add production obstacle state machines based on custom `0.40 m`, `0.10 m`, front sectors, clear timers, custom wait, or custom resume logic.

### Astra Pro docking

Astra Pro is used only for precision docking at MATERIAL slots and HOME.

Minimal docking flow:

```text
Nav2 reaches pre-dock vicinity
-> Nav2 motion ends
-> docking becomes sole motion producer
-> RGB + depth target observation
-> longitudinal / lateral / heading error
-> bounded low-speed correction
-> DOCKED
-> stop / withdraw authority
-> Mission Manager advances
```

Reuse existing Astra/OpenNI/perception assets. Do not create a parallel camera stack without need.

Do not invent docking markers, target geometry, tolerances, or camera behavior. Inspect existing project assets first. If a required physical target definition is genuinely absent, issue `DECISION REQUIRED` instead of guessing.

Lost/invalid docking target => stop safely; never drive blind.

### Motion boundary

The ROS motion boundary stays thin. It owns only:

- selecting the approved active motion source;
- command freshness while that source is active;
- Motion Authority requests;
- required ROS/bridge health checks;
- zero/disarm when motion ends.

Exactly one motion producer may own the robot:

```text
Nav2 controller
OR
RGB-D docking controller
```

Never merge or compete velocity commands.

During Nav2 planning/native Wait/non-motion phases: withdraw authority; absence of `/cmd_vel` is not a command-timeout fault.

During an active motion phase: command freshness remains strict.

### STM32

STM32 remains authoritative for wheel control and actuator-level safety. Never duplicate or weaken Motion Authority, freshness supervision, watchdogs, CAN/protocol faults, Safety Supervisor, motor-safe startup, operator STOP, or true fault latches.

---

## 5. Obstacles and Safety Semantics

Normal obstacles belong to Nav2.

```text
bypass exists:
obstacle -> costmap update -> replan -> bypass -> continue same mission

no bypass:
obstacle -> stop -> retain mission/goal -> wait/replan -> obstacle clears -> continue
```

Ordinary obstacles must not require the user to resend the mission or goal.

Ordinary obstacle waiting is **not** a true system fault.

True safety faults remain separate and latched according to the existing accepted system. Never auto-clear a real safety fault merely to continue a mission.

Do not remove or weaken existing independent safety protections while implementing final-demo behavior.

---

## 6. Active Final-Demo Execution Order

Proceed one dependency at a time:

```text
1. confirm/reuse Astra Pro RGB-D runtime and perception assets
2. define the minimum docking-target observation contract
3. make one physical docking target work end-to-end
4. reuse that docking implementation for MATERIAL_1/2/3 and HOME by configuration
5. implement minimal Mission Manager + Nav2/docking motion-source handoff
6. run one complete material -> station -> HOME mission without injected obstacle
7. verify native obstacle bypass or blocked-path wait/resume without losing the mission
8. run the final one-command industrial AMR demo
```

Do not build all stages in parallel.

Do not repeat mapping, odometry, full-map navigation, thin-wall, CAN, or firmware acceptance unless the current integration exposes a related failure.

Current unfinished critical path is **RGB-D precision docking -> Mission Manager -> final integrated mission**.

---

## 7. Final Acceptance

PASS requires one real mission proving:

1. one user transport command;
2. correct material and station selected;
3. autonomous HOME -> requested MATERIAL navigation;
4. Astra Pro precision material docking;
5. 3-second simulated loading wait;
6. automatic navigation to requested station;
7. normal obstacle bypass, or safe blocked-path wait;
8. automatic continuation of the same mission after obstacle clearance;
9. station delivery completion without RGB-D docking;
10. automatic return HOME;
11. Astra Pro precision HOME docking;
12. `MISSION_COMPLETE -> IDLE`;
13. no true safety mechanism bypassed, weakened, or silently auto-cleared.

Project identity:

> **Industrial material-transport AMR using LiDAR autonomous navigation, dynamic obstacle recovery, and RGB-D precision docking.**

---

## 8. Validation, Actions, and Documentation

Real hardware is the acceptance source of truth.

For each change:

- validate only changed behavior;
- prefer one representative real run;
- collect only evidence needed for PASS/FAIL;
- stop tuning when the requirement passes;
- never bypass safety for testing.

Codex may directly perform relevant read-only inspection, project edits, affected-package builds, normal project deployment, bounded non-destructive checks, and required docs/CHANGELOG updates inside its assigned domain.

Stop before:

- `sudo` / root or system package changes;
- OS/kernel/BSP changes;
- destructive filesystem/Git operations;
- Protocol 1.0 changes;
- cross-domain ownership changes;
- Motion Authority/watchdog/fault/safety semantic changes;
- materially new unsafe actuation;
- unresolved physical/electrical assumptions.

For a genuine unresolved decision:

```text
DECISION REQUIRED
Observed:
Why execution stopped:
Recommended action:
Impact:
Decision requested:
```

Do not ask questions repository inspection can answer.

Runtime target:

```text
HwHiAiUser@robot-core.local
/data/ros2_ws
/data/projects
```

Documentation discipline:

- update technical docs only for meaningful accepted results;
- use `CHANGELOG.md` for shared contracts, safety/architecture changes, or peer-relevant milestones;
- new CHANGELOG timestamps are exactly `YYYY-MM-DDTHH:MM`;
- README files are showcase documents, not development logs;
- never modify `AGENTS.md` unless the user explicitly requests it.

Milestone reports stay short:

```text
MILESTONE
Stage:
Status: PASS / PARTIAL / FAIL
Changed:
Real evidence:
Safety state:
Open blocker:
Next:
```

The next action must always be the smallest step that advances the final demo.
