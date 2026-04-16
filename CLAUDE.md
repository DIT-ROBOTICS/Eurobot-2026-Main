# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

All development happens inside the Docker container. From the host:

```bash
export USER_UID=$(id -u) && export USER_GID=$(id -g)  # add to ~/.bashrc
cd docker && docker compose build   # one-time setup
cd .. && ./main build               # colcon build inside container
./main enter                        # enter dev container
```

Inside the container:

```bash
source install/setup.bash

# Build all packages
colcon build

# Build a single package
colcon build --packages-select bt_core

# Run the full system test (tmux with 4 panes)
bash test_launch.sh

# Launch bt_core directly
ros2 launch bt_core bt_engine_launch.py
```

The `ROS_DOMAIN_ID` environment variable selects the robot configuration:
- `11` → White robot (`robot_config_white.yaml`, `map_points_white.yaml`)
- `13` → Black robot (`robot_config_black.yaml`, `map_points_black.yaml`)
- anything else → default configs

## Architecture Overview

This is a ROS 2 (Humble) workspace implementing an autonomous robot for the Eurobot competition using BehaviorTree.CPP.

### Package Roles

| Package | Purpose |
|---------|---------|
| `bt_core` | Core behavior tree engine, all BT action nodes, and `DecisionCore` |
| `sensors` | ROS subscribers that feed data into the BT blackboard |
| `navigation` | Nav2 action clients wrapped as BT nodes |
| `startup` | System orchestration: team/plan selection, startup handshake |
| `utils` | Shared types header (`bt_config.hpp`) |
| `behaviortree_ros2` | Vendored BehaviorTree.CPP ROS 2 integration library |
| `Interface/btcpp_ros2_interfaces` | Custom ROS 2 service/message types |

### Data Flow

1. **Startup** (`startup_new.cpp`) receives plan selection (team, robot, plan number) from an external web service or default config. It publishes the chosen `.xml` filename to `/robot/startup/plan_file`.
2. **BTengine** (`bt_core/src/bt_engine.cpp`) receives the plan filename, loads the corresponding XML behavior tree from `bt/`, creates the BT factory with all registered nodes, initializes the blackboard, and waits for a start signal on `/robot/start_signal`.
3. **Sensor nodes** (`sensors/src/`) run as background threads feeding field state into the blackboard:
   - `CamReceiver` — vision data (pantry/collection status, hazelnut flip state)
   - `FirmwareReceiver` — microcontroller feedback
   - `LocReceiver` — robot pose from localization
   - `GameInfoReceiver` — game timer, opponent position
4. **BT execution** — the tree ticks at `time_rate` Hz until `game_time >= terminate_time`. All BT nodes share state via the single `BT::Blackboard::Ptr blackboard`.

### Shared Types (`src/utils/include/bt_config.hpp`)

This header is included by all packages. It defines the key domain enums and structs:
- `MapPoint` — a navigation goal (x, y, staging_dist, sign, `DockType`, direction)
- `FieldStatus` — `EMPTY / OCCUPIED / UNKNOWN / CAN_ROB`
- `FlipStatus` — hazelnut flip state per robot side
- `ActionType` — `TAKE / PUT / FLIP / DOCK / GO_HOME / CURSOR / NAV / ROTATE`
- `GoalPose` — named poses A–R for pantry (A–J) and collection (K–R) points, plus special positions
- Constants: `PANTRY_LENGTH=10`, `COLLECTION_LENGTH=8`, `HAZELNUT_LENGTH=4`

### DecisionCore (`bt_core/src/decision_core.cpp`)

The central decision-making BT node. Each tick it:
1. Reads current field status, robot pose, and sequences from the blackboard.
2. **Priority 1 — Predefined sequence**: follows `pantry_sequence` / `collection_sequence` from `params/mission_sequence_{white|black}.json`. Points are skipped if no longer viable.
3. **Priority 2 — Spectral scoring fallback**: when the sequence is exhausted, scores all viable points using configurable `aggressiveness`, `sensitivity`, `rival_sigma`, and `rival_distance_threshold` parameters (set in `params/robot_config_*.yaml`).
4. Sets the chosen `GoalPose` and `ActionType` on the blackboard for downstream navigation/mission nodes.

### Plan Files

BT XML files live in `src/bt_core/bt/` and are named `{robot}_{team}_{plan_number}.xml` (e.g., `white_yellow_1.xml`). The startup node determines which file to load and publishes the filename; BTengine loads it at runtime.

### Robot Config & Map Points

- `src/startup/params/robot_config_{white|black|default}.yaml` — timing, BT paths, dock types, staging distances, spectrum scoring parameters
- `src/bt_core/params/map_points_{white|black|default}.yaml` — flat list of `MapPoint` values (6 doubles per point: x, y, staging_dist, sign, dock_type, direction)
- `src/bt_core/params/mission_sequence_{white|black}.json` — `pantry_sequence` and `collection_sequence` arrays of point indices

### Blackboard Key Entries

| Key | Type | Set by |
|-----|------|--------|
| `team` | `string` | BTengine |
| `robot` | `string` | BTengine |
| `selected_plan` | `int` | BTengine |
| `game_time` | `double` | BTengine (subscription) |
| `stop_time` / `go_home_time` | `double` | BTengine |
| `MapPointList` | `vector<MapPoint>` | BTengine |
| `pantry_sequence` / `collection_sequence` | `vector<int>` | BTengine (from JSON) |
| `collection_info` / `pantry_info` | `vector<FieldStatus>` | CamReceiver |
| `collection_info_raw` / `pantry_info_raw` | `Int32MultiArray` | CamReceiver (camera mode) |
| `hazelnut_status` | `vector<vector<FlipStatus>>` | CamReceiver |

`use_camera_for_planning` (bool param) switches FieldUpdater and DecisionCore between raw camera data and pre-processed field status vectors.
