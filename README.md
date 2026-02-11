# Eurobot 2026 Main

Main workspace for Eurobot 2026 robotics competition with behavior tree management, navigation, and ROS 2 interfaces.

## Quick Start

### Setup

```bash
# Clone and setup
git clone https://github.com/DIT-ROBOTICS/Eurobot-2026-Main.git
cd Eurobot-2026-Main

# Add to ~/.bashrc
export USER_UID=$(id -u)
export USER_GID=$(id -g)

# Build and run
cd docker && docker compose build
cd .. && ./main build
./main enter  # Enter development container
```

### Testing

**1. Prepare external systems**

Make sure your **localization**, **navigation**, **vision**, and **mission** systems are ready and running before testing.

**2. Enter the container**

```bash
./main enter
```

**3. Run the test script**

```bash
bash test_launch.sh
```

This launches a tmux session with 3 panes:

| Pane | Command |
|------|---------|
| Left | `ros2 launch bt_core bt_engine_launch.py` |
| Right-Top | `ros2 launch startup startup_launch.py` |
| Right-Bottom | `python3 mock_groups_test.py` (interactive tester) |

**Tmux controls:**

| Key | Action |
|-----|--------|
| Mouse click | Switch between panes (mouse mode is enabled) |
| `Ctrl+B` then arrow keys | Switch between panes |
| `Ctrl+B` then `D` | Detach from session (triggers auto-reset prompt) |

After detaching, you will be prompted to **press Enter to restart** all nodes or **press `q` to quit**.

## Workspace Structure

```
Eurobot-2026-Main/
├── docker/                    # Docker configs & scripts
│   ├── Dockerfile            # Container image
│   ├── docker-compose.yaml   # Services
│   └── scripts/              # Groot, VNC, micro_ros, settings
├── src/                      # ROS 2 packages
│   ├── behaviortree_ros2/   # BT executor
│   ├── Interface/           # ROS 2 interfaces
│   └── startup/             # Startup node
└── main                     # CLI script
```

## Main Commands

| Command | Description |
|---------|-------------|
| `./main build` | Build workspace with colcon |
| `./main enter` | Enter development container |
| `./main groot` | Launch Groot2 (requires X11) |
| `./main groot-vnc` | Launch Groot2 in VNC |
| `./main vnc` | Start VNC server & enter container |
| `./main close` | Stop all containers |

## VNC Access

```bash
./main vnc
```

Connect with VNC viewer to `localhost:5902`, password: `ros`

## Development Workflow

```bash
# Enter container
./main enter

# Source workspace
source install/setup.bash

# Build specific packages
colcon build --packages-select <package_name>

# Run nodes
ros2 run <package_name> <node_name>
```

## ROS 2 Packages

- **behaviortree_ros2** - BehaviorTree.CPP integration with ROS 2
- **btcpp_ros2_interfaces** - BT interfaces (ExecuteTree, FirmwareMission, Navigation)
- **startup** - System initialization node

## Docker Services

- **main-build** - Build service (host network)
- **main-develop** - Interactive dev container (host network)
- **main-vnc** - VNC with XFCE desktop (bridge network, port 5902)

## Environment Variables

Auto-set by `main` script or add to `~/.bashrc`:

```bash
export USER_UID=$(id -u)      # Container user ID
export USER_GID=$(id -g)      # Container group ID
export ROS_DOMAIN_ID=100      # ROS 2 domain (default: 100)
```

## Troubleshooting

**Permission errors:**
```bash
echo $USER_UID $USER_GID  # Should match host UID/GID (usually 1001)
cd docker && docker compose build
```

**VNC fails to start:**
```bash
./main close
docker compose exec main-vnc bash -c "sudo rm -rf /tmp/.X11-unix/X2 /tmp/.X2-lock"
./main vnc
```

## Dependencies

ROS 2 Humble, Navigation2, BehaviorTree.CPP, Cyclone DDS, Foxglove Bridge, XFCE (VNC), TigerVNC (VNC)

## License & Contact

DIT Robotics Team
