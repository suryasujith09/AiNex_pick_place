# Quick Start Guide

## New AiNex Robot Setup

When cloning this package to a new AiNex robot, follow these steps:

### 1. Clone the Package

```bash
cd ~/ros_ws/src
git clone <your-repo-url> pick_place
```

### 2. Install Dependencies

Make sure the following are installed:
- ROS Noetic
- Python 3 packages: `opencv-python`, `numpy`

```bash
pip3 install opencv-python numpy
```

### 3. Build the Workspace

```bash
cd ~/ros_ws
catkin build pick_place
source devel/setup.bash
```

### 4. Make Scripts Executable

```bash
chmod +x ~/ros_ws/src/pick_place/scripts/*.py
```

### 5. Verify Action Files

Ensure these action files exist in `/home/ubuntu/software/ainex_controller/ActionGroups/`:
- `lefthand_place_square.d6a`
- `righthand_place_square.d6a`
- `walk_ready.d6a`

If missing, copy from your original AiNex robot.

### 6. Run the Game

**Terminal 1 - Start Base System:**
```bash
roslaunch ainex_bringup base.launch
```

**Terminal 2 - Run Game:**
```bash
source ~/ros_ws/devel/setup.bash
roslaunch pick_place interactive_game.launch
```

Or run directly:
```bash
source ~/ros_ws/devel/setup.bash
rosrun pick_place interactive_game_final.py
```

## Verification Checklist

- [ ] Package cloned to `~/ros_ws/src/pick_place`
- [ ] Workspace built successfully with `catkin build`
- [ ] Scripts are executable
- [ ] Action files exist in ActionGroups folder
- [ ] base.launch starts without errors
- [ ] Game shows camera feed with ROI boxes

## Expected Behavior

1. Robot initializes to `walk_ready` position
2. Camera tilts down to look at table (servo 24 = 200)
3. Window shows live camera feed with LEFT and RIGHT ROI boxes
4. Place colored squares in the ROI zones (boxes turn green)
5. Remove one square and wait 1 second
6. Robot executes corresponding pick-and-place action

## Common Issues

**Camera not found:**
- Check `/dev/video0` exists
- Kill conflicting processes: `pkill -f usb_cam_node`

**Servo controller error:**
- Ensure `base.launch` is running first
- Check ROS topics: `rostopic list | grep servo`

**Action not executing:**
- Verify .d6a files exist
- Check MotionManager initialization messages

**Colors not detected:**
- Adjust lighting
- Lower COLOR_THRESHOLD in script (default 0.15)

## Configuration

Edit `scripts/interactive_game_final.py` for customization:

```python
LEFT_ROI = (50, 50, 150, 150)    # Detection zone position
RIGHT_ROI = (440, 50, 150, 150)
STABILITY_FRAMES = 30             # Confirmation time
COLOR_THRESHOLD = 0.15            # Detection sensitivity
```

## Support

See full documentation in `README.md`
