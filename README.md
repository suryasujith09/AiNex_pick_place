# Pick & Place Interactive Game

Interactive color detection game for AiNex humanoid robot. The robot detects colored squares and performs pick-and-place actions when objects are removed.

## Features

- **Real-time color detection** using OpenCV and HSV color space
- **ROI-based monitoring** for LEFT and RIGHT game zones
- **Stability checking** (1 second confirmation before action)
- **Proper action execution** using MotionManager for smooth movements
- **Camera positioning** to look down at the game board

## Dependencies

- ROS Noetic
- Python 3.8+
- OpenCV (cv2)
- numpy
- ainex_kinematics
- ainex_bringup
- ros_robot_controller_sdk

## Installation

1. **Clone into your ROS workspace:**
   ```bash
   cd ~/ros_ws/src
   git clone <your-repo-url> pick_place
   ```

2. **Build the workspace:**
   ```bash
   cd ~/ros_ws
   catkin_make
   source devel/setup.bash
   ```

3. **Make scripts executable:**
   ```bash
   chmod +x ~/ros_ws/src/pick_place/scripts/*.py
   ```

## Usage

### Method 1: Using Launch File (Recommended)

Start the base system and game together:

```bash
# Terminal 1: Start base launch
roslaunch ainex_bringup base.launch

# Terminal 2: Run the game
roslaunch pick_place interactive_game.launch
```

### Method 2: Direct Script Execution

```bash
# Terminal 1: Start base launch
roslaunch ainex_bringup base.launch

# Terminal 2: Run script directly
source ~/ros_ws/devel/setup.bash
rosrun pick_place interactive_game_final.py
```

### Method 3: Standalone (Alternative)

```bash
python3 ~/ros_ws/src/pick_place/scripts/interactive_game_final.py
```

## Game Setup

1. Place two colored squares (red/green/blue/yellow) in the camera view
2. Position them in the LEFT and RIGHT ROI zones (visible as green boxes)
3. The robot will monitor these zones continuously
4. Remove one square and hold for 1 second
5. Robot will execute the corresponding pick-and-place action:
   - **LEFT square removed** → `lefthand_place_square` action
   - **RIGHT square removed** → `righthand_place_square` action

## Configuration

Edit these parameters in `scripts/interactive_game_final.py`:

```python
# ROI Zones (x, y, width, height)
LEFT_ROI = (50, 50, 150, 150)    # Left detection zone
RIGHT_ROI = (440, 50, 150, 150)  # Right detection zone

# Stability check
STABILITY_FRAMES = 30  # 1 second at 30 FPS

# Color detection threshold
COLOR_THRESHOLD = 0.15  # 15% of pixels must match color

# Camera position (Servo values)
PAN_CENTER = 500   # Servo 23 - centered
TILT_DOWN = 200    # Servo 24 - looking down
```

## Action Files Required

Ensure these action files exist in `/home/ubuntu/software/ainex_controller/ActionGroups/`:

- `lefthand_place_square.d6a`
- `righthand_place_square.d6a`
- `walk_ready.d6a`

## Troubleshooting

### Camera Not Opening
```bash
# Kill conflicting processes
pkill -f usb_cam_node
sudo systemctl restart ainex_controller
```

### Servo Controller Not Available
```bash
# Make sure base.launch is running
roslaunch ainex_bringup base.launch
```

### Action Not Executing
- Check that action .d6a files exist
- Verify MotionManager initialized correctly
- Ensure gait is disabled before actions

### Colors Not Detected
- Adjust lighting conditions
- Modify HSV color ranges in the script
- Lower COLOR_THRESHOLD value
- Check camera focus

## Development

### File Structure
```
pick_place/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── interactive_game.launch
└── scripts/
    ├── interactive_game_final.py  (Main production script)
    └── interactive_game.py        (Legacy version)
```

### Testing

Run with visual feedback:
```bash
rosrun pick_place interactive_game_final.py
```

Press `q` to quit the game.

## Version History

- **v1.0.0** - Initial release with MotionManager integration
- Camera positioning for downward view
- Stable color detection with 1-second confirmation
- ROI-based zone monitoring

## License

MIT License

## Support

For issues or questions, contact the AiNex development team.
