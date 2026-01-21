# Git Push Instructions for pick_place Package

## Package is Ready for Git! ✅

Your `pick_place` package is now fully configured and ready to be cloned to any AiNex robot.

## What's Included

### Core Files
- **CMakeLists.txt** - Build configuration (includes scripts and launch files)
- **package.xml** - ROS package metadata with all dependencies
- **.gitignore** - Excludes build artifacts and temp files

### Scripts
- **scripts/interactive_game_final.py** - Main production script (MotionManager-based)
- **scripts/interactive_game.py** - Legacy version

### Launch Files
- **launch/interactive_game.launch** - ROS launch file for the game

### Documentation
- **README.md** - Complete documentation with features, usage, troubleshooting
- **QUICKSTART.md** - Quick setup guide for new robots
- **GIT_INSTRUCTIONS.md** - This file

### Helper Scripts
- **run_game.sh** - Quick run script
- **launch.sh** - Alternative launch script

## Git Commands

### 1. Stage All Files

```bash
cd ~/ros_ws/src/pick_place
git add .
git status  # Review what will be committed
```

### 2. Commit Changes

```bash
git commit -m "Complete pick_place package with MotionManager integration

- Interactive color detection game
- Proper action execution using MotionManager
- Camera positioning for downward view
- ROI-based zone monitoring with stability check
- Complete documentation and launch files
- Ready for deployment on new AiNex robots"
```

### 3. Push to Remote

```bash
# If you haven't set up remote yet:
git remote add origin <your-repo-url>

# Push to main/master branch:
git push -u origin main
# or
git push -u origin master
```

## Deployment on New Robot

On the new AiNex robot:

```bash
# 1. Clone the repository
cd ~/ros_ws/src
git clone <your-repo-url> pick_place

# 2. Build
cd ~/ros_ws
catkin build pick_place
source devel/setup.bash

# 3. Make executable
chmod +x ~/ros_ws/src/pick_place/scripts/*.py

# 4. Run
roslaunch ainex_bringup base.launch  # Terminal 1
roslaunch pick_place interactive_game.launch  # Terminal 2
```

## Dependencies Required on New Robot

These must be installed on the target AiNex robot:

### System Dependencies
- ROS Noetic
- ainex_bringup package
- ainex_kinematics package
- ros_robot_controller_sdk

### Python Dependencies
```bash
pip3 install opencv-python numpy
```

### Action Files
Copy these from your original robot to `/home/ubuntu/software/ainex_controller/ActionGroups/`:
- `lefthand_place_square.d6a`
- `righthand_place_square.d6a`
- `walk_ready.d6a`

## Package Structure

```
pick_place/
├── .gitignore                         # Git ignore rules
├── CMakeLists.txt                     # Build configuration
├── package.xml                        # Package metadata
├── README.md                          # Full documentation
├── QUICKSTART.md                      # Quick setup guide
├── launch/
│   └── interactive_game.launch        # ROS launch file
├── scripts/
│   ├── interactive_game_final.py      # Main production script ⭐
│   └── interactive_game.py            # Legacy version
└── run_game.sh                        # Quick run helper
```

## Key Features

✅ **Portable**: No hardcoded paths, uses ROS standards
✅ **Documented**: Complete README and QUICKSTART guides
✅ **Proper ROS Package**: CMakeLists.txt, package.xml, launch files
✅ **Production Ready**: Using MotionManager for reliable action execution
✅ **Tested**: Fully working on your current robot

## Configuration

The main configuration is in `scripts/interactive_game_final.py`:

```python
# ROI positions (already optimized)
LEFT_ROI = (50, 50, 150, 150)
RIGHT_ROI = (440, 50, 150, 150)

# Camera position (looking down)
PAN_CENTER = 500   # Servo 23
TILT_DOWN = 200    # Servo 24

# Detection parameters
STABILITY_FRAMES = 30        # 1 second confirmation
COLOR_THRESHOLD = 0.15       # 15% detection threshold
```

## Verification

After cloning to a new robot, verify:

1. ✅ Package builds: `catkin build pick_place`
2. ✅ Scripts are executable: `ls -l scripts/*.py`
3. ✅ Launch file found: `rospack find pick_place`
4. ✅ Action files exist in ActionGroups folder
5. ✅ Can run: `roslaunch pick_place interactive_game.launch`

## Support

- See README.md for detailed troubleshooting
- Check QUICKSTART.md for quick reference
- All paths use ROS standards (rospack, catkin workspace)

## Ready to Push! 🚀

Your package is completely ready. Just run the git commands above and it will work on any other AiNex robot with the same base system.
