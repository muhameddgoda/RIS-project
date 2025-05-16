# Duckiebot Line Follower & AprilTag Behaviors

This repository contains a custom ROS 1 catkin package (`my_package`) for:

- **White‐line following** (keep the bot to the left of a white guide line on its right)  
- **AprilTag detection** (pure-Python detector)  
- **Tag-action mapping** (stop/go/turn when a tag appears)  
- **Combined launch** for driving + vision behaviors  

All code runs inside the Duckietown Docker environment on your Duckiebot (e.g. `motoduck`).

---

## 📋 Prerequisites

1. A Duckiebot running the Duckietown template‐ros image (Noetic).  
2. `dts` (Duckietown Shell) installed on your laptop.  
3. A GitHub account & Git installed locally.  

---

## 🛠️ Dependencies

### 1. System packages (`dependencies-apt.txt`)

```text
libapriltag-dev
python3-opencv
ros-noetic-teleop-twist-keyboard
````

These are installed automatically when you rebuild the Docker image.

### 2. Python packages (`dependencies-py3.txt`)

```text
apriltag
```

This is installed via `pip` inside the container.

---

## 📦 Building the Workspace

1. Ensure launcher scripts and Python nodes are executable:

   ```bash
   chmod +x launchers/*.sh
   chmod +x packages/my_package/src/*.py
   ```

2. Build in the Duckiebot container:

   ```bash
   dts devel build -H motoduck
   ```

---

## 🚀 Running Your Nodes

Use one of these Duckietown launchers:

| Launcher            | Description                                                 |
| ------------------- | ----------------------------------------------------------- |
| `line-follower`     | Run the white‐line follower                                 |
| `apriltag-detector` | Run the pure-Python AprilTag detector                       |
| `tag-action`        | Map detected tag IDs → stop/go/turn behaviors               |
| `combined`          | Run line follower + AprilTag detector + tag-action together |

Example:

```bash
dts devel run -H motoduck -L combined
```

---

## 🔧 ROS Launch Files

Under `packages/my_package/launch/` you’ll find:

* `line_follower.launch`
* `apriltag_detector.launch`
* `tag_action.launch`
* `combined.launch`

And in `launchers/` the corresponding shell scripts.

---

## 🗂️ Repository Layout

```
Duckiebot-luna-ros/
├── dependencies-apt.txt
├── dependencies-py3.txt
├── launchers/
│   ├── line-follower.sh
│   ├── apriltag-detector.sh
│   ├── tag-action.sh
│   └── combined.sh
└── packages/
│ ├── apriltag_ros/ ← Fork or clone of AprilRobotics/apriltag_ros
│ ├── apriltag_detector/ ← Fork or clone of ros-misc-utilities/apriltag_detector
    └── my_package/
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/
        │   ├── line_follower.launch
        │   ├── apriltag_detector.launch
        │   ├── tag_action.launch
        │   └── combined.launch
        └── src/
            ├── white_line_follower_node.py
            ├── apriltag_detector_node.py
            └── tag_action_node.py
```

---

## 📖 Further Improvements

* PID control tuning via dynamic reconfigure
* Overlay debug mask & centroid in RViz or OpenCV viewer (on your laptop)
* Integration with Duckietown’s simulator for faster iteration

Feel free to open issues or pull requests if you have ideas or fixes!

