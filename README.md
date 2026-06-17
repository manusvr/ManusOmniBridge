# Manus Omni Bridge

This workspace marries the **Manus MetaGlove Pro** with the **Agibot Omnihand**. Through **Pinocchio** (kinematics) and **NLopt** (non-linear optimization), we retarget human hand movements from Manus into the robotic fingers of the Omnihand.

---

##  Packages

This workspace contains:

### 1. The Human Side (Manus)
* `ManusSDK` - The official Manus SDK headers/libs.
* `manus_ros2` - The ROS 2 publisher.
* `manus_ros2_msgs` - Custom message definitions.

### 2. The Robot Side (Agibot)
* `omnihand_node` - The interface node that actually talks to the Agibot hardware.
* `omnihand_*_msgs` (e.g., `omnihand_2025_node_msgs`, `omnihand_pro_2025_node_msgs`) - Agibot's native message structures.
* `omnihand_description-*` - URDFs, visual meshes, and kinematics configurations defining the Omnihand (versions O10 and O12).

### 3. The Bridge
* `manus_omni_bridge` - This node subscribes to the MetaGlove Pro joints, feeds them into a **Pinocchio** kinematic tree, and uses **NLopt** to optimize the output joint angles so the Omnihand mimics your real hand flawlessly without breaking itself.

---

## 🛠️ Dependencies

* **ROS 2** (Tested on Humble)
* **Eigen3** (Math fundamentals)
* **Pinocchio** (`ros-<distro>-pinocchio` or compiled from source)
* **NLopt** (`libnlopt-dev` `libnlopt-cxx-dev`- Non-linear optimization library)
* **Manus Core** running on your Windows/Linux machine (make sure the glove is connected and calibrated in Manus Core).

---

## 🚀 Building the Project

Just your standard ROS 2 `colcon` workflow. From the root of your workspace (`\Manus_Omni_bridge\`):

```bash
# Build the entire workspace
colcon build --symlink-install

# Source it
source install/setup.bash
```

---

## 🕹️ Running this project

You have a few ways to launch the stack depending on your exact setup. By default, the `manus_omni_bridge` handles the whole process.

**1. Run the Standard Hand (Direct Mapping)**
```bash
ros2 launch manus_omni_bridge manus_omni_bridge.launch.py
```

**2. Run with the Omnihand Pro**
```bash
ros2 launch manus_omni_bridge manus_omni_bridge.launch.py is_pro:=true
```

**3. Run the Pro Hand + Full NLopt Retargeting**
*(This enables the full mathematical optimization pipeline using Pinocchio and NLopt!)*
```bash
ros2 launch manus_omni_bridge manus_omni_bridge.launch.py is_pro:=true manus_retargeting:=true
```

**4. Debugging Mode**
```bash
ros2 launch manus_omni_bridge manus_omni_bridge.launch.py is_pro:=true debug_print:=true
```

---

