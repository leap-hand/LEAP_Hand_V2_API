## Welcome to the LEAP Hand V2 ROS 2 SDK

This package provides ROS 2 support for the LEAP Hand V2.  
Tested on **Ubuntu 22.04** with the **Humble** ROS 2 distribution.

---

### System Setup (Ubuntu)

**Install ROS 2 Humble:**  
    Follow the official [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html). 

**Create a new ROS 2 workspace (or use an existing one):**
    Follow the [ROS 2 workspace creation tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

---

### ROS2 Setup

1. **Add the LEAP package:**

   * Copy the `leap_v2_ros2` folder into the `src/` directory of your workspace.

2. **Build the workspace:**

   ```bash
   colcon build --symlink-install
   ```

3. **Source ROS 2 and your workspace:**
   (Optional: add these to your `.bashrc` for convenience)

   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ```

---

### First-Time Preparation
* **Install Python dependencies:**
   ```bash
   pip install pyserial numpy
    ```
* **Make scripts executable:**

  ```bash
  chmod +x leaphand_node.py
  chmod +x ros2_example.py
  ```
* Calibrate by running `calibration.py`
---

### Launch Instructions

1. Power the hand with **7.6V**.
2. Connect a **USB-C** cable to the hand. *(Avoid too many USB extension cables.)*
3. Launch the LEAP node:

   ```bash
   ros2 launch launch_leap_v2.py
   ```

---

### Usage Example

* Check the example script: `ros2_example.py`:

  * Queries the LEAP Hand ROS 2 service.
  * Publishes a target pose for the hand to move to.
  * Provides a basic starting point for your own ROS 2 application.
  * Functionality is similar to the Python API, but uses ROS 2 topics and services.

For more advanced usage, refer to:

* The [ROS 2 documentation](https://docs.ros.org/en/humble/index.html)
* The [LEAP Hand V2 documentation](http://v2.leaphand.com/)