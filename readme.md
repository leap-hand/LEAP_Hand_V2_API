# LEAP Hand V2 SDK

Welcome to the LEAP Hand V2 SDK!  
For details about the LEAP Hand V2 hardware, features, and latest updates, visit our [website](http://v2.leaphand.com/).

---

## Software Setup

- **Python API:** [LEAP Hand Python API]()  
- **ROS 2 API:** [LEAP Hand ROS 2 Module]()  

## Hardware Setup

1. Power the hand using a **7.6V power supply**, as listed in the parts list on our website.
2. Connect the hand with a **USB-C cable**. Avoid using excessive USB extension cables.
3. On Ubuntu, the hand will show up in `/dev/serial/by-id`. This ID stays the same across reboots.
4. Configure this port in your software. Official support is available for **Python** and **ROS 2**.
5. Auto-Calibration:

    - Run `python calibration.py` after your first assembly. This script uses motor current control to automatically detect the open and closed positions of the joints and saves it to a CSV.
    -  If needed, you can fine-tune the calibration by manually editing the minimum and maximum position values in that CSV.
    - You can also rerun the calibration script if for some reason the finger behavior changes.
6. If you are on Windows you can use their FD Studio, otherwise check the `config_tools` folder to change the ID and straighten the motors.
**Tip:** Improve USB latency on Ubuntu by adjusting the latency timer:
```bash
# Check latency (default is 16)
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

# Set it to 1 for lower latency
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

# Confirm the change
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
````

## Hand Features
* Send joint commands:
  * MCP side joint in **radians**
  * Finger curl as **normalized 0–1 values**
* Tendon Control:
  * The tendon control has a linear relationship to the sum of ALL the finger joint angles' actuations.  
  * Note that each joint in the hand has different strengths so that the MCP joint will actuate the most first, followed by the PIP and DIP last when not in contact.  
  * Each joint has approximately a range of 0- 1.5 radians.
  * When in contact, the fingers wrap and conform to the environment.  

* The IDS are as follows:

    | ID    | MCP Side | Curl |
    |------ |----------|------|
    | Index | **0**    | **1** |
    | Middle| **2**    | **3** |
    | Ring  | **4**    | **5** |
    | Thumb | **6**    | **7** |

* Read:

  * **Position**
  * **Velocity**
  * **Motor current**
* Recommended max read frequencies:
  * **90 Hz** when reading position, velocity together.
* Default control mode: PID control, with a current limit cap to protect the motors.
* **⚠️ Do not increase current limits — this can damage the motors.**
* Tuning:

  * If the hand is jittery → **Lower PID values**
  * If the hand feels weak → **Increase PID values**
## Troubleshooting

* No motors detected? Check:

  * Serial port permissions
  * Replug the motors and hand
* Missing motors?

  * Ensure correct motor IDs
  * Check all physical connections


## Useful Tools

* See [Bimanual Dexterity for Complex Tasks](https://bidex-teleop.github.io/) for integrating Manus gloves with LEAP Hand.
* Looking for additional tools? Let me know :)

---
**Kenneth Shaw** — [kshaw2@andrew.cmu.edu](mailto:kshaw2@andrew.cmu.edu)

* Software: **MIT License** for LEAP V2, Feetech Licenses apply for the feetech code.
* CAD Files: **CC BY-NC-SA Attribution-NonCommercial-ShareAlike License**
* Provided **as-is, without warranty**.

If you use LEAP Hand V2 in your research, please cite:

```bibtex
@article{shaw2025leaphandv2,
  title={Demonstrating LEAP Hand v2: Low-Cost, Easy-to-Assemble, High-Performance Hand for Robot Learning},
  author={Shaw, Kenneth and Pathak, Deepak},
  journal={Robotics: Science and Systems (RSS)},
  year={2025}
}
```