# Drone environment

The drone script uses a dedicated Python 3.7 Conda environment because the
RoboMaster SDK does not work reliably with the newer Python used by the ROS 2
workspace.

Create the environment from the repository root:

```bash
conda env create --file Drone/environment.yml
conda activate drone-robomaster
```

Verify the important packages:

```bash
python --version
python -c "import robomaster, cv2; print('RoboMaster SDK loaded'); print('OpenCV', cv2.__version__)"
```

`python --version` should report Python 3.7.x. The environment uses
`opencv-contrib-python`, rather than the base OpenCV package, because
`drone_tag.py` requires the `cv2.aruco` module and its `ArucoDetector` API.

Connect the computer to the drone Wi-Fi and run:

```bash
python Drone/drone_tag.py
```

The result is written to `Drone/targetLocation.json`. After the drone lands,
convert it from the drone frame (centimetres) to the robot frame (metres):

```bash
python Drone/convert_to_robot_frame.py
```

By default, the converter reads `Drone/targetLocation.json`, uses the robot
start location `x = -1 m`, `y = -6 m`, and writes the converted target to
`Drone/targetLocation_robot.json`. For example, a drone target at
`x = 100 cm`, `y = 200 cm` is converted to the robot location
`x = -2 m`, `y = -8 m`.

Override the default locations when needed:

```bash
python Drone/convert_to_robot_frame.py \
  --input Drone/targetLocation.json \
  --output Drone/targetLocation_robot.json \
  --start-x -1 \
  --start-y -6
```

Then manually switch to the robot network and transfer the result:

```bash
python Drone/transfer_and_start_mission.py \
  --robot pi@192.168.1.20 \
  --remote-file /tmp/targetLocation.json
```

To convert and transfer in one step, use the automation script for your
platform. The configuration block at the top of each script sets the robot
address and password, start coordinates, local files, and remote destination.
Replace the `ROBOT_PASSWORD` placeholder before running. Both scripts initially
use `pi@144.32.70.209`.

The Bash script requires `sshpass`, and the Windows batch script requires
PuTTY's `pscp.exe` on `PATH`. The configured password is stored as plaintext,
so restrict access to these scripts and do not commit a real password to Git.

```bash
./Drone/convert_and_transfer.sh
```

```bat
Drone\convert_and_transfer.bat
```

Pass a different SSH destination as the first argument when needed:

```bash
./Drone/convert_and_transfer.sh pi@192.168.1.30
```

```bat
Drone\convert_and_transfer.bat pi@192.168.1.30
```

The conversion must succeed before the transfer starts. These scripts use the
default converter files: `Drone/targetLocation.json` is converted to
`Drone/targetLocation_robot.json`, which is then transferred to the robot.

To rebuild the environment after changing `environment.yml`:

```bash
conda env update --name drone-robomaster --file Drone/environment.yml --prune
```

Remove it when it is no longer required:

```bash
conda env remove --name drone-robomaster
```
