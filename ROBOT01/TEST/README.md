# Raspberry Pi LED test

This standalone script tests the Docker GPIO libraries without ROS. It uses BCM
GPIO numbering and turns both outputs off when stopped with Ctrl+C.

Inside the server container:

```bash
python3 /root/ros2_ws/TEST/test_leds.py --mode blink
python3 /root/ros2_ws/TEST/test_leds.py --mode red
python3 /root/ros2_ws/TEST/test_leds.py --mode green
python3 /root/ros2_ws/TEST/test_leds.py --mode off
```

Override the pins or blink period when necessary:

```bash
python3 /root/ros2_ws/TEST/test_leds.py \
  --red-pin 17 --green-pin 27 --period 0.25
```
