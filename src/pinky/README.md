# pinky

## LED Server

Start the LED service server:

```bash
ros2 run pinky led_server
```

Turn off all LEDs:

```bash
ros2 service call /set_led pinky/srv/SetLed "{command: 'fill', r: 0, g: 0, b: 0}"
```
