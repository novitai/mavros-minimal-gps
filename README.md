# MAVROS Minimal GPS Bridge

Ultra-minimal GPS relay using MAVROS. **27 lines total.**

## Install
```bash
sudo apt install ros-humble-mavros
```

## Run
```bash
# Start MAVROS
ros2 launch mavros mavros.launch.py fcu_url:=udp://:14550@

# Start bridge
python3 gps_bridge.py
```

## What it does
Subscribes to `/mavros/global_position/raw/fix` → Publishes to `/gps/fix`

That's it. No BS.