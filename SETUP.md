# 🎯 MAVROS GPS Bridge - Complete Setup Guide

## When To Use This
✅ You already have MAVROS running  
✅ You want GPS on a different topic  
✅ You need the simplest possible solution  

## Prerequisites
```bash
# Check MAVROS is installed
ros2 pkg list | grep mavros
# Should show mavros and mavros_msgs
```

## 1. Start MAVROS First

### For Network Connection (UDP)
```bash
ros2 launch mavros mavros.launch.py fcu_url:=udp://:14550@
```

### For Serial Connection (Pixhawk USB)
```bash
# Find your device
ls /dev/ttyACM* /dev/ttyUSB*

# Launch MAVROS (change ttyACM0 to your device)
ros2 launch mavros mavros.launch.py fcu_url:=serial:///dev/ttyACM0:57600
```

### Verify MAVROS is Getting GPS
```bash
# Check if GPS data is coming
ros2 topic echo /mavros/global_position/raw/fix --once
```

If you see coordinates, MAVROS is working! If not, check your connection.

## 2. Run GPS Bridge

### Option A: Direct Python
```bash
# Clone this repo
git clone https://github.com/novitai/mavros-minimal-gps.git
cd mavros-minimal-gps

# Run the bridge
python3 gps_bridge.py
```

### Option B: Docker
```bash
docker run --rm --network host ghcr.io/novitai/mavros-minimal-gps:latest
```

## 3. Verify Output
```bash
# You should now have GPS on /gps/fix
ros2 topic echo /gps/fix
```

## Complete Example (Jetson + Pixhawk)

```bash
# Terminal 1: Start MAVROS
ros2 launch mavros mavros.launch.py fcu_url:=serial:///dev/ttyACM0:921600

# Terminal 2: Wait for GPS lock
ros2 topic echo /mavros/global_position/raw/fix --once
# Make sure you see valid coordinates

# Terminal 3: Run bridge
python3 gps_bridge.py

# Terminal 4: Verify new topic
ros2 topic hz /gps/fix
# Should show ~5-10 Hz
```

## Foxglove Setup

```bash
# Terminal 1: Install and run Foxglove bridge
sudo apt install ros-humble-foxglove-bridge
ros2 run foxglove_bridge foxglove_bridge

# Terminal 2: Your GPS bridge should be running
python3 gps_bridge.py
```

In Foxglove Studio:
1. Connect to `ws://YOUR_IP:8765`
2. Add Map panel
3. Select `/gps/fix` topic
4. See your position!

## Custom Topic Names

Want different topic names? Modify `gps_bridge.py`:

```python
# Change this line:
self.pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
# To whatever you want:
self.pub = self.create_publisher(NavSatFix, '/my_robot/gps', 10)
```

## Multiple Vehicles

For multiple drones/robots:

```bash
# Drone 1
ros2 launch mavros mavros.launch.py fcu_url:=udp://:14550@ mavros_ns:=/drone1/mavros
python3 gps_bridge.py --ros-args -r __ns:=/drone1

# Drone 2
ros2 launch mavros mavros.launch.py fcu_url:=udp://:14551@ mavros_ns:=/drone2/mavros
python3 gps_bridge.py --ros-args -r __ns:=/drone2
```

## Troubleshooting

### No GPS Data from MAVROS?
```bash
# Check MAVLink connection
ros2 topic list | grep mavros
# Should show many topics

# Check GPS specifically
ros2 topic echo /mavros/global_position/raw/fix
# If empty, flight controller isn't sending GPS
```

### Bridge Running but No Output?
```bash
# Check both topics exist
ros2 topic list | grep -E "(mavros|gps)"
# Should show both /mavros/global_position/raw/fix and /gps/fix
```

### Permission Issues?
```bash
# Add to dialout group
sudo usermod -aG dialout $USER
# Logout and login
```

## Auto-Start Everything

Create `/etc/systemd/system/gps-system.service`:

```ini
[Unit]
Description=GPS System (MAVROS + Bridge)
After=network.target

[Service]
Type=simple
User=YOUR_USER
ExecStart=/bin/bash -c 'ros2 launch mavros mavros.launch.py fcu_url:=serial:///dev/ttyACM0:921600 & sleep 5 && python3 /home/YOUR_USER/mavros-minimal-gps/gps_bridge.py'
Restart=always

[Install]
WantedBy=multi-user.target
```

Enable:
```bash
sudo systemctl enable gps-system
sudo systemctl start gps-system
```

## Done! 🚀

GPS data should now flow:  
**Flight Controller → MAVROS → GPS Bridge → Your Application**