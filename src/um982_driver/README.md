# UM982 ROS2 Driver

ROS2 driver for Unicorecomm UM982 dual-antenna RTK GNSS receiver with:
- Integrated NTRIP client
- Odometry output (GPS → local coordinates)
- TF broadcasting (odom → base_link)
- **Auto-detection** of serial port

## Installation

```bash
cd ~/um982_ws/src
tar -xzf um982_driver_ros2_v3_autodetect.tar.gz

# Install dependencies
pip install pyserial --break-system-packages

# Build
cd ~/um982_ws
colcon build
source install/setup.bash
```

## Usage

### Auto-detect (default)
```bash
ros2 launch um982_driver um982.launch.py \
    ntrip_username:=your@email.com
```

### Manual port
```bash
ros2 launch um982_driver um982.launch.py \
    serial_port:=/dev/ttyUSB1 \
    ntrip_username:=your@email.com
```

### With udev rule (recommended for persistent name)
```bash
sudo cp udev/99-um982.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
# Reconnect device, then use:
ros2 launch um982_driver um982.launch.py \
    serial_port:=/dev/um982 \
    ntrip_username:=your@email.com
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/gnss/fix` | NavSatFix | GPS position (lat/lon/alt) |
| `/gnss/heading` | Float64 | Heading in degrees (GNSS: 0°=North, CW) |
| `/gnss/heading_quaternion` | QuaternionStamped | Heading as quaternion (ROS convention) |
| `/gnss/vel` | TwistStamped | Velocity |
| `/odom` | Odometry | Local position (x,y) + orientation |

## TF Frames

```
odom → base_link
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `auto` | Serial port or 'auto' for auto-detection |
| `baud_rate` | `115200` | Baud rate |
| `ntrip_enabled` | `true` | Enable NTRIP client |
| `ntrip_host` | `rtk2go.com` | NTRIP caster host |
| `ntrip_port` | `2101` | NTRIP caster port |
| `ntrip_mountpoint` | `DEU00WOLF0` | NTRIP mountpoint |
| `ntrip_username` | `` | NTRIP username (email for rtk2go) |
| `ntrip_password` | `none` | NTRIP password |
| `heading_offset` | `0.0` | Heading offset in degrees |
| `publish_tf` | `true` | Publish TF odom→base_link |
| `publish_odom` | `true` | Publish /odom topic |
| `odom_frame_id` | `odom` | Odometry frame ID |
| `base_frame_id` | `base_link` | Base link frame ID |

## Antenna Configuration

Default: ANT1 (Master) hinten, ANT2 (Slave) vorne → heading_offset = 0°

```
Fahrtrichtung →
[ANT1]----1m----[ANT2]
```

If antennas are swapped: `heading_offset:=180.0`
