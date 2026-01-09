#!/usr/bin/env python3
"""
UM982 ROS2 Driver with integrated NTRIP Client and Odometry Output
- Receives RTCM corrections via NTRIP
- Publishes NMEA data as ROS2 topics
- Transforms GPS to local odometry frame (UTM projection)
- Broadcasts TF: odom -> base_link
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped, QuaternionStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Float64
from tf2_ros import TransformBroadcaster

import serial
import socket
import base64
import threading
import math
import time
from typing import Optional


class UTMProjection:
    """Simple UTM projection for GPS -> local coordinates"""
    
    def __init__(self):
        self.origin_lat = None
        self.origin_lon = None
        self.origin_x = None
        self.origin_y = None
        self.zone = None
        self.initialized = False
        
    def set_origin(self, lat: float, lon: float):
        """Set the origin point (first GPS fix)"""
        self.origin_lat = lat
        self.origin_lon = lon
        self.zone = int((lon + 180) / 6) + 1
        self.origin_x, self.origin_y = self._latlon_to_utm(lat, lon)
        self.initialized = True
        
    def _latlon_to_utm(self, lat: float, lon: float) -> tuple:
        """Convert lat/lon to UTM coordinates"""
        # WGS84 parameters
        a = 6378137.0  # semi-major axis
        f = 1 / 298.257223563  # flattening
        k0 = 0.9996  # scale factor
        
        # Calculate e^2
        e2 = 2 * f - f * f
        e_prime2 = e2 / (1 - e2)
        
        # Convert to radians
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        
        # Central meridian
        lon0 = math.radians((self.zone - 1) * 6 - 180 + 3)
        
        # Calculate N, T, C, A, M
        N = a / math.sqrt(1 - e2 * math.sin(lat_rad) ** 2)
        T = math.tan(lat_rad) ** 2
        C = e_prime2 * math.cos(lat_rad) ** 2
        A = (lon_rad - lon0) * math.cos(lat_rad)
        
        M = a * (
            (1 - e2/4 - 3*e2**2/64 - 5*e2**3/256) * lat_rad
            - (3*e2/8 + 3*e2**2/32 + 45*e2**3/1024) * math.sin(2*lat_rad)
            + (15*e2**2/256 + 45*e2**3/1024) * math.sin(4*lat_rad)
            - (35*e2**3/3072) * math.sin(6*lat_rad)
        )
        
        # Calculate UTM coordinates
        x = k0 * N * (
            A + (1 - T + C) * A**3 / 6
            + (5 - 18*T + T**2 + 72*C - 58*e_prime2) * A**5 / 120
        ) + 500000  # False easting
        
        y = k0 * (
            M + N * math.tan(lat_rad) * (
                A**2 / 2
                + (5 - T + 9*C + 4*C**2) * A**4 / 24
                + (61 - 58*T + T**2 + 600*C - 330*e_prime2) * A**6 / 720
            )
        )
        
        # False northing for southern hemisphere
        if lat < 0:
            y += 10000000
            
        return x, y
    
    def to_local(self, lat: float, lon: float) -> tuple:
        """Convert lat/lon to local coordinates relative to origin"""
        if not self.initialized:
            return 0.0, 0.0
            
        x, y = self._latlon_to_utm(lat, lon)
        return x - self.origin_x, y - self.origin_y


class NTRIPClient:
    """Simple NTRIP Client for receiving RTCM corrections"""
    
    def __init__(self, host: str, port: int, mountpoint: str, 
                 username: str, password: str, logger):
        self.host = host
        self.port = port
        self.mountpoint = mountpoint
        self.username = username
        self.password = password
        self.logger = logger
        self.socket: Optional[socket.socket] = None
        self.connected = False
        
    def connect(self) -> bool:
        """Connect to NTRIP caster"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10)
            self.socket.connect((self.host, self.port))
            
            # Build NTRIP request
            credentials = base64.b64encode(
                f"{self.username}:{self.password}".encode()
            ).decode()
            
            request = (
                f"GET /{self.mountpoint} HTTP/1.0\r\n"
                f"User-Agent: NTRIP UM982ROS2Client/1.0\r\n"
                f"Authorization: Basic {credentials}\r\n"
                f"\r\n"
            )
            
            self.socket.send(request.encode())
            
            # Read response
            response = self.socket.recv(1024).decode('latin-1')
            
            if "ICY 200 OK" in response or "HTTP/1.0 200 OK" in response or "HTTP/1.1 200 OK" in response:
                self.connected = True
                self.socket.settimeout(5)
                self.logger.info(f"NTRIP connected to {self.host}:{self.port}/{self.mountpoint}")
                return True
            else:
                self.logger.error(f"NTRIP connection failed: {response[:100]}")
                return False
                
        except Exception as e:
            self.logger.error(f"NTRIP connection error: {e}")
            return False
    
    def read_rtcm(self) -> Optional[bytes]:
        """Read RTCM data from NTRIP caster"""
        if not self.connected or not self.socket:
            return None
        try:
            data = self.socket.recv(4096)
            return data if data else None
        except socket.timeout:
            return None
        except Exception as e:
            self.logger.warn(f"NTRIP read error: {e}")
            self.connected = False
            return None
    
    def disconnect(self):
        """Disconnect from NTRIP caster"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.connected = False


class UM982Driver(Node):
    """ROS2 Node for UM982 GNSS Receiver with NTRIP support and Odometry output"""
    
    def __init__(self):
        super().__init__('um982_driver')
        
        # Declare parameters
        self.declare_parameter('serial_port', 'auto')  # 'auto' for auto-detection
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'gnss_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        
        # NTRIP parameters
        self.declare_parameter('ntrip_host', 'rtk2go.com')
        self.declare_parameter('ntrip_port', 2101)
        self.declare_parameter('ntrip_mountpoint', 'DEU00WOLF0')
        self.declare_parameter('ntrip_username', '')
        self.declare_parameter('ntrip_password', 'none')
        self.declare_parameter('ntrip_enabled', True)
        
        # Heading parameters
        self.declare_parameter('heading_offset', 0.0)  # Offset in degrees
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_odom', True)
        
        # Get parameters
        serial_port_param = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        
        # Auto-detect serial port if set to 'auto'
        if serial_port_param == 'auto':
            self.serial_port = self._auto_detect_port()
            if not self.serial_port:
                self.get_logger().error("Could not auto-detect UM982. Please specify serial_port manually.")
                raise RuntimeError("UM982 not found")
        else:
            self.serial_port = serial_port_param
        self.frame_id = self.get_parameter('frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        
        self.ntrip_host = self.get_parameter('ntrip_host').value
        self.ntrip_port = self.get_parameter('ntrip_port').value
        self.ntrip_mountpoint = self.get_parameter('ntrip_mountpoint').value
        self.ntrip_username = self.get_parameter('ntrip_username').value
        self.ntrip_password = self.get_parameter('ntrip_password').value
        self.ntrip_enabled = self.get_parameter('ntrip_enabled').value
        
        self.heading_offset = self.get_parameter('heading_offset').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.publish_odom = self.get_parameter('publish_odom').value
        
        # Publishers
        self.fix_pub = self.create_publisher(NavSatFix, 'gnss/fix', 10)
        self.heading_pub = self.create_publisher(Float64, 'gnss/heading', 10)
        self.heading_quat_pub = self.create_publisher(QuaternionStamped, 'gnss/heading_quaternion', 10)
        self.vel_pub = self.create_publisher(TwistStamped, 'gnss/vel', 10)
        
        # Odometry publisher
        if self.publish_odom:
            self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # TF Broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # UTM Projection for GPS -> local
        self.utm = UTMProjection()
        
        # Serial connection
        self.serial: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()
        
        # NTRIP client
        self.ntrip_client: Optional[NTRIPClient] = None
        
        # Threads
        self.running = True
        self.nmea_thread: Optional[threading.Thread] = None
        self.ntrip_thread: Optional[threading.Thread] = None
        
        # State
        self.last_heading = None  # degrees, GNSS convention (0=North, CW)
        self.last_heading_quat = None
        self.last_velocity = (0.0, 0.0, 0.0)  # vx, vy, vz in local frame
        
        # Statistics
        self.rtcm_bytes_received = 0
        self.nmea_sentences_parsed = 0
        self.last_fix_quality = 0
        self.last_hdop = 99.0
        
        # Start
        self._connect_serial()
        if self.ntrip_enabled:
            self._start_ntrip()
        self._start_nmea_reader()
        
        # Status timer
        self.create_timer(10.0, self._print_status)
        
        self.get_logger().info(f"UM982 Driver started on {self.serial_port}")
        self.get_logger().info(f"Odometry: {self.publish_odom}, TF: {self.publish_tf}")
    
    def _connect_serial(self):
        """Connect to serial port"""
        try:
            self.serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0
            )
            self.get_logger().info(f"Serial connected: {self.serial_port} @ {self.baud_rate}")
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            raise
    
    def _auto_detect_port(self) -> Optional[str]:
        """Auto-detect UM982 on available serial ports"""
        import glob
        
        # Possible port patterns
        port_patterns = [
            '/dev/ttyUSB*',
            '/dev/ttyACM*',
            '/dev/serial/by-id/*'
        ]
        
        ports = []
        for pattern in port_patterns:
            ports.extend(glob.glob(pattern))
        
        if not ports:
            self.get_logger().warn("No serial ports found")
            return None
        
        self.get_logger().info(f"Scanning ports: {ports}")
        
        for port in sorted(ports):
            try:
                self.get_logger().info(f"Trying {port}...")
                
                # Try to open port
                test_serial = serial.Serial(
                    port=port,
                    baudrate=self.baud_rate,
                    timeout=2.0
                )
                
                # Clear buffer
                test_serial.reset_input_buffer()
                
                # Wait for data
                time.sleep(0.5)
                
                # Read available data
                nmea_found = False
                start_time = time.time()
                
                while time.time() - start_time < 3.0:  # 3 second timeout
                    if test_serial.in_waiting:
                        data = test_serial.read(test_serial.in_waiting)
                        try:
                            text = data.decode('ascii', errors='ignore')
                            # Check for NMEA sentences (typical for GNSS)
                            if '$GN' in text or '$GP' in text or '$GB' in text:
                                nmea_found = True
                                # Check specifically for UM982 indicators
                                if 'GGA' in text or 'RMC' in text or 'THS' in text:
                                    self.get_logger().info(f"UM982 detected on {port}")
                                    test_serial.close()
                                    return port
                        except:
                            pass
                    time.sleep(0.1)
                
                test_serial.close()
                
                if nmea_found:
                    self.get_logger().info(f"GNSS device found on {port} (assuming UM982)")
                    return port
                    
            except serial.SerialException as e:
                self.get_logger().debug(f"Could not open {port}: {e}")
                continue
            except Exception as e:
                self.get_logger().debug(f"Error testing {port}: {e}")
                continue
        
        return None
    
    def _start_ntrip(self):
        """Start NTRIP client thread"""
        self.ntrip_client = NTRIPClient(
            host=self.ntrip_host,
            port=self.ntrip_port,
            mountpoint=self.ntrip_mountpoint,
            username=self.ntrip_username,
            password=self.ntrip_password,
            logger=self.get_logger()
        )
        
        self.ntrip_thread = threading.Thread(target=self._ntrip_loop, daemon=True)
        self.ntrip_thread.start()
    
    def _ntrip_loop(self):
        """NTRIP client loop - receives RTCM and sends to receiver"""
        reconnect_delay = 5
        
        while self.running:
            # Connect if not connected
            if not self.ntrip_client.connected:
                self.get_logger().info("Connecting to NTRIP...")
                if not self.ntrip_client.connect():
                    time.sleep(reconnect_delay)
                    continue
            
            # Read RTCM data
            rtcm_data = self.ntrip_client.read_rtcm()
            
            if rtcm_data:
                self.rtcm_bytes_received += len(rtcm_data)
                # Send RTCM to receiver
                with self.serial_lock:
                    if self.serial and self.serial.is_open:
                        try:
                            self.serial.write(rtcm_data)
                        except Exception as e:
                            self.get_logger().warn(f"Failed to write RTCM: {e}")
            
            time.sleep(0.01)  # Small delay to prevent busy loop
    
    def _start_nmea_reader(self):
        """Start NMEA reader thread"""
        self.nmea_thread = threading.Thread(target=self._nmea_loop, daemon=True)
        self.nmea_thread.start()
    
    def _nmea_loop(self):
        """NMEA reader loop - reads and parses NMEA sentences"""
        buffer = ""
        
        while self.running:
            try:
                with self.serial_lock:
                    if self.serial and self.serial.is_open and self.serial.in_waiting:
                        data = self.serial.read(self.serial.in_waiting)
                    else:
                        data = None
                
                if data:
                    # Filter out non-ASCII bytes (RTCM responses)
                    try:
                        text = data.decode('ascii', errors='ignore')
                        buffer += text
                    except:
                        continue
                    
                    # Process complete sentences
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line.startswith('$'):
                            self._parse_nmea(line)
                
                time.sleep(0.01)
                
            except Exception as e:
                self.get_logger().warn(f"NMEA read error: {e}")
                time.sleep(0.1)
    
    def _parse_nmea(self, sentence: str):
        """Parse NMEA sentence and publish ROS2 messages"""
        if not self._verify_checksum(sentence):
            return
        
        self.nmea_sentences_parsed += 1
        
        try:
            if 'GGA' in sentence:
                self._parse_gga(sentence)
            elif 'RMC' in sentence:
                self._parse_rmc(sentence)
            elif 'THS' in sentence or 'HDT' in sentence:
                self._parse_heading(sentence)
        except Exception as e:
            self.get_logger().debug(f"Parse error for {sentence[:20]}: {e}")
    
    def _verify_checksum(self, sentence: str) -> bool:
        """Verify NMEA checksum"""
        if '*' not in sentence:
            return False
        
        try:
            data, checksum = sentence.rsplit('*', 1)
            data = data[1:]  # Remove leading $
            
            calc_checksum = 0
            for char in data:
                calc_checksum ^= ord(char)
            
            return calc_checksum == int(checksum[:2], 16)
        except:
            return False
    
    def _gnss_heading_to_ros_yaw(self, heading_deg: float) -> float:
        """
        Convert GNSS heading to ROS yaw
        GNSS: 0° = North, clockwise positive
        ROS:  0° = East (X-axis), counter-clockwise positive
        
        Formula: ros_yaw = 90° - gnss_heading
        """
        ros_yaw = math.radians(90.0 - heading_deg)
        # Normalize to [-pi, pi]
        while ros_yaw > math.pi:
            ros_yaw -= 2 * math.pi
        while ros_yaw < -math.pi:
            ros_yaw += 2 * math.pi
        return ros_yaw
    
    def _yaw_to_quaternion(self, yaw: float) -> tuple:
        """Convert yaw angle to quaternion (x, y, z, w)"""
        return (
            0.0,
            0.0,
            math.sin(yaw / 2),
            math.cos(yaw / 2)
        )
    
    def _parse_gga(self, sentence: str):
        """Parse GGA sentence and publish NavSatFix + Odometry"""
        parts = sentence.split(',')
        
        if len(parts) < 15:
            return
        
        try:
            # Parse fields
            lat_raw = parts[2]
            lat_dir = parts[3]
            lon_raw = parts[4]
            lon_dir = parts[5]
            quality = int(parts[6]) if parts[6] else 0
            num_sats = int(parts[7]) if parts[7] else 0
            hdop = float(parts[8]) if parts[8] else 99.0
            altitude = float(parts[9]) if parts[9] else 0.0
            
            if not lat_raw or not lon_raw:
                return
            
            # Convert coordinates
            latitude = self._nmea_to_decimal(lat_raw, lat_dir)
            longitude = self._nmea_to_decimal(lon_raw, lon_dir)
            
            self.last_fix_quality = quality
            self.last_hdop = hdop
            
            now = self.get_clock().now().to_msg()
            
            # ========== NavSatFix ==========
            fix_msg = NavSatFix()
            fix_msg.header = Header()
            fix_msg.header.stamp = now
            fix_msg.header.frame_id = self.frame_id
            
            fix_msg.latitude = latitude
            fix_msg.longitude = longitude
            fix_msg.altitude = altitude
            
            # Set status based on quality
            fix_msg.status.service = NavSatStatus.SERVICE_GPS
            if quality == 0:
                fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
            elif quality == 1:
                fix_msg.status.status = NavSatStatus.STATUS_FIX
            elif quality == 2:
                fix_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
            elif quality in [4, 5]:
                fix_msg.status.status = NavSatStatus.STATUS_GBAS_FIX  # RTK
            else:
                fix_msg.status.status = NavSatStatus.STATUS_FIX
            
            # Covariance based on quality and HDOP
            if quality == 4:  # RTK Fixed
                pos_std = 0.02  # 2 cm
            elif quality == 5:  # RTK Float
                pos_std = 0.10  # 10 cm
            else:
                pos_std = hdop * 2.5  # Rough estimate
            
            position_cov = pos_std ** 2
            fix_msg.position_covariance = [
                position_cov, 0.0, 0.0,
                0.0, position_cov, 0.0,
                0.0, 0.0, position_cov * 2
            ]
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            
            self.fix_pub.publish(fix_msg)
            
            # ========== Odometry ==========
            if self.publish_odom and quality > 0:
                # Initialize origin on first fix
                if not self.utm.initialized:
                    self.utm.set_origin(latitude, longitude)
                    self.get_logger().info(
                        f"Origin set: {latitude:.7f}, {longitude:.7f} (UTM Zone {self.utm.zone})"
                    )
                
                # Convert to local coordinates
                x, y = self.utm.to_local(latitude, longitude)
                
                odom_msg = Odometry()
                odom_msg.header.stamp = now
                odom_msg.header.frame_id = self.odom_frame_id
                odom_msg.child_frame_id = self.base_frame_id
                
                # Position
                odom_msg.pose.pose.position.x = x
                odom_msg.pose.pose.position.y = y
                odom_msg.pose.pose.position.z = 0.0
                
                # Orientation from dual-antenna heading
                if self.last_heading is not None:
                    ros_yaw = self._gnss_heading_to_ros_yaw(self.last_heading)
                    qx, qy, qz, qw = self._yaw_to_quaternion(ros_yaw)
                    odom_msg.pose.pose.orientation.x = qx
                    odom_msg.pose.pose.orientation.y = qy
                    odom_msg.pose.pose.orientation.z = qz
                    odom_msg.pose.pose.orientation.w = qw
                else:
                    odom_msg.pose.pose.orientation.w = 1.0
                
                # Covariance
                odom_msg.pose.covariance[0] = position_cov   # x
                odom_msg.pose.covariance[7] = position_cov   # y
                odom_msg.pose.covariance[14] = position_cov * 2  # z
                odom_msg.pose.covariance[35] = 0.01 if self.last_heading else 1.0  # yaw
                
                # Velocity
                odom_msg.twist.twist.linear.x = self.last_velocity[0]
                odom_msg.twist.twist.linear.y = self.last_velocity[1]
                odom_msg.twist.twist.linear.z = self.last_velocity[2]
                
                self.odom_pub.publish(odom_msg)
                
                # ========== TF Broadcast ==========
                if self.publish_tf:
                    t = TransformStamped()
                    t.header.stamp = now
                    t.header.frame_id = self.odom_frame_id
                    t.child_frame_id = self.base_frame_id
                    
                    t.transform.translation.x = x
                    t.transform.translation.y = y
                    t.transform.translation.z = 0.0
                    
                    t.transform.rotation = odom_msg.pose.pose.orientation
                    
                    self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().debug(f"GGA parse error: {e}")
    
    def _parse_rmc(self, sentence: str):
        """Parse RMC sentence and publish velocity"""
        parts = sentence.split(',')
        
        if len(parts) < 8:
            return
        
        try:
            speed_knots = float(parts[7]) if parts[7] else 0.0
            course = float(parts[8]) if len(parts) > 8 and parts[8] else 0.0
            
            # Convert to m/s
            speed_ms = speed_knots * 0.514444
            
            # Velocity in body frame (forward = x)
            # For stationary robot with heading, velocity is in heading direction
            self.last_velocity = (speed_ms, 0.0, 0.0)  # Forward velocity only
            
            # Create TwistStamped
            msg = TwistStamped()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame_id
            
            msg.twist.linear.x = speed_ms  # Forward
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0
            
            self.vel_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().debug(f"RMC parse error: {e}")
    
    def _parse_heading(self, sentence: str):
        """Parse THS/HDT sentence and publish heading"""
        parts = sentence.split(',')
        
        if len(parts) < 2:
            return
        
        try:
            # THS format: $GNTHS,123.45,A*XX
            heading_str = parts[1]
            if not heading_str:
                return
                
            heading = float(heading_str)
            
            # Check validity (A = valid, V = invalid)
            valid = True
            if len(parts) > 2 and '*' in parts[2]:
                valid_char = parts[2].split('*')[0]
                valid = valid_char == 'A'
            
            if not valid:
                return
            
            # Apply offset and normalize
            heading = (heading + self.heading_offset) % 360.0
            self.last_heading = heading
            
            now = self.get_clock().now().to_msg()
            
            # Publish heading as Float64 (degrees, GNSS convention)
            heading_msg = Float64()
            heading_msg.data = heading
            self.heading_pub.publish(heading_msg)
            
            # Publish as QuaternionStamped (ROS convention)
            quat_msg = QuaternionStamped()
            quat_msg.header = Header()
            quat_msg.header.stamp = now
            quat_msg.header.frame_id = self.frame_id
            
            # Convert to ROS yaw and then to quaternion
            ros_yaw = self._gnss_heading_to_ros_yaw(heading)
            qx, qy, qz, qw = self._yaw_to_quaternion(ros_yaw)
            
            quat_msg.quaternion.x = qx
            quat_msg.quaternion.y = qy
            quat_msg.quaternion.z = qz
            quat_msg.quaternion.w = qw
            
            self.last_heading_quat = (qx, qy, qz, qw)
            self.heading_quat_pub.publish(quat_msg)
            
        except Exception as e:
            self.get_logger().debug(f"Heading parse error: {e}")
    
    def _nmea_to_decimal(self, coord: str, direction: str) -> float:
        """Convert NMEA coordinate to decimal degrees"""
        if not coord:
            return 0.0
        
        # NMEA format: DDDMM.MMMM or DDMM.MMMM
        if '.' in coord:
            point_pos = coord.index('.')
            degrees = int(coord[:point_pos - 2])
            minutes = float(coord[point_pos - 2:])
        else:
            return 0.0
        
        decimal = degrees + minutes / 60.0
        
        if direction in ['S', 'W']:
            decimal = -decimal
        
        return decimal
    
    def _print_status(self):
        """Print status information"""
        quality_names = {
            0: "No Fix",
            1: "GPS",
            2: "DGPS",
            4: "RTK Fixed",
            5: "RTK Float"
        }
        quality_str = quality_names.get(self.last_fix_quality, f"Unknown({self.last_fix_quality})")
        
        ntrip_status = "Connected" if (self.ntrip_client and self.ntrip_client.connected) else "Disconnected"
        
        heading_str = f"{self.last_heading:.1f}°" if self.last_heading else "N/A"
        
        self.get_logger().info(
            f"Quality={quality_str}, Heading={heading_str}, "
            f"NTRIP={ntrip_status}, RTCM={self.rtcm_bytes_received/1024:.1f}KB"
        )
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        self.running = False
        
        if self.ntrip_client:
            self.ntrip_client.disconnect()
        
        if self.serial and self.serial.is_open:
            self.serial.close()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = UM982Driver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
