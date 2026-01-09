# um982_driver_ros2_odom_portautodetect
um982_driver_ros2_odom_portautodetect RTK
pip install pyserial 
python3 -m venv venv
  source venv/bin/activate
  pip install pyserial

copy src into server into ros2_ws in home directory
  colcon build
  source install/setup.bash


ros2 launch um982_driver um982.launch.py \
    ntrip_username:=youremailaddress@youremailprovider.com \   ## NO REGISTRATION REQUIRED
    publish_odom:=true \
    publish_tf:=true
