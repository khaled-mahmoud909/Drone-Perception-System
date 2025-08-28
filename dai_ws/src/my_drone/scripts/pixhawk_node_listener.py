#!/usr/bin/env python3

import rospy
import time
import math
from pymavlink import mavutil
from threading import Lock

# Import standard ROS message types
from sensor_msgs.msg import Imu, BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, String, Bool # Import Bool for armed status
from geometry_msgs.msg import Quaternion, Vector3

# Helper function to request a message interval
def set_message_interval(master, message_id, frequency_hz):
    """
    Requests a MAVLink message to be sent at a specific frequency.
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,  # Confirmation
        message_id,
        int(1e6 / frequency_hz) if frequency_hz > 0 else -1, # Interval in microseconds
        0, 0, 0, 0, 0  # Unused params
    )
    rospy.loginfo(f"Requested message {message_id} at {frequency_hz} Hz")
    time.sleep(0.1)

def mavlink_to_ros_publisher():
    rospy.init_node('mavlink_telemetry_publisher', anonymous=True)

    # --- Parameters ---
    connection_string = rospy.get_param('~connection_string', '/dev/ttyAMA0')
    baud_rate = rospy.get_param('~baud', 921600)

    # --- Create ROS Publishers ---
    imu_pub = rospy.Publisher('/fc/imu', Imu, queue_size=10)
    odom_pub = rospy.Publisher('/fc/odom', Odometry, queue_size=10)
    battery_pub = rospy.Publisher('/fc/battery', BatteryState, queue_size=10)
    flight_mode_pub = rospy.Publisher('/fc/flight_mode', String, queue_size=10)
    # --- NEW: Publisher for armed status ---
    armed_pub = rospy.Publisher('/fc/armed_status', Bool, queue_size=10)
    
    rospy.loginfo("Created ROS publishers on /fc/* topics.")

    # --- MAVLink Connection ---
    rospy.loginfo(f"Connecting to MAVLink at {connection_string}...")
    try:
        master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
        master.wait_heartbeat()
        rospy.loginfo("MAVLink Heartbeat received!")
    except Exception as e:
        rospy.logerr(f"Failed to connect to MAVLink: {e}")
        return

    # --- Request Data Streams ---
    rospy.loginfo("Requesting MAVLink message intervals...")
    set_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2)
    set_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 20)
    set_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20)
    # Heartbeat is sent automatically, no need to request
    rospy.loginfo("Message intervals requested.")

    # --- State storage ---
    last_pose = {'position': Vector3(), 'orientation': Quaternion(w=1.0)}
    last_twist = {'linear': Vector3(), 'angular': Vector3()}
    state_lock = Lock()

    # --- Main Loop ---
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        msg = master.recv_match(blocking=False)
        if not msg:
            continue

        msg_type = msg.get_type()
        now = rospy.Time.now()
        
        publish_odom = False

        with state_lock:
            if msg_type == 'ATTITUDE_QUATERNION':
                last_pose['orientation'] = Quaternion(w=msg.q1, x=msg.q2, y=msg.q3, z=msg.q4)
                last_twist['angular'] = Vector3(x=msg.rollspeed, y=msg.pitchspeed, z=msg.yawspeed)
                
                # --- FIX: Create a proper Header object ---
                header = Header(stamp=now, frame_id='base_link')
                imu_msg = Imu(header=header, orientation=last_pose['orientation'], angular_velocity=last_twist['angular'])
                imu_pub.publish(imu_msg)
                
                publish_odom = True

            elif msg_type == 'LOCAL_POSITION_NED':
                last_pose['position'] = Vector3(x=msg.y, y=msg.x, z=-msg.z)
                last_twist['linear'] = Vector3(x=msg.vy, y=msg.vx, z=-msg.vz)
                publish_odom = True

            elif msg_type == 'SYS_STATUS':
                # --- FIX: Create a proper Header object ---
                header = Header(stamp=now) # frame_id is not typical for BatteryState
                battery_msg = BatteryState(header=header, voltage=msg.voltage_battery/1000.0, current=msg.current_battery/100.0, percentage=msg.battery_remaining/100.0, present=True)
                battery_pub.publish(battery_msg)

            elif msg_type == 'HEARTBEAT':
                flight_mode = mavutil.mode_string_v10(msg)
                flight_mode_pub.publish(String(data=flight_mode))
                is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                armed_pub.publish(Bool(data=is_armed))

            if publish_odom:
                # --- FIX: Create a proper Header object ---
                header = Header(stamp=now, frame_id='odom')
                odom_msg = Odometry(header=header, child_frame_id='base_link')
                
                odom_msg.pose.pose.position = last_pose['position']
                odom_msg.pose.pose.orientation = last_pose['orientation']
                odom_msg.twist.twist.linear = last_twist['linear']
                odom_msg.twist.twist.angular = last_twist['angular']
                odom_pub.publish(odom_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        mavlink_to_ros_publisher()
    except rospy.ROSInterruptException:
        pass
