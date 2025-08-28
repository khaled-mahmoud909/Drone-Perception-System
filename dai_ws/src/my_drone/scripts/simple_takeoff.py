#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest

class TakeoffController:
    def __init__(self):
        rospy.init_node('takeoff_node', anonymous=True)

        # Member variables
        self.current_state = State()
        self.takeoff_alt = 1.0  # Takeoff altitude in meters

        # Subscribers
        self.state_sub = rospy.Subscriber("mavros/state", State, self.state_cb)

        rospy.loginfo("Waiting for MAVROS services...")
        # Wait for services to become available
        rospy.wait_for_service("mavros/cmd/arming")
        rospy.wait_for_service("mavros/set_mode")
        rospy.wait_for_service("mavros/cmd/takeoff")
        rospy.loginfo("MAVROS services available.")

        # Service clients
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self.takeoff_client = rospy.ServiceProxy("mavros/cmd/takeoff", CommandTOL)

    def state_cb(self, msg):
        """Callback for MAVROS state messages."""
        self.current_state = msg

    def run(self):
        """Main execution loop."""
        rospy.loginfo("Waiting for FCU connection...")
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.sleep(0.1)
        rospy.loginfo("FCU connected.")

        # Set mode to GUIDED
        self.set_mode("GUIDED")

        # Arm the drone
        self.arm()

        # Command takeoff
        self.takeoff()

        rospy.loginfo("Takeoff sequence complete. Spinning.")
        rospy.spin()

    def set_mode(self, mode):
        """Set the flight mode."""
        set_mode_req = SetModeRequest()
        set_mode_req.custom_mode = mode
        last_req_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.current_state.mode != mode and (rospy.Time.now() - last_req_time) > rospy.Duration(5.0):
                rospy.loginfo(f"Attempting to set mode to {mode}...")
                try:
                    if self.set_mode_client.call(set_mode_req).mode_sent:
                        rospy.loginfo(f"Mode set to {mode} successfully.")
                    last_req_time = rospy.Time.now()
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call failed: {e}")
            
            if self.current_state.mode == mode:
                rospy.loginfo(f"Mode is now {mode}.")
                break
            rospy.sleep(0.2)

    def arm(self):
        """Arm the drone."""
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True
        last_req_time = rospy.Time.now()

        while not rospy.is_shutdown() and not self.current_state.armed:
            if (rospy.Time.now() - last_req_time) > rospy.Duration(5.0):
                rospy.loginfo("Attempting to arm...")
                try:
                    if self.arming_client.call(arm_cmd).success:
                        rospy.loginfo("Vehicle armed.")
                    last_req_time = rospy.Time.now()
                except rospy.ServiceException as e:
                    rospy.logerr(f"Arming failed: {e}")

            rospy.sleep(0.2)

    def takeoff(self):
        """Command the drone to take off."""
        takeoff_cmd = CommandTOLRequest()
        takeoff_cmd.min_pitch = 0
        takeoff_cmd.yaw = 0
        takeoff_cmd.latitude = 0
        takeoff_cmd.longitude = 0
        takeoff_cmd.altitude = self.takeoff_alt

        rospy.loginfo(f"Attempting to take off to {self.takeoff_alt}m...")
        try:
            response = self.takeoff_client.call(takeoff_cmd)
            if response.success:
                rospy.loginfo("Takeoff command sent successfully.")
            else:
                rospy.logerr("Takeoff command failed.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Takeoff service call failed: {e}")


if __name__ == "__main__":
    try:
        controller = TakeoffController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
