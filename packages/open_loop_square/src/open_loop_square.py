loop_square.py 
#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
 
class Drive_Square:
    def __init__(self):
        #Initialize global class variables
        self.cmd_msg = Twist2DStamped()

        #Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)
        
        #Initialize Pub/Subs
        self.pub = rospy.Publisher('/majdoor/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/majdoor/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        
    # robot only moves when lane following is selected on the duckiebot joystick app
    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":            
            rospy.sleep(1) # Wait for a sec for the node to be ready
            self.move_robot()
 
    # Sends zero velocities to stop the robot
    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
 
    # Spin forever but listen to message callbacks
    def run(self):
    	rospy.spin() # keeps node from exiting until node has shutdown

    def move_robot(self):
        for i in range(4):
            # Move forward
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.5   # Forward velocity
            self.cmd_msg.omega = 0.0
            self.pub.publish(self.cmd_msg)
            rospy.loginfo(f"Forward! Side {i+1}/4")
            rospy.sleep(1)  # Move forward for 1 second

            self.stop_robot()
            rospy.sleep(0.5)

            # Turn 90 degrees
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = 2.0  # Angular velocity to turn in place
            self.pub.publish(self.cmd_msg)
            rospy.loginfo("Turning 90 degrees...")
            rospy.sleep(1)  # Rotate for ~1 second (adjust for accuracy)

            self.stop_robot()
            rospy.sleep(0.5)

        self.stop_robot()
        rospy.loginfo("Finished square path!")


if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass

