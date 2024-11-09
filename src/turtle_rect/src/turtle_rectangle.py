
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi

class TurtleRect:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('turtle_rectangle', anonymous=True)
        
        # Create a publisher for the turtle's velocity
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Set the rate
        self.rate = rospy.Rate(10)
        
        # Create a Twist message
        self.vel_msg = Twist()

    def move_straight(self, distance, speed=1.0):
        # Calculate time needed to move distance at given speed
        duration = abs(distance) / speed
        
        # Set linear velocity
        self.vel_msg.linear.x = speed if distance > 0 else -speed
        self.vel_msg.angular.z = 0
        
        # Get starting time
        t0 = rospy.Time.now().to_sec()
        
        # Move for calculated duration
        while rospy.Time.now().to_sec() - t0 < duration:
            self.velocity_publisher.publish(self.vel_msg)
            self.rate.sleep()
        
        # Stop moving
        self.vel_msg.linear.x = 0
        self.velocity_publisher.publish(self.vel_msg)

    def rotate(self, angle_degrees, speed=45.0):
        # Convert angle to radians
        angle_rad = angle_degrees * pi / 180.0
        
        # Calculate time needed to rotate angle at given speed
        duration = abs(angle_rad) / (speed * pi / 180.0)
        
        # Set angular velocity
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = speed * pi / 180.0 if angle_degrees > 0 else -speed * pi / 180.0
        
        # Get starting time
        t0 = rospy.Time.now().to_sec()
        
        # Rotate for calculated duration
        while rospy.Time.now().to_sec() - t0 < duration:
            self.velocity_publisher.publish(self.vel_msg)
            self.rate.sleep()
        
        # Stop rotating
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)

    def draw_rectangle(self, length=2.0, width=1.0):
        # Draw each side of the rectangle
        for _ in range(2):
            self.move_straight(length)
            self.rotate(90)
            self.move_straight(width)
            self.rotate(90)

if __name__ == '__main__':
    try:
        # Create and run the turtle controller
        turtle = TurtleRect()
        rospy.sleep(1)  # Wait for connections to establish
        
        # Draw the rectangle
        turtle.draw_rectangle()
        
    except rospy.ROSInterruptException:
        pass
