
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi

class TurtleRect:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('turtle_rectangle', anonymous=True)
        
        # Publisher
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Rate
        self.rate = rospy.Rate(10)
        
        # Twist Message
        self.vel_msg = Twist()

    def move_straight(self, distance, speed=1.0):
        # Time needed to move distance at given speed
        duration = abs(distance) / speed
        
        # Linear velocity
        self.vel_msg.linear.x = speed if distance > 0 else -speed
        self.vel_msg.angular.z = 0
        
        # Starting time
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
        
        # Angular velocity
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = speed * pi / 180.0 if angle_degrees > 0 else -speed * pi / 180.0
        
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
        # Creating and running the turtle controller
        turtle = TurtleRect()
        rospy.sleep(1)  # Waiting for connections to establish
        
        # Drawing the rectangle
        turtle.draw_rectangle()

    # Handling any sorts of Exceptions
    except rospy.ROSInterruptException:
        pass
