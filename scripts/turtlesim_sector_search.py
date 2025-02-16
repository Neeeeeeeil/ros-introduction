import rospy
import random
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtlesimSectorSearch:
    def __init__(self):
        rospy.init_node('turtlesim_sector_search', anonymous=True)
        
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.pose = Pose()
        self.rate = rospy.Rate(10)
    
    def update_pose(self, data):
        self.pose = data
    
    def move_to_random_start(self):
        target_x = random.uniform(2, 9)  # Avoid edges
        target_y = random.uniform(2, 9)
        
        move_cmd = Twist()
        while not rospy.is_shutdown():
            distance = math.sqrt((target_x - self.pose.x) ** 2 + (target_y - self.pose.y) ** 2)
            if distance < 0.1:
                break
            
            angle_to_goal = math.atan2(target_y - self.pose.y, target_x - self.pose.x)
            move_cmd.linear.x = min(1.5, distance)
            move_cmd.angular.z = 4 * (angle_to_goal - self.pose.theta)
            
            self.velocity_publisher.publish(move_cmd)
            self.rate.sleep()
    
    def sector_search(self, radius_increment=1.0, num_legs=6):
        move_cmd = Twist()
        radius = radius_increment
        
        for i in range(num_legs):
            # Move forward
            move_cmd.linear.x = 2.0  # Speed
            move_cmd.angular.z = 0.0
            
            start_time = rospy.Time.now().to_sec()
            while rospy.Time.now().to_sec() - start_time < radius / 2.0:  # Time to move forward
                self.velocity_publisher.publish(move_cmd)
                self.rate.sleep()
            
            # Stop before turning
            move_cmd.linear.x = 0.0
            self.velocity_publisher.publish(move_cmd)
            rospy.sleep(0.5)
            
            # Turn 120 degrees
            move_cmd.angular.z = 2.0  # Adjust speed
            start_time = rospy.Time.now().to_sec()
            while rospy.Time.now().to_sec() - start_time < (math.pi / 1.5) / 2.0:  # π/1.5 is 120 degrees
                self.velocity_publisher.publish(move_cmd)
                self.rate.sleep()
            
            # Stop turning
            move_cmd.angular.z = 0.0
            self.velocity_publisher.publish(move_cmd)
            rospy.sleep(0.5)
            
            # Increase radius for next leg
            radius += radius_increment
    
    def run(self):
        rospy.sleep(1)  # Ensure pose updates
        self.move_to_random_start()
        self.sector_search()
        rospy.loginfo("Sector search pattern complete!")
        
if __name__ == '__main__':
    try:
        search_pattern = TurtlesimSectorSearch()
        search_pattern.run()
    except rospy.ROSInterruptException:
        pass
