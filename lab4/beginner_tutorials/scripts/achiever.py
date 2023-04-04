import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from geometry_msgs.msg import Point
import math

SPEED = 1

def vector_len(vector_like):
    return math.sqrt(sum([i**2 for i in vector_like]))

def multiply(a,vector_like):
    return [i*a for i in vector_like]

class goalAndPose():
    goal:Point = None
    pos:Pose = None

    def assign(self,goal = None,pos = None)->None:
        if goal: self.goal = goal
        if pos: self.pos = pos

    def calcDif(self):
        goal = self.goal    
        pos = self.pos
        print(goal,pos)
        if not pos or not goal: 
            return None

        position = map(float,(pos.x, pos.y))
        goal_v = map(float,(goal.x, goal.y))
        vel = tuple(g-p for g,p in zip(goal_v,position))
        vel = multiply(SPEED/vector_len(vel),vel)
        cmd_vel = Twist()
        cmd_vel.linear.x,cmd_vel.linear.y = vel
        #print(cmd_vel)
        return cmd_vel

def achiever():
    rospy.init_node('Achiever_listener',anonymous=True)

    pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)

    gP = goalAndPose()
    rospy.Subscriber('/turtle1/goal',Point,lambda point: gP.assign(goal=point))
    rospy.Subscriber('/turtle1/pose',Pose,lambda pose: gP.assign(pos=pose))
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cmd_vel = gP.calcDif()
        if cmd_vel: 
            #print("Cmd vel",cmd_vel)
            pub.publish(cmd_vel)
        rate.sleep()




if __name__ == '__main__':
    try:
        achiever()
    except rospy.ROSInterrupyException:
        pass