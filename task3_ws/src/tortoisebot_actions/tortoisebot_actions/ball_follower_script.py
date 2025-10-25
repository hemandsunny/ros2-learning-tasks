import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import isinf,isnan,pi,degrees,exp


'''
Twist msg type
Vector3  linear
	float64 x
	float64 y
	float64 z
Vector3  angular
	float64 x
	float64 y
	float64 z
'''

class BallFollowNode(Node):

    def __init__(self):
        super().__init__('ball_follow_node') # node name
        self.subscription=self.create_subscription(
            LaserScan,
            'scan_filtered',
            self.listener_callback,
            10) # subscribing to /scan
        self.publisher = self.create_publisher(Twist,'cmd_vel',10) #publishing to /closest_object_distance

        self.msgData=None
        self.safeDistanceObject=0.7
        
        
        self.get_logger().warn('[DEBUG][BALL_FOLLOW_NODE] STARTED')
        self.get_logger().info('[DEBUG][BALL_FOLLOW_NODE] SUBSCRIBED TO /scan_filtered')
        self.get_logger().info('[DEBUG][BALL_FOLLOW_NODE] Publishing to /cmd_vel')
        

        
    



    def findClosestObjectDirection(self,listVar,angMin,angMax,angInc):
        closestValue=min(listVar)
        closestIndex=listVar.index(closestValue)
        targetDir=angMin+closestIndex*angInc
        
        if closestValue==float('inf'):
            closestValue=self.safeDistanceObject
            
        return [targetDir,closestValue]
    
    
    def moveRobot(self,angularTarget=0.0,LinearTarget=0.0):
        (angularVel,linearVel)=(0.0,0.0)
        distDiff=LinearTarget-self.safeDistanceObject
        if angularTarget!=0.0:
             angularVel=0.5*(angularTarget)
        if distDiff>0.025 and -0.25<angularTarget<0.25:
            linearVel=1*distDiff
        if distDiff<0.0:# and -0.25<angularTarget<0.25:
            linearVel=0.25*distDiff
        if distDiff==0.0:
            linearVel=0.0

            
        # VELOCITY LIMITER
        linearVel=min(0.5,linearVel)
        linearVel=max(-1,linearVel)
        angularVel=min(0.5,angularVel)

        return (angularVel,linearVel)
            
        
        
    def listener_callback(self,msg):
        # /scan INCOMING DATA
        self.msgData=msg
        distanceList=list(msg.ranges)
        minSensorAngle=msg.angle_min
        maxSensorAngle=msg.angle_max
        angleIncrement=msg.angle_increment
        infCount=distanceList.count(float('inf'))+distanceList.count(float('nan'))

        self.targetAngle=0.0
        self.resultVariable=0.0
        self.noObjectFlag=False

        if infCount == len(distanceList):
            self.noObjectFlag=True
        else:
            objDistDir=self.findClosestObjectDirection(distanceList,minSensorAngle,maxSensorAngle,angleIncrement)
            self.targetAngle=objDistDir[0]
            self.resultVariable=objDistDir[1]
        self.cmdvel_callback()
    
    def cmdvel_callback(self):

        if self.msgData is None:
            self.get_logger().warn('[DEBUG][BALL_FOLLOW_NODE] NO SCAN DATA RECIEVED')
            return
        outputCmdVel=Twist()
        resultMove=self.moveRobot(0.0,self.safeDistanceObject)
        if self.noObjectFlag==False:
            resultMove=self.moveRobot(self.targetAngle,self.resultVariable)
        
        outputCmdVel.angular.z=resultMove[0]
        outputCmdVel.linear.x=resultMove[1]
        
        
        self.publisher.publish(outputCmdVel)
        self.get_logger().info('[DEBUG][BALL_FOLLOW_NODE] Dir : %f' %self.targetAngle)
        self.get_logger().info('[DEBUG][BALL_FOLLOW_NODE] Dist: %f' %self.resultVariable)



def main(args=None):
    rclpy.init(args=args)
    lidarsubb=BallFollowNode()
    rclpy.spin(lidarsubb)

    lidarsubb.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

