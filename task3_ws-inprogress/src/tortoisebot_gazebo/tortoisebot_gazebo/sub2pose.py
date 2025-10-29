import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Pose
from math import isinf,isnan,pi,degrees,exp


class CloseObjectFinder(Node):

    def __init__(self):
        super().__init__('qwe') # node name
        self.subscription=self.create_subscription(
            Pose,
            '/world/empty/pose/info',
            self.listener_callback,
            10) # subscribing to /scan filtered
        # self.publisher = self.create_publisher(String,'current_closest_object',10) #publishing to /current_closest_object

        self.msgData=None        
        
        
    
    
        
        
    def listener_callback(self,msg):
        # /scan INCOMING DATA
        self.msgData=msg
        print(msg)


def main(args=None):
    rclpy.init(args=args)
    class_subb=CloseObjectFinder()
    rclpy.spin(class_subb)

    class_subb.destroy_node()
    rclpy.shutdown()
    

if __name__=='__main__':
    main()

