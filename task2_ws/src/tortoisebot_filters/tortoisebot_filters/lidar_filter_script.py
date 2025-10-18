import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from math import isinf,isnan


class LidarFilterNode(Node):

    def __init__(self):
        super().__init__('lidar_filtering_node') # node name
        self.subscription=self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10) # subscribing to /scan
        self.publisher = self.create_publisher(Float32,'closest_object_distance',10) #publishing to /closest_object_distance

        self.msgData=None
        self.resultVariable=[]


        #timer_period=0.125 #seconds
        #self.timer=self.create_timer(timer_period,self.publisher_callback) # if publishing at specific interval
        
    def movingAverageFilter():
        pass

    @staticmethod
    def spatialMedianFilter(listVariable,windowSize=5):
        rangeLength=len(listVariable)
        medianFilteredList=[]

        for i in range(rangeLength):
            
            if i != rangeLength-2 and i != rangeLength-1:
                currentWindow=[listVariable[i-2],listVariable[i-1],listVariable[i],listVariable[i+1],listVariable[i+2]]
                currentWindow.sort()
                medianFilteredList.append(currentWindow[2])
            if i==rangeLength-2:
                currentWindow=[listVariable[i-2],listVariable[i-1],listVariable[i],listVariable[i+1],listVariable[0]]
                currentWindow.sort()
                medianFilteredList.append(currentWindow[2])
            if i==rangeLength-1:
                currentWindow=[listVariable[i-2],listVariable[i-1],listVariable[i],listVariable[0],listVariable[1]]
                currentWindow.sort()
                medianFilteredList.append(currentWindow[2])
        
        return medianFilteredList

    
    @staticmethod
    def minimumValidFilter(listVariable,minRange,maxRange):
        listLength=len(listVariable)
        
        minValFilteredList=[]

        for i in range(listLength):
            if minRange<=listVariable[i]<=maxRange and not isinf(listVariable[i]) and not isnan(listVariable[i]):
                minValFilteredList.append(listVariable[i])
        return minValFilteredList
    
    @staticmethod
    def FrontRangeFilter(listVariable,maxSample):
        listLength=len(listVariable)
        updatedList=[0]*listLength

        for i in range(listLength//2):
            
            if i in range(maxSample):
                updatedList[i]=listVariable[i]
                updatedList[-i]=listVariable[-i]
            else:
                updatedList[i]=(float('inf'))
        
        return updatedList
    
    
    def listener_callback(self,msg):
        # /scan INCOMING DATA
        self.msgData=msg
        distanceList=list(msg.ranges)
        sampleCount=len(distanceList)
        requiredAngleDegrees=15 #degrees
        requiredMaxSample=(int)((sampleCount/360)*requiredAngleDegrees)
                                
        
        # EXTRACTING DATA ONLY IN -15 TO 15 DEGREES
        angleFilteredScan=self.FrontRangeFilter(distanceList,requiredMaxSample)
        
        # APPLYING MEDIAN FILTER TO MANAGE SPIKES
        medianFilteredScan=self.spatialMedianFilter(angleFilteredScan)
        


        # FILTERING OUT INVALID VALUES - inf,NaN,Zeros
        lidarMinRange=msg.range_min+0.3
        lidarMaxRange=msg.range_max
        minValFilteredList=self.minimumValidFilter(medianFilteredScan,lidarMinRange,lidarMaxRange)
        #print(minValFilteredList)
        
        self.resultVariable=min(minValFilteredList)



        self.get_logger().info('[DEBUG] SUBSCRIBED TO /scan')
        self.publisher_callback()
    
    def publisher_callback(self):
        if self.msgData is None:
            self.get_logger().warn('[DEBUG] NO SCAN DATA RECIEVED')
            return
        
        
        # rangeFilterOutput=LaserScan()
        # rangeFilterOutput.header=self.msgData.header
        # rangeFilterOutput.angle_min=self.msgData.angle_min
        # rangeFilterOutput.angle_max=self.msgData.angle_max
        # rangeFilterOutput.angle_increment=self.msgData.angle_increment
        # rangeFilterOutput.time_increment=self.msgData.time_increment
        # rangeFilterOutput.scan_time=self.msgData.scan_time
        # rangeFilterOutput.range_min=self.msgData.range_min
        # rangeFilterOutput.range_max=self.msgData.range_max
        # rangeFilterOutput.ranges=self.resultVariable
        # rangeFilterOutput.intensities=self.msgData.intensities
        rangeFilterOutput=Float32()
        rangeFilterOutput.data=self.resultVariable
        self.publisher.publish(rangeFilterOutput)
        self.get_logger().info('[DEBUG] Closest Distance %f' %self.resultVariable)
        self.get_logger().info('[DEBUG] Publishing to /')



def main(args=None):
    rclpy.init(args=args)
    lidarsubb=LidarFilterNode()
    rclpy.spin(lidarsubb)

    lidarsubb.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

