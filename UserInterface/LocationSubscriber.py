import rospy
import numpy as np
from tf2_msgs.msg import TFMessage
import cv2

class LocationSubscriber:
    """
    Creates a subscriber object that processes the location of the turtle bot in the Gazebo simulator.


    (6,0)    <-20->     (0,0)
    ----------------------
    |                     |
    |                     |  10 deep...
    |                     |  
    |                     |  
    |                     |   
    -----------------------
    (6,3)                   (0, 3)
    """

    def __init__(self, topic_path):
        self.row = None
        self.col = None
        self.subscriber = rospy.Subscriber(topic_path, TFMessage, self.callback)

    def callback(self, msg):
        # Grab location data from /tf topic, datatype is tf2_msgs/TFMessage
        for transform in msg.transforms:
            self.col = transform.transform.translation.x
            self.row = transform.transform.translation.y
    
    def get_location(self):
        return self.col, self.row
    
if __name__ == "__main__":
    rospy.init_node("location_subscriber")
    subscriber = LocationSubscriber("/tf")
    

    while not rospy.is_shutdown():
        x,y = subscriber.get_location()
        print(x, y)
