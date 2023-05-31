import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage
import cv2

class ImageSubscriber:
    """
    Creates a subscriber object that processes the Raspberry Pi camera's compressed image stream and converts it into an openCV format that can be consumed as a video source.
    """

    def __init__(self, topic_path):
        self.frame = None
        self.subscriber = rospy.Subscriber(topic_path, CompressedImage, self.callback)

    def callback(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        self.frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    def get_frame(self):
        return self.frame
    
if __name__ == "__main__":
    rospy.init_node("image_subscriber")
    subscriber = ImageSubscriber("/raspicam_node/image/compressed")
    

    while not rospy.is_shutdown():
        frame = subscriber.get_frame()

        if frame is not None:
            cv2.imshow('Video Feed', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cv2.destroyAllWindows()
