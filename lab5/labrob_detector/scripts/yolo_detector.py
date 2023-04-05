#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('labrob_detector')
import sys
import rospy
import cv2
import numpy as np
import time
import os
from cv_bridge import CvBridge
from ultralytics import YOLO
import rospkg
from cv_bridge import CvBridge
# DONE 1: Import proper ROS image messages
from sensor_msgs.msg import CompressedImage
# DONE 3: Import proper ROS detection messages
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection2D
from vision_msgs.msg import BoundingBox2D
from vision_msgs.msg import ObjectHypothesisWithPose
from geometry_msgs.msg import PoseWithCovariance

class image_converter:
    def __init__(self):
        self.name = "yolo_detector"
        queue_size = 10
        # DONE 1: Add image subscriber
        self.subscriber = rospy.Subscriber('/camera/raw_image/compressed',CompressedImage,self.callback,queue_size=queue_size)
        # DONE 2: Add image publisher
        self.image_publisher = rospy.Publisher(f'/{self.name}/raw_image/compressed',CompressedImage,queue_size=queue_size)
        # DONE 3: Add 2D detections publisher
        self.detection_publisher = rospy.Publisher(f'/{self.name}/2Ddetection/array',Detection2DArray,queue_size=queue_size)

        # initialize the model
        self.model = YOLO("yolov8n.pt")

        # initialize a list of colors to represent each possible class label
        np.random.seed(42)
        self.COLORS = np.random.randint(0, 255, size=(1000, 3), dtype="uint8")
        self.bridge = CvBridge()
        
        
    def callback(self,data):

		# DONE 1: Add code that converts ROS image to OpenCV image. Remove image=np.zeros(1,1,1)
        image = cv2.imdecode(
            np.frombuffer(data.data, np.uint8),
            cv2.IMREAD_COLOR)

        stamp = rospy.Time.now()

        results = self.model(image)[0]
        
        detections: list[Detection2D] = []
        for result in results:
            # extract the bounding box coordinates
            (x0, y0, x1, y1) = result.boxes.xyxy.cpu().numpy().astype(int)[0]
            class_id = int(result.boxes.cls.cpu().numpy()[0])
            confidence =  result.boxes.conf.cpu().numpy()[0]
            # draw a bounding box rectangle and label on the image
            color = [int(c) for c in self.COLORS[class_id]]
            cv2.rectangle(image, (x0, y0), (x1, y1), color, 2)
            text = "{}: {:.4f}".format(result.names[class_id],confidence)
            cv2.putText(image, text, (x0, y0 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # DONE? 3 Fill 2D detections based on information here
            detection = Detection2D()
            detection.header.stamp = stamp
            # bounding box
            bbox = BoundingBox2D()
            bbox.center.x = (x0 + x1)/2
            bbox.center.y = (y0 + y1)/2
            bbox.size_x = abs(x1-x0)
            bbox.size_y = abs(y1-y0)
            
            detection.bbox = bbox

            # class probabilities
            classes = result.boxes.cls.cpu().numpy()
            confidences = result.boxes.conf.cpu().numpy()
            hypotheses = []
            for i in range(len(classes)):
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = int(classes[i])
                hypothesis.score = confidences[i]
                x0,y0,x1,y1 = result.boxes.xyxy.cpu().numpy().astype(int)[i]
                hypothesis.pose.pose.position.x = (x0 + x1)/2
                hypothesis.pose.pose.position.y = (y0 + y1)/2
                hypotheses.append(hypothesis)

            detection.results = hypotheses

            detections.append(detection)
        
        raw_detection = Detection2DArray()
        raw_detection.header.stamp = stamp
        raw_detection.detections = detections

        # DONE 3: Add the code that publishes raw detections
        
        self.detection_publisher.publish(raw_detection)

        # DONE 2: Convert the annotated image to ROS image and publish it.
        converted_image = CompressedImage()
        converted_image.header.stamp = stamp
        converted_image.format = 'jpeg'
        converted_image.data = np.array(cv2.imencode('.jpg', image)[1]).tobytes()

        self.image_publisher.publish(converted_image)

def main(args):
    ic = image_converter()
    rospy.init_node('labrob_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
if __name__ == '__main__':
    main(sys.argv)
