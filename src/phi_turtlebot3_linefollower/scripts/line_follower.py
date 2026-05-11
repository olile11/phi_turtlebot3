#! /usr/bin/env python3
import cv2
import numpy as np
import cv_bridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
import rclpy
from rclpy.node import Node
import math

class LineDetection(Node):
    def __init__(self):
        super().__init__('ERROR') # node name
        self.bridge = cv_bridge.CvBridge()
        self.pub = self.create_publisher(Float32, 'geometry_msgs', 10)
        
        # Use timer to replace while loop so it doesn't block executor
        self.timer = self.create_timer(0.1, self.image_callback)
        self.v = cv2.VideoCapture(0)

    @staticmethod
    def draw_grid(img, grid_shape, color=(0, 255, 0), thickness=1):
        h, w, _ = img.shape
        rows, cols = grid_shape
        dy, dx = h / rows, w / cols

        # draw vertical lines
        for x in np.linspace(start=dx, stop=w-dx, num=cols-1):
            x = int(round(x))
            cv2.line(img, (x, 0), (x, h), color=color, thickness=thickness)

        # draw horizontal lines
        for y in np.linspace(start=dy, stop=h-dy, num=rows-1):
            y = int(round(y))
            cv2.line(img, (0, y), (w, y), color=color, thickness=thickness)

    def image_callback(self):
        cx=320
        cy=90
        ret, frame1 = self.v.read()
        if not ret:
            return
            
        frame=frame1[300:480, ::]
        LineDetection.draw_grid(frame,(2,2))
        
        # color space change
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv=cv2.medianBlur(hsv,9)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 30])
        mask = cv2.inRange(hsv,lower_black, upper_black)
        result=cv2.bitwise_and(frame,frame,mask=mask)
        
        #Contours
        contours, hierarchy = cv2.findContours(mask, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE)
        
        d = 0.0
        if len(contours) !=0:
            cv2.drawContours(image=result, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=1,
                             lineType=cv2.LINE_AA)
            #Centre
            c=max(contours,key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                print(cX,cY)
                if cX<cx:
                    d = math.sqrt(( cy-cY)**2 + ( cx-cX)**2)
                    print("turn left")
                    print(d)
                if cX>cx:
                    d = -(math.sqrt(( cy-cY)**2 + ( cx-cX)**2))
                    print("turn right")
                    print(d)
                if cX==cx:
                    print(d)
                    print("you r on right path")
                if d==0:
                    print(d)
                    print("you r right")
            else:
                cX, cY = 0, 0
            
            cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
            cv2.putText(frame, "centroid", (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        msg = Float32()
        msg.data = float(d)
        self.pub.publish(msg)
        
        cv2.imshow('Cropped', frame)
        cv2.imshow('result', result)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    linedec = LineDetection()
    try:
        rclpy.spin(linedec)
    except KeyboardInterrupt:
        pass
        
    linedec.v.release()
    cv2.destroyAllWindows()
    linedec.destroy_node()
    rclpy.try_shutdown()

if __name__=='__main__':
    main()