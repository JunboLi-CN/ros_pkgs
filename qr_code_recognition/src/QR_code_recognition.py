#!/usr/bin/env python
# AKAMAV
# project: Indoor-copter
# author: Junbo Li
# Version: 2021.10.19

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from pyzbar.pyzbar import decode


def QR_code_recognition_1():
    # ROS node(Publisher)
    pub = rospy.Publisher('QR_code', String, queue_size = 10)
    rospy.init_node('camera', anonymous = True)

    capture = cv2.VideoCapture(0)      
    while not rospy.is_shutdown():
        ret, frame = capture.read()
        if ret is True:
            frame = cv2.flip(frame, 1)
            code_image = frame
            if code_image is not None:
                text = decode(code_image)
                if len(text) > 0:
                    info = text[0][0].decode("utf-8")
                    # Collect and send the result to the receiving node
                    rospy.loginfo(info)
                    pub.publish(info)
                    # Visualization
                    cv2.putText(frame, info, (20, 100), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2, 8)
            # Visualization
            cv2.imshow("result", frame)
            if cv2.waitKey(100) & 0xff == ord('q'):
                break
        else:
            break
    cv2.destroyAllWindows()
    capture.release()

def QR_code_recognition_2():
    # ROS node(Publisher)
    pub = rospy.Publisher('QR_code', String, queue_size = 10)
    rospy.init_node('camera', anonymous = True)

    cap = cv2.VideoCapture(0)
    while not rospy.is_shutdown():
        ret_camera, frame = cap.read()
        if ret_camera is True:
            detector = cv2.QRCodeDetector()
            ret_QR, decoded_info, points, decoded_qrcode = detector.detectAndDecodeMulti(frame)
            if ret_QR is True:
                # Calculate the total number of identifiable QR codes
                qr_num = len(decoded_info)
                # Visualization
                cv2.putText(frame, str(qr_num) + " QR codes detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                # Locate and decode QR code
                for n in range(qr_num):
                    text = "The information of the <" + str(n) + "> QR code is--->" + decoded_info[n]
                    # Collect and send the result to the receiving node
                    rospy.loginfo(text)
                    pub.publish(text)
                    # Visualization
                    color = [(0, 0, 255), (255, 0, 0), (0, 255, 255), (255, 255, 0), (255, 0, 255), (0, 155, 155)]
                    cv2.putText(frame, text, (10, 30*(n+2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color[n], 1)
                    cv2.drawContours(frame, [np.int32(points[n])], -1, color[n], 5)
                    cv2.putText(frame, str(n), (int(points[n][0][0]), int(points[n][0][1])), cv2.FONT_HERSHEY_SIMPLEX, 1, color[n], 2)
                
                # QR code reconstruction (Use it when necessary)

                # decoded_num = len(decoded_qrcode)
                # for n in range(decoded_num):
                #     ori_qrcode = np.array(decoded_qrcode[n])
                #     qrcode_size = ori_qrcode.shape[0]
                    
                #     res_qrcode = cv2.resize(np.resize(ori_qrcode, (qrcode_size, qrcode_size)), (0, 0), fx=5, fy=5)
                #     cv2.imshow("res_qrcode: " + str(n), res_qrcode)

                #     if decoded_num == 6:
                #         cv2.imwrite("decoded_qrcode" + str(n) + ": " + decoded_info[n], res_qrcode[n])
            
                # if qr_num == 6:
                #     cv2.imwrite("result_img/result.png", frame)
                #     if decoded_num == 6:
                #         break

                # # another way
                # qr_contours = []
                # for n in range(qr_num):
                #     text = "The information of the" + str(n) + "QR code is:   " + decoded_info[n]
                #     cv2.putText(frame, text, (10, 30*(n+2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                #     qr_contours.append(np.int32(points[n]))
                # cv2.drawContours(frame, qr_contours, -1, (0, 0, 255), 5)
                
                # # TEST
                # res_qrcode = np.resize(np.array(decoded_qrcode), (21, 21))
                # res_qrcode = cv2.resize(res_qrcode, (0, 0), fx=10, fy=10)
                # cv2.imshow("res_qrcode", res_qrcode)
        
            # Visualization
            cv2.imshow("result", frame)
            if cv2.waitKey(100) & 0xff == ord('q'):
                break
        else:
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        # M1: faster but low accuracy
        #QR_code_recognition_1()
        # M2: slower but higher accuracy
        QR_code_recognition_2()
    except rospy.ROSInterruptException:
        pass
