#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from ultralytics import YOLO  # type: ignore[import]
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32, Bool
from cv_bridge import CvBridge
import os
import logging
import signal
from geometry_msgs.msg import Point  
import time
import numpy as np

class CVNode(Node):
    def __init__(self):
        super().__init__("cv_node")


        # timer for pcb logging 
        self.last_pcb_log_time = 0
        # Suppress YOLO logs by setting the logging level to WARNING
        logging.getLogger("ultralytics").setLevel(logging.WARNING)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Publisher for log messages
        self.log_pub = self.create_publisher(String, '/aims/logs', 10)

        # Path to the YOLO model weights
        self.model_path = "/home/aims_user/aims_ws/best.pt"
        if not os.path.isfile(self.model_path):
            self.log_to_ui("ERROR", f"Model file not found or is not a file: {self.model_path}")
            self.destroy_node()
            rclpy.shutdown()
            return

        # Load YOLO model
        self.model = YOLO(self.model_path)

        # Publishers
        self.pub_detections = self.create_publisher(String, "/aims/detections", 10)
        self.pub_cv_image_robot = self.create_publisher(Image, "/aims/cv_image_robot", 10)
        self.pub_person_flag = self.create_publisher(Int32, "/aims/person_detected", 10)
        self.pcb_pos_pub = self.create_publisher(Point, "/aims/pcb_position", 10)
        self.pub_control = self.create_publisher(String, "/aims/control", 10)
        self.pub_auto = self.create_publisher(String, "/aims/auto", 10)
        # pick & place toggle
        self.pick_place_enabled = False
        self.isPicking = False

        # Subscriber to listen for is picking status
        self.sub_is_picking = self.create_subscription(
            Bool,
            '/aims/is_picking',
            self.cb_is_picking,
            10
        )

        #MD COMMENTED OUT THE PCB POSITION PUBLISHING ON 5/12
        self.sub_pick_mode = self.create_subscription(
            Bool,
            '/aims/pick_place_mode',
            self.cb_pick_mode,
            10
        )

        
        # your six center‐move CAN hex commands
        self.center_sequence = [
            "01FE0030AA000000", 
            "02FE0060AAFE8976", 
            "03FE0060AA000FDE", 
            "04FE0030AA000000", 
            "05FE0060AA00150F", 
            "06FE0060AAFFEAF1",
        ]
        self.place_sequence = [
            "01FE0030AA000E11",  
            "02FE0030AAFE71D2", 
            "03FE0030AA005DB5", 
            "04FE0030AA000000",
            "05FE0030AA003432",
            "06FE0030AA000A53",
        ]
        # HSV color ranges for your markers (tweak if needed)
        self.color_ranges = {
            "blue":   ((100, 150, 50), (130, 255, 255)),  # Top-left
            "pink": ((145, 100, 100), (170, 255, 255)),   # Top-right
            "red":    ((0, 120, 50), (10, 255, 255)),     # Bottom-left
            "purple": ((130, 80, 50), (160, 255, 255)),     # Botton-right
        }
        # Subscriber to listen for shutdown commands
        self.sub_shutdown = self.create_subscription(
            Bool,
            '/aims/shutdown',
            self.shutdown_callback,
            10
        )

        # Subscriber to listen for images from the Robot Camera
        self.create_subscription(Image, "/aims/robot_image_raw", self.callback_cam, 10)

        self.log_to_ui("INFO", "CV Node has started successfully!")

    # Callback function to process the incoming image from the Robot Camera
    def callback_cam(self, data):
        try:

            # Convert ROS Image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            matrix = None
            if self.pick_place_enabled:
                matrix = self.compute_homography_matrix(frame)



            # Run YOLO detection
            results = self.model.predict(frame, conf=0.60, iou=0.85)

            detected_objects = []
            person_detected = Int32()
            person_detected.data = 0  # Default flag is 0 (no person detected)

            for r in results:
                for box in r.boxes:
                    cls = int(box.cls[0])  # Get class ID
                    conf = float(box.conf[0])  # Confidence score
                    x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates

                    # Convert class ID to label
                    label = self.model.names[cls]
                    detected_objects.append(f"{label} {conf:.2f} ({x1},{y1})-({x2},{y2})")

                    # Calculate center of the bounding box
                    # Get bbox center
                    x_center = (x1 + x2) / 2
                    y_center = (y1 + y2) / 2

                    # Draw the visual center
                    cv2.circle(frame, (int(x_center), int(y_center)), 6, (0, 0, 255), -1)
                    cv2.putText(frame, "center", (int(x_center) + 10, int(y_center)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                    
                    # If it's a PCB, publish its real-world position
                    if label.lower() == "pcb" and self.pick_place_enabled:
                        #skip until homogoraphy is computed
                        if matrix is None:
                            continue   
                        x_ws, y_ws = self.warp_point_to_workspace(x_center, y_center, matrix)
                        y_ws = y_ws - 0.0 # Adjust y coordinate to match the PCB position
                        # Print the workspace coordinates
                        point_msg = Point()
                        point_msg.x = float(x_ws)     # ensure these are Python floats
                        point_msg.y = float(y_ws)
                        point_msg.z = float(0.0)              # you must set z or the default None will crash

                        # 2) publish it
                        self.pcb_pos_pub.publish(point_msg)


                        current_time = time.time()
                        if current_time - self.last_pcb_log_time > 10: #added 5/15 for 10 sec delay on pcb log 
                            self.log_to_ui("INFO", f"PCB Coordinates: ({x_ws}, {y_ws})")
                            self.last_pcb_log_time = current_time

                        if not self.isPicking:
                        # Set isPicking to True
                            self.isPicking = True

                            # Keep 2 decimal places
                            x = round(x_ws, 2)
                            y = round(y_ws, 2)
                            if y > 5:
                                y = 5
                            elif y < -6:
                                y = -6
                            if x > 6:
                                x = 6
                            elif x < -6:
                                x = -6

                            # Publish the PCB coordinates to the automated system handler
                            cmd = f"unknown {x} {y}"
                            self.pub_auto.publish(String(data=cmd))
                            self.log_to_ui("INFO", f"Sent interpolated move: {cmd}")

                    # Check if the detected object is a person
                    if label.lower() in ["person", "hand"]:
                        person_detected.data = 1  # Set flag to 1 if a person is detected

                    # Draw bounding box and label on the frame
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Publish detections to ROS
            detection_msg = String()
            detection_msg.data = ", ".join(detected_objects)
            self.pub_detections.publish(detection_msg)

            # Publish the person detection flag
            self.pub_person_flag.publish(person_detected)

            # Convert the processed frame to a ROS Image message
            processed_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.pub_cv_image_robot.publish(processed_image_msg)

        except Exception as e:
            self.log_to_ui("ERROR", f"Error in callback: {e}")

    # Publish log messages to the /aims/logs topic
    def log_to_ui(self, level, message):
        log_msg = String()
        log_msg.data = f"[{level}] {message}"
        self.log_pub.publish(log_msg)

#MD ADDED AND COMMENTED OUT THE PIXEL TO WORKSPACE FUNCTION ON 5/12 
    def pixel_to_workspace(self, x_px, y_px, image_width=640, image_height=480):
        x_norm = (x_px / image_width) * 2 - 1
        y_norm = (y_px / image_height) * 2 - 1

        x_workspace = x_norm * 6
        y_workspace = -y_norm * 5.5
        return round(x_workspace, 2), round(y_workspace, 2)

    # Callback to handle is picking status
    def cb_is_picking(self, msg: Bool):
        self.isPicking = bool(msg.data)
        if self.isPicking:
            self.log_to_ui("INFO", "Pick & Place in progress...")
            

    # Callback to handle pick & place mode toggle
    def cb_pick_mode(self, msg: Bool):
        self.pick_place_enabled = bool(msg.data)
        if self.pick_place_enabled:
            self.log_to_ui("INFO", "Pick & Place MODE ENABLED")
        else:
            self.log_to_ui("INFO", "Pick & Place MODE DISABLED")

    # Detect color blobs in the image
    def detect_color_blob(self, frame, hsv_lower, hsv_upper):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                cv2.circle(frame, (cx, cy), 6, (255, 255, 255), -1)

                return (cx, cy)
        return None

    # Compute homography and get the PCB position based on the detected color blobs
    def compute_homography_matrix(self, frame):
        tl = self.detect_color_blob(frame, *self.color_ranges["blue"])
        tr = self.detect_color_blob(frame, *self.color_ranges["pink"])
        bl = self.detect_color_blob(frame, *self.color_ranges["red"])
        br = self.detect_color_blob(frame, *self.color_ranges["purple"])

        if None in [tl, tr, bl, br]:
            #self.log_to_ui("ERROR", "Not all corners detected for homography.")
            return None

        src_pts = np.array([tl, tr, bl, br], dtype="float32")
        dst_pts = np.array([
            [-6,  5.5],
            [ 6,  5.5],
            [-6, -5.5],
            [ 6, -5.5]
        ], dtype="float32")

        matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
        # self.log_to_ui("INFO", f"Computed workspace homography.")
        return matrix

    def warp_point_to_workspace(self, x_px, y_px, matrix):
        pts = np.array([[[x_px, y_px]]], dtype="float32")
        warped = cv2.perspectiveTransform(pts, matrix)
        return warped[0][0][0], warped[0][0][1]

    # Callback to handle shutdown commands
    def shutdown_callback(self, msg):
        if msg.data:  # If the shutdown command is True
            self.log_to_ui("INFO", "Shutdown command received. Stopping CV Node...")
            self.pub_person_flag.publish(Int32(data=0))  # Reset person detection flag
            self.destroy_node()

    # Override the destroy_node method to handle cleanup
    def destroy_node(self):
        self.log_to_ui("INFO", "Shutting down CVNode...")
        super().destroy_node()  # Call the parent class's destroy_node

def main(args=None):
    rclpy.init(args=args)
    node = CVNode()

    # Define a signal handler to ensure proper cleanup
    def signal_handler(sig, frame):
        node.destroy_node()
        rclpy.shutdown()
        exit(0)

    # Register the signal handler for SIGINT (Ctrl+C) and SIGTERM (termination)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure shutdown is only called if rclpy is still active
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()