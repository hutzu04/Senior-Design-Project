#!/usr/bin/env python3

# <------------------------------------------>
#                  IMPORTS
# <------------------------------------------>

import os
import rclpy
import subprocess
import serial
import threading
import time
from std_msgs.msg import Int32, Bool, String
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow  # type: ignore
from rqt_aims.send_g_code import calculate_crc
from PyQt5.QtCore import Qt, QMetaObject, Q_ARG
from time import sleep
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtWidgets import QFileDialog
from rclpy.node import Node
from geometry_msgs.msg import Point
import json
from pathlib import Path

def wait_for_ros_folder_unlock():
    ros_folder = "/opt/ros"
    while os.path.exists(ros_folder) and not os.access(ros_folder, os.R_OK):
        time.sleep(1)  # Wait for 1 second before retrying

# <------------------------------------------>
#                  AIMS CLASS
# <------------------------------------------>

# Define the AIMS class
class AIMS(Plugin):
    instance_created = False  # Class-level flag to track if an instance already exists

    def __init__(self, context):
        # Always call the parent class's __init__ method
        super(AIMS, self).__init__(context)

        # Ensure rclpy is initialized only once
        if not rclpy.ok():
            wait_for_ros_folder_unlock()
            rclpy.init(args=None)

        # Create a ROS 2 node for the plugin
        self.node = rclpy.create_node('aims_plugin_node')

        #6/26: Initialize the pause flag 
        self.is_paused = False
        #6/28: Initialize the go_auto_running flag
        self.go_auto_running = False
        self.motor_done = threading.Event() 
        
        # ----------------------- PUBLISHERS AND SUBSCRIBERS ------------------------- #

        # Add publishers
        self.shutdown_pub = self.node.create_publisher(Bool, '/aims/shutdown', 10)
        self.joystick_enabled_pub = self.node.create_publisher(Bool, '/aims/joystick_enabled', 10)
        self.log_pub = self.node.create_publisher(String, '/aims/logs', 10)
        self.control_pub = self.node.create_publisher(String, '/aims/control', 10)
        self.stop_on_human_detected_pub = self.node.create_publisher(Bool, '/aims/stop_on_human_detected', 10)
        self.pcb_pos_pub = self.node.create_publisher(Point, "/aims/pcb_position", 10)
        self.log_can_pub = self.node.create_publisher(String, '/aims/log_can', 10)
        self.autonomous_pub = self.node.create_publisher(Bool, '/aims/pick_place_mode', 10)
        self.is_picking_pub = self.node.create_publisher(Bool, '/aims/is_picking', 10)
        self.in_position_mode = False  # Flag to track if the robot is in position mode
        # Subscribe to the /aims/person_detected topic
        self.person_detected_flag = 0
        self.subscription = self.node.create_subscription(
            Int32,
            '/aims/person_detected',
            self.person_detected_callback,
            10
        )
        # Subscribe to the /aims/logs topic
        self.node.create_subscription(
            String,
            '/aims/logs',
            self.log_callback,
            10
        )
        # Subscribe to the /aims/motor_feedback topic
        self.motor_feedback_sub = self.node.create_subscription(
            String,
            '/aims/motor_feedback',
            self.motor_feedback_callback,
            10
        )
        # Subscribe to the /aims/scanning topic
        self.scanning_sub = self.node.create_subscription(
            String,
            '/aims/scanning',
            self.scanning_callback,
            10
        )
        self.auto_sub = self.node.create_subscription(
            String,
            '/aims/auto',
            self.auto_callback,
            10
        )


        # ---------------------------- ROS 2 INITIALIZATION ------------------------- #

        # Check if an instance already exists
        if AIMS.instance_created:
            self.log_to_ui("WARN", "AIMS plugin instance already exists. Skipping initialization.")
            return
        AIMS.instance_created = True  # Mark the instance as created
        # Initialize executor to None
        self.executor = None

        
        # ------------------------- UI SETUP ------------------------- #

        # Set name for the plugin
        self.setObjectName('AIMS')
        # Load the UI file
        self.ui = QMainWindow()  # Create a QMainWindow instance
        ui_file = os.path.join(get_package_share_directory('rqt_aims'), 'resource', 'AIMS_2.ui')
        if not os.path.exists(ui_file):
            self.log_to_ui("ERROR", f"UI file not found: {ui_file}")
        else:
            self.log_to_ui("INFO", f"Loading UI file: {ui_file}")
        try:
            loadUi(ui_file, self.ui)  # Load the UI file into the QMainWindow instance
            self.log_to_ui("INFO", "UI loaded successfully.")
        except Exception as e:
            self.log_to_ui("ERROR", f"Failed to load UI: {e}")
        # Set the object name for the main widget
        self.ui.setObjectName('AIMSUi')
        # Add the widget to the context
        context.add_widget(self.ui)
        self.ui.show()
        # Check if the plugin is being loaded in a visible context
        if context.serial_number() > 1:
            self.log_to_ui("WARN", "AIMS plugin is being loaded in a non-visible context. Skipping initialization.")
            return

        # ----------------------------- INITIAL VALUES ----------------------------- #

        # Initialize the serial connection status to "Unavailable"
        self.update_serial_status("Serial Connection Unavailable", "red")
        # Initialize the current working directory
        self.current_directory = os.getcwd()
        self.ui.speedSlider.setValue(49)  # Set the slider to start at the middle value
        self.global_can = ""
        self.message = ""
        self.last_command = ""
        self.last_auto_sequence = -1 # Last auto sequence command sent: -1 means not started or already completed
        self.hasBeenHomed = False # Marker for if the robot has been homed
        self.homing = False # Marker for if the robot is currently homing
        self.scanning = False # Marker for if the robot is currently scanning
        self.in_position_mode = False  # Flag to track if the robot is in position mode
        self.needs_to_continue = False  # Flag to track if the robot needs to continue after a person is detected
        # Initialize a dictionary to store motor angles
        self.motor_angles = {
            "1": 0,
            "2": 0,
            "3": 0,
            "4": 0,
            "5": 0,
            "6": 0,
        }

        # ----------------------------- UI CONNECTIONS ----------------------------- #

        # Connect buttons to methods
        self.ui.sendCANButton.clicked.connect(self.on_send_can_clicked)
        self.ui.startROSButton.clicked.connect(self.on_start_ros_clicked)
        self.ui.startCameraButton.clicked.connect(self.on_start_camera_clicked)
        self.ui.startRobotCamButton.clicked.connect(self.on_start_robot_camera_clicked)
        # Connect buttons to generalized methods for pressed and released signals
        self.ui.motor1LeftButton.pressed.connect(lambda: self.on_motor_command_pressed("motor1_left"))
        self.ui.motor1LeftButton.released.connect(lambda: self.on_motor_command_released("stop1"))
        self.ui.motor1RightButton.pressed.connect(lambda: self.on_motor_command_pressed("motor1_right"))
        self.ui.motor1RightButton.released.connect(lambda: self.on_motor_command_released("stop1"))
        self.ui.motor2UpButton.pressed.connect(lambda: self.on_motor_command_pressed("motor2_up"))
        self.ui.motor2UpButton.released.connect(lambda: self.on_motor_command_released("stop2"))
        self.ui.motor2DownButton.pressed.connect(lambda: self.on_motor_command_pressed("motor2_down"))
        self.ui.motor2DownButton.released.connect(lambda: self.on_motor_command_released("stop2"))
        self.ui.motor3UpButton.pressed.connect(lambda: self.on_motor_command_pressed("motor3_up"))
        self.ui.motor3UpButton.released.connect(lambda: self.on_motor_command_released("stop3"))
        self.ui.motor3DownButton.pressed.connect(lambda: self.on_motor_command_pressed("motor3_down"))
        self.ui.motor3DownButton.released.connect(lambda: self.on_motor_command_released("stop3"))
        self.ui.motor4CwiseButton.pressed.connect(lambda: self.on_motor_command_pressed("motor4_clockwise"))
        self.ui.motor4CwiseButton.released.connect(lambda: self.on_motor_command_released("stop4"))
        self.ui.motor4CCwiseButton.pressed.connect(lambda: self.on_motor_command_pressed("motor4_counterclockwise"))
        self.ui.motor4CCwiseButton.released.connect(lambda: self.on_motor_command_released("stop4"))
        self.ui.motor5_6UpButton.pressed.connect(lambda: self.on_motor_command_pressed("motor5_6_up"))
        self.ui.motor5_6UpButton.released.connect(lambda: self.on_motor_command_released("stop5_6"))
        self.ui.motor5_6DownButton.pressed.connect(lambda: self.on_motor_command_pressed("motor5_6_down"))
        self.ui.motor5_6DownButton.released.connect(lambda: self.on_motor_command_released("stop5_6"))
        self.ui.motor5_6CwiseButton.pressed.connect(lambda: self.on_motor_command_pressed("motor5_6_clockwise"))
        self.ui.motor5_6CwiseButton.released.connect(lambda: self.on_motor_command_released("stop5_6"))
        self.ui.motor5_6CCwiseButton.pressed.connect(lambda: self.on_motor_command_pressed("motor5_6_counterclockwise"))
        self.ui.motor5_6CCwiseButton.released.connect(lambda: self.on_motor_command_released("stop5_6"))
        self.ui.gripperAdvOpenButton.pressed.connect(lambda: self.on_motor_command_pressed("gripper_open"))
        self.ui.gripperAdvCloseButton.pressed.connect(lambda: self.on_motor_command_pressed("gripper_close"))
        self.ui.stopROSButton.clicked.connect(self.on_stop_ros_clicked)
        self.ui.goHomeButton.clicked.connect(self.start_go_home_thread)
        self.ui.goScanButton.clicked.connect(self.start_go_scan_thread)
        self.ui.findHomeButton.clicked.connect(self.start_find_home_thread)
        self.ui.goPositionButton.clicked.connect(self.start_go_position_thread)
        self.ui.goTestButton.clicked.connect(self.start_go_test_thread)
        self.ui.goPlaceButton.clicked.connect(self.start_go_place_thread)

        # Connect the checkboxes to their handlers
        self.ui.joystickCheckBox.stateChanged.connect(self.on_joystick_checkbox_changed)
        self.ui.humanCheckBox.stateChanged.connect(self.on_human_checkbox_changed)
        self.ui.autonomousCheckBox.stateChanged.connect(self.on_autonomous_checkbox_changed)
        # Connect the command input field to the terminal handler
        self.ui.commandLineEdit.returnPressed.connect(self.execute_terminal_command)
        # Connect the speedSlider to update the speed
        self.ui.speedSlider.valueChanged.connect(self.update_speed)

        # Set UI initial states
        self.ui.commandTextEdit.setReadOnly(True)
        self.ui.outputLogTextEdit.setReadOnly(True)
        self.ui.joystickCheckBox.setEnabled(False)  # False joystick mode checkbox
        self.ui.feedbackCheckBox.setEnabled(False)  # False feedback mode checkbox
        self.ui.autonomousCheckBox.setEnabled(False)  # False autonomous mode checkbox
        self.ui.humanCheckBox.setEnabled(False)  # False human mode checkbox
        self.ui.stopROSButton.setEnabled(False)
        self.ui.startROSButton.setEnabled(True)
        self.ui.speedSlider.setEnabled(False)
        self.ui.startRobotCamButton.setEnabled(False) # False
        self.ui.findHomeButton.setEnabled(False)
        self.ui.startRobotCamButton.setToolTip("Nodes must be started in order to start the Robot Cam.")
        self.ui.findHomeButton.setToolTip("Nodes must be started and feedback mode enabled to find home.")
        self.ui.goHomeButton.setEnabled(False) # False
        self.ui.goHomeButton.setToolTip("Nodes must be started and home found to go home.")
        self.ui.goScanButton.setEnabled(False) # False
        self.ui.goScanButton.setToolTip("Nodes must be started and home found to scan.")
        self.ui.goPositionButton.setEnabled(False) # False 
        self.ui.goPositionButton.setToolTip("Nodes must be started and home found to go to position.")
        self.ui.goTestButton.setEnabled(False) # False
        self.ui.goTestButton.setToolTip("Nodes must be started and home found to go to pick position.")
        self.ui.goPlaceButton.setEnabled(False) # False
        self.ui.goPlaceButton.setToolTip("Nodes must be started and home found to go to place position.")
        #MD added for rqt logger button 
        self.ui.logAnglesButton.clicked.connect(self.log_pick_angles)
        # -------------------------- INITIALIZE PROCESSES ----------------------------- #

        # Store the process handle for the launched ROS nodes and processes
        self.ros_process = None
        self.robot_cam_started = False
        self.robot_cam_process = None
        self.background_cam_started_started = False
        self.background_cam_process = None
        
        # Start the ROS spinning thread
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        # Set the program as running
        self.running = True

        # State gear ratios for the 6 motors in an array
        self.gear_ratios = {
            "1": 13.5,
            "2": 150,
            "3": 150,
            "4": 48,
            "5": 67.82,
            "6": 67.82,
        } 

        # -------------------------- INITIALIZE TREADS ----------------------------- #
        self.ros_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.ros_thread.start()
        self.start_feedback_request_loop()
        self.start_serial_monitoring()

        # Log the successful start of the plugin
        self.log_to_ui("INFO", "AimsPlugin has started successfully!")


    # <------------------------------------------>
    #             TERMINAL EXECUTION
    # <------------------------------------------>

    # Function to execute terminal commands
    def execute_terminal_command(self):
        command = self.ui.commandLineEdit.text().strip()  # Get the command from the input field
        if not command:
            return  # Do nothing if the command is empty

        # Display the current directory and the command in the terminal output
        self.ui.commandTextEdit.appendPlainText(f"{self.current_directory} $ {command}")

        try:
            # Handle 'cd' commands separately
            if command.startswith("cd"):
                # Extract the target directory
                target_dir = command[3:].strip() or os.path.expanduser("~")  # Default to home directory if no path is provided
                new_directory = os.path.abspath(os.path.join(self.current_directory, target_dir))
                if os.path.isdir(new_directory):
                    self.current_directory = new_directory
                    self.ui.commandTextEdit.appendPlainText(f"Changed directory to: {self.current_directory}")
                else:
                    self.ui.commandTextEdit.appendPlainText(f"cd: no such file or directory: {target_dir}")
            else:
                # Execute other commands in the current working directory
                result = subprocess.run(command, shell=True, cwd=self.current_directory,
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

                # Display the command output
                if result.stdout:
                    self.ui.commandTextEdit.appendPlainText(result.stdout)
                if result.stderr:
                    self.ui.commandTextEdit.appendPlainText(result.stderr)

        except Exception as e:
            # Display any errors that occur
            self.ui.commandTextEdit.appendPlainText(f"Error: {str(e)}")

        # Clear the input field
        self.ui.commandLineEdit.clear()

    # Function to log messages to the output log
    def log_to_output(self, message):
        QMetaObject.invokeMethod(
            self.ui.outputLogTextEdit,
            "appendPlainText",
            Qt.QueuedConnection,
            Q_ARG(str, message)
        )

    # Publish log messages to the /aims/logs topic
    def log_to_ui(self, level, message):
        log_msg = String()
        log_msg.data = f"[{level}] {message}"
        self.log_pub.publish(log_msg)

    # <------------------------------------------>
    #          THREAD AND SERIAL FUNCTIONS
    # <------------------------------------------>

    # Function to start feedback request loop
    def start_feedback_request_loop(self):
        msg_1 = String()
        msg_1.data = "speed_1"
        msg_2 = String()
        msg_2.data = "speed_2"
        msg_3 = String()
        msg_3.data = "speed_3"
        msg_4 = String()
        msg_4.data = "speed_4"
        msg_5 = String()
        msg_5.data = "speed_5"
        msg_6 = String()
        msg_6.data = "speed_6"
        amsg_1 = String()
        amsg_1.data = "angle_1"
        amsg_2 = String()
        amsg_2.data = "angle_2"
        amsg_3 = String()
        amsg_3.data = "angle_3"
        amsg_4 = String()
        amsg_4.data = "angle_4"
        amsg_5 = String()
        amsg_5.data = "angle_5"
        amsg_6 = String()
        amsg_6.data = "angle_6"

        def feedback_request_loop():
            while self.running and rclpy.ok():
                if self.ui.feedbackCheckBox.isChecked():
                    self.ui.findHomeButton.setEnabled(True) # True
                    if self.homing == True:
                        self.control_pub.publish(amsg_1)
                        time.sleep(0.02)
                        self.control_pub.publish(amsg_2)
                        time.sleep(0.02)
                        self.control_pub.publish(amsg_3)
                        time.sleep(0.02)
                        self.control_pub.publish(amsg_4)
                        time.sleep(0.02)
                        self.control_pub.publish(amsg_5)
                        time.sleep(0.02)
                        self.control_pub.publish(amsg_6)
                        time.sleep(0.02)
                    #6/26 commented out the ultrasonic scanning code
                    # elif self.scanning == True:
                    #     self.control_pub.publish(String(data="ultrasonic"))
                    else:
                        self.control_pub.publish(msg_1)
                        time.sleep(0.05)  # sleep
                        self.control_pub.publish(amsg_1)
                        time.sleep(0.05)
                        self.control_pub.publish(msg_2)
                        time.sleep(0.05)  # sleep
                        self.control_pub.publish(amsg_2)
                        time.sleep(0.05)
                        self.control_pub.publish(msg_3)
                        time.sleep(0.05)
                        self.control_pub.publish(amsg_3)
                        time.sleep(0.05)
                        self.control_pub.publish(msg_4)
                        time.sleep(0.05)
                        self.control_pub.publish(amsg_4)
                        time.sleep(0.05)
                        self.control_pub.publish(msg_5)
                        time.sleep(0.05)
                        self.control_pub.publish(amsg_5)
                        time.sleep(0.05)
                        self.control_pub.publish(msg_6)
                        time.sleep(0.05)  # sleep
                        self.control_pub.publish(amsg_6)
                        time.sleep(0.05)
                else:
                    self.ui.findHomeButton.setEnabled(False) # False

        # Start the feedback request loop in a separate thread
        threading.Thread(target=feedback_request_loop, daemon=True).start()

    # Function to start the serial monitoring thread
    def start_serial_monitoring(self):
        def monitor_serial_ports():
            current_port = None  # Initialize current_port to None

            # Start monitoring the serial ports in a loop
            while self.running and rclpy.ok():
                available_ports = [port for port in ["/dev/ttyACM0", "/dev/ttyACM1"] if os.path.exists(port)]

                if available_ports:
                    # Serial connection is available
                    new_port = available_ports[0]  # Use the first available port
                    if new_port != current_port:
                        # If the port has changed, update the current port and publish it
                        current_port = new_port
                        self.update_serial_status("Serial Connection Available!!!!", "green")
                        self.publish_serial_port(current_port)
                else:
                    # Serial connection is unavailable
                    if current_port is not None:
                        # If the port was previously available but is now unavailable, reset it
                        current_port = None
                        self.update_serial_status("Serial Connection Unavailable", "red")
                        self.publish_serial_port("None")

                sleep(0.25)  # Check every 250ms

        # Start the monitoring thread
        threading.Thread(target=monitor_serial_ports, daemon=True).start()

    # Function to publish the available serial port to the /aims/serial_port topic
    def publish_serial_port(self, port_name):
        # Publish the name of the available serial port to the /aims/serial_port topic
        serial_port_msg = String()
        serial_port_msg.data = port_name
        self.node.create_publisher(String, '/aims/serial_port', 10).publish(serial_port_msg)

    # Function to update the UI with the serial connection status
    def update_serial_status(self, message, color):
        # Use QMetaObject.invokeMethod to safely update the UI from the thread
        QMetaObject.invokeMethod(self.ui.serialTextBrowser, "setText", Qt.QueuedConnection, Q_ARG(str, message))
        QMetaObject.invokeMethod(self.ui.serialTextBrowser, "setStyleSheet", Qt.QueuedConnection, Q_ARG(str, f"background-color: {color}; color: white;"))

    # Function to start the find home process in a separate thread
    def start_find_home_thread(self):
        self.log_to_ui("INFO", "Starting homing process in a separate thread...")
        thread = threading.Thread(target=self.find_home, daemon=True)
        thread.start()

    # Function to start the go home process in a separate thread
    def start_go_home_thread(self):
        self.log_to_ui("INFO", "Going home...")
        thread = threading.Thread(target=self.go_home, daemon=True)
        thread.start()

    # Function to start the scan process in a separate thread
    def start_go_test_thread(self):
        self.log_to_ui("INFO", "Performing pick sequence...")
        thread = threading.Thread(target=self.go_test, daemon=True)
        thread.start()

    # Function to start the scan process in a separate thread
    def start_go_place_thread(self):
        self.log_to_ui("INFO", "Performing place sequence...")
        thread = threading.Thread(target=self.go_place, daemon=True)
        thread.start()

    # Function to start the go to scan position process in a separate thread
    def start_go_scan_thread(self):
        self.log_to_ui("INFO", "Going to scan position...")
        thread = threading.Thread(target=self.go_scan, daemon=True)
        thread.start()

    # Function to start the go position process in a separate thread
    def start_go_position_thread(self):
        self.log_to_ui("INFO", "Going to position...")
        thread = threading.Thread(target=self.go_position, daemon=True)
        thread.start()

    # Function to start the go auto process in a separate thread
    def start_go_auto_thread(self):
        """Launch the FSM in a thread, unless it’s already running."""
        if self.go_auto_running:
            self.log_to_ui("WARN", "go_auto already running.")
            return
        threading.Thread(target=self.go_auto, daemon=True).start()

    def _reissue_current_motion(self):
        if   self.last_auto_sequence == 1:   # move-to-pick
            self.control_pub.publish(String(data=self.message))
        elif self.last_auto_sequence == 3:   # half-raise
            self.control_pub.publish(String(data="half_raise"))  
        elif self.last_auto_sequence == 4:   # place
            self.control_pub.publish(String(data="place"))
        elif self.last_auto_sequence == 6:   # place_up
            self.control_pub.publish(String(data="place_up"))
        elif self.last_auto_sequence == 7:   # scan
            self.control_pub.publish(String(data="scan"))
        # Let the running FSM handle the next wait_for_stop().


    # <------------------------------------------>
    #                 CALLBACKS
    # <------------------------------------------>

    def scanning_callback(self, msg):
        if msg.data == "start":
            self.scanning = True
            self.log_to_ui("INFO", "Scanning started.")
        elif msg.data == "stop":
            self.scanning = False
            self.log_to_ui("INFO", "Scanning stopped.")
        else:
            self.log_to_ui("WARN", f"Unknown scanning command: {msg.data}")
    
    # Callback function for the /aims/person_detected topic
    def person_detected_callback(self, msg):
        self.person_detected_flag = msg.data

        if self.ui.humanCheckBox.isChecked():
            if self.person_detected_flag == 1 and not self.is_paused:
                self.is_paused = True
                self.log_to_ui("WARN", f"Person detected! Stopping all motion.")
                self.disable_motor_buttons()
                if self.in_position_mode:
                    self.needs_to_continue = True
                    self.control_pub.publish(String(data="stop_all"))

            elif self.person_detected_flag == 0 and self.is_paused:
                self.is_paused = False
                self.log_to_ui("INFO", f"Person no longer detected. Resuming.")
                self.enable_motor_buttons()
                if self.last_command.startswith("auto"):
                    self._reissue_current_motion()
                if self.in_position_mode and self.needs_to_continue:
                    self.needs_to_continue = False
                    if self.last_command == "home":
                        self.start_go_home_thread()
                    elif self.last_command == "scan":
                        self.start_go_scan_thread()
                    elif self.last_command == "test":
                        self.start_go_test_thread()
                    elif self.last_command == "place":
                        self.start_go_place_thread()
                    elif self.last_command.startswith("known") or self.last_command.startswith("unknown"):
                        self.start_go_position_thread()
                    elif self.last_command.startswith("auto"):
                        pass  # let the still-alive FSM thread resume itself

    # Callback to handle log messages from other nodes
    def log_callback(self, msg):
        self.log_to_output(msg.data)

    # Callback to handle motor feedback messages
    def motor_feedback_callback(self, msg):
        feedback = msg.data.strip()

        # Remove the "Response: " prefix if it exists
        if feedback.startswith("Response: "):
            feedback = feedback[len("Response: "):]

        # Split the feedback message into parts
        parts = feedback.split(" ")
        if len(parts) == 5:  # Expecting 5  or 5fields: motor_id, command_type, speed_high, speed_low, crc
            # Extract the motor ID, command type, and speed
            motor_id, command_type, speed_high, speed_low, crc = parts  # Ignore the CRC field

            # Check if the command type is "32" (speed feedback)
            if command_type == "32":
                #self.log_to_ui("INFO", f"Motor {motor_id} speed feedback received: {feedback}")
                try:
                    if speed_low == "FF":
                        speed_low = "00"
                    # Combine the high and low parts of the speed and convert to base 10
                    speed_decimal = int(speed_low, 16)
                    if speed_high == "FF" and speed_low != "00":
                        speed_decimal -= 255

                    # Update the respective LCD panel in the UI based on the motor ID
                    if motor_id == "1":
                        self.ui.motor1SpeedLCD.display(speed_decimal)
                    elif motor_id == "2":
                        self.ui.motor2SpeedLCD.display(speed_decimal)
                    elif motor_id == "3":
                        self.ui.motor3SpeedLCD.display(speed_decimal)
                    elif motor_id == "4":
                        self.ui.motor4SpeedLCD.display(speed_decimal)
                    elif motor_id == "5":
                        self.ui.motor5SpeedLCD.display(speed_decimal)
                    elif motor_id == "6":
                        self.ui.motor6SpeedLCD.display(speed_decimal)
                    else:
                        self.log_to_ui("WARN", f"Unknown motor ID: {motor_id}")
                except ValueError:
                    self.log_to_ui("ERROR", f"Failed to parse speed value: {speed_high} {speed_low}")  
            #else:
                #self.log_to_ui("INFO", f"Motor {motor_id} motion feedback received: {feedback}")
        elif len(parts) == 9:
            # Extract the motor ID, command type, and speed
            motor_id, command_type, angle6, angle5, angle4, angle3, angle2, angle1, crc = parts  # Ignore the CRC field

            # Check if the command type is "31" (angle feedback)
            if command_type == "31":
                #self.log_to_ui("INFO", f"Motor {motor_id} angle feedback received: {feedback}")
                try:
                    # Combine the high and low parts of the speed and convert to base 10 with 2's complement in mind
                    # Step 1: Concatenate all hex values into one string
                    concatenated_hex = angle6 + angle5 + angle4 + angle3 + angle2 + angle1

                    # Step 2: Convert the concatenated hex string to a signed integer (2's complement)
                    decimal_value = int(concatenated_hex, 16)
                    #self.log_to_ui("INFO", f"Decimal value: {decimal_value}")
                    if decimal_value >= 2**(len(concatenated_hex) * 4 - 1):  # Check if the value is negative
                        decimal_value -= 2**(len(concatenated_hex) * 4)

                    # Step 3: Calculate the angle using the formula
                    angle = (decimal_value / (16384 * self.gear_ratios[motor_id])) * 360

                    # Save the motor angle
                    self.motor_angles[motor_id] = angle

                    # Update the respective LCD panel in the UI based on the motor ID
                    if motor_id == "1":
                        self.ui.motor1AngleLCD.display(angle)
                    elif motor_id == "2":
                        self.ui.motor2AngleLCD.display(angle)
                    elif motor_id == "3":
                        self.ui.motor3AngleLCD.display(angle)
                    elif motor_id == "4":
                        self.ui.motor4AngleLCD.display(angle)
                    elif motor_id == "5":
                        self.ui.motor5AngleLCD.display(angle)
                    elif motor_id == "6":
                        self.ui.motor6AngleLCD.display(angle)
                    else:
                        self.log_to_ui("WARN", f"Unknown motor ID: {motor_id}")
                except ValueError:
                    self.log_to_ui("ERROR", f"Failed to parse speed value: {speed_high} {speed_low}")  
            #else:
                #self.log_to_ui("INFO", f"Motor {motor_id} motion feedback received: {feedback}")
        else:
            self.log_to_ui("WARN", f"Invalid feedback format: {feedback}")
            return

    # Function to start the autonomous mode process in a separate thread
    def auto_callback(self, msg):
        self.log_to_ui("INFO", "Starting autonomous pick and place...")
        self.message = msg.data
        self.start_go_auto_thread()


    # <------------------------------------------>
    #           MOVEMENT BUTTON FUNCTIONS
    # <------------------------------------------>
    #6/26: added helper function to pause if human detected
    def pause_if_human_detected(self):
        if self.is_paused:
            self.log_to_ui("WARN", "⏸ Paused due to human detection. Waiting to resume...")
        while self.is_paused:
            time.sleep(0.5)
    #6/26 : added helper function to disable motor buttons
    def finish_homing_abort(self):
        self.homing = False
        self.enable_motor_buttons()
        self.log_to_ui("INFO", "Homing stopped due to human detection. Please restart homing sequence.")

    def abort_if_human_detected(self):
        if self.ui.humanCheckBox.isChecked() and self.person_detected_flag == 1:
            self.log_to_ui("WARN", "Human detected during homing. Aborting.")
            self.finish_homing_abort()
            raise Exception("Homing aborted due to human detection.")

    #6/26: added helper function to wait with pause
    def wait_with_pause(self, seconds):
        waited = 0
        while waited < seconds:
            if self.is_paused:
                self.log_to_ui("WARN", "⏸ Paused during autonomous action.")
            while self.is_paused:
                time.sleep(0.5)
            time.sleep(0.5)
            waited += 0.5

    # Generalized function to handle motor commands
    def on_motor_command_pressed(self, command):
        msg = String()
        msg.data = command
        self.control_pub.publish(msg)
    #6/26 commented 
    # Function to handle motor command release
    # def on_motor_command_released(self, stop_command):
    #     msg = String()
    #     msg.data = stop_command
    #     self.node.create_publisher(String, '/aims/control', 10).publish(msg)
    def on_motor_command_released(self, stop_command):
        msg = String()
        msg.data = stop_command
        self.control_pub.publish(msg)

    # Function to go to home
    def go_home(self):
        # Send the CANBUS command to move the motor in the specified direction
        self.last_command = "home"
        self.control_pub.publish(String(data="home"))
        self.control_pub.publish(String(data="led_red"))
        self.disable_motor_buttons()  # Disable all motor buttons
        self.in_position_mode = True  # Set in position mode to True
        # Create an array of all 6 motor id numbers
        motors_stopped = [False] * 5
        previous_angles = [None] * 5
        stability_counts = [0] * 5
        required_stability = 3

        while True:
            #6/26: added pause if human detected
            self.pause_if_human_detected()
            if all(motors_stopped):
                self.control_pub.publish(String(data="led_green"))
                self.enable_motor_buttons()
                self.in_position_mode = False
                return
            else:
                for i in range(5):
                    motor_id = str(i + 1)
                    angle_feedback = self.motor_angles.get(motor_id)

                    if angle_feedback is None:
                        self.log_to_ui("ERROR", f"Failed to get angle feedback for motor {motor_id}.")
                        return

                    if previous_angles[i] is not None and abs(angle_feedback - previous_angles[i]) < 0.1:
                        stability_counts[i] += 1
                        if stability_counts[i] >= required_stability:
                            motors_stopped[i] = True
                            stop_cmd = "stop5_6" if motor_id == "5" else f"stop{motor_id}"
                            self.control_pub.publish(String(data=stop_cmd))
                    else:
                        stability_counts[i] = 0

                    previous_angles[i] = angle_feedback

            time.sleep(0.5)

    #6/26 added debugging version of go_scan function
    def go_scan(self):
        self.log_to_ui("INFO", "Starting go_scan() sequence...")

        time.sleep(0.5)  # Let bus settle before command spam

        self.scanning = True
        self.last_command = "scan"

        self.log_to_ui("DEBUG", "Publishing 'scan' command")
        self.control_pub.publish(String(data="scan"))
        time.sleep(0.25)  # Allow bus to process 'scan'

        self.log_to_ui("DEBUG", "Publishing 'led_red' command")
        self.control_pub.publish(String(data="led_red"))
        time.sleep(0.25)

        self.disable_motor_buttons()
        self.in_position_mode = True
        self.log_to_ui("DEBUG", f"is_paused: {self.is_paused}, person_detected_flag: {self.person_detected_flag}")

        motors_stopped = [False] * 5
        previous_angles = [None] * 5
        stability_counts = [0] * 5
        required_stability = 3

        start_time = time.time()
        max_duration = 30  # fail-safe timeout

        while True:
            #  Timeout to avoid indefinite hanging
            if time.time() - start_time > max_duration:
                self.log_to_ui("ERROR", " Scan timed out.")
                self.in_position_mode = False
                self.enable_motor_buttons()
                return

            self.pause_if_human_detected()

            if all(motors_stopped):
                self.control_pub.publish(String(data="led_green"))
                self.enable_motor_buttons()
                self.in_position_mode = False
                self.log_to_ui("INFO", "Scan complete.")
                return
            else:
                for i in range(5):
                    motor_id = str(i + 1)
                    angle_feedback = self.motor_angles.get(motor_id)
                    
                    if angle_feedback is None:
                        self.log_to_ui("ERROR", f"Failed to get angle feedback for motor {motor_id}.")
                        return

                    if previous_angles[i] is not None and abs(angle_feedback - previous_angles[i]) < 0.1:
                        stability_counts[i] += 1
                        if stability_counts[i] == required_stability:  # Only send stop once
                            motors_stopped[i] = True
                            stop_cmd = "stop5_6" if motor_id == "5" else f"stop{motor_id}"
                            self.control_pub.publish(String(data=stop_cmd))
                            self.log_to_ui("DEBUG", f"Sent {stop_cmd} for motor {motor_id}")
                            time.sleep(0.1)  # Avoid spamming CAN
                    else:
                        stability_counts[i] = 0

                    previous_angles[i] = angle_feedback

            time.sleep(0.5)

    # Function to go test position
    def go_test(self):
        # Send the CANBUS command to move the motor in the specified direction
        self.last_command = "test"
        self.control_pub.publish(String(data="test"))
        self.control_pub.publish(String(data="gripper_open"))
        self.control_pub.publish(String(data="led_red"))
        self.disable_motor_buttons()  # Disable all motor buttons
        self.in_position_mode = True  # Set in position mode to True
        # Create an array of all 6 motor id numbers
        motors_stopped = [False] * 5
        previous_angles = [None] * 5
        stability_counts = [0] * 5
        required_stability = 3

        while True:
            #6/26: added pause if human detected
            self.pause_if_human_detected()
            if all(motors_stopped):
                self.control_pub.publish(String(data="led_green"))
                self.enable_motor_buttons()
                self.in_position_mode = False
                return
            else:
                for i in range(5):
                    motor_id = str(i + 1)
                    angle_feedback = self.motor_angles.get(motor_id)

                    if angle_feedback is None:
                        self.log_to_ui("ERROR", f"Failed to get angle feedback for motor {motor_id}.")
                        return

                    if previous_angles[i] is not None and abs(angle_feedback - previous_angles[i]) < 0.1:
                        stability_counts[i] += 1
                        if stability_counts[i] >= required_stability:
                            motors_stopped[i] = True
                            stop_cmd = "stop5_6" if motor_id == "5" else f"stop{motor_id}"
                            self.control_pub.publish(String(data=stop_cmd))
                    else:
                        stability_counts[i] = 0

                    previous_angles[i] = angle_feedback

            time.sleep(0.5)

    # Function to go test position
    def go_place(self):
        self.last_command = "place"
        self.control_pub.publish(String(data="test"))
        self.control_pub.publish(String(data="gripper_close"))
        self.control_pub.publish(String(data="led_red"))
        self.disable_motor_buttons()  # Disable all motor buttons
        self.in_position_mode = True  # Set in position mode to True
        # Create an array of all 6 motor id numbers
        motors_stopped = [False] * 5
        previous_angles = [None] * 5
        stability_counts = [0] * 5
        required_stability = 3

        while True:
            #6/26: added pause if human detected
            self.pause_if_human_detected()
            if all(motors_stopped):
                self.control_pub.publish(String(data="led_green"))
                self.enable_motor_buttons()
                self.in_position_mode = False
                return
            else:
                for i in range(5):
                    motor_id = str(i + 1)
                    angle_feedback = self.motor_angles.get(motor_id)

                    if angle_feedback is None:
                        self.log_to_ui("ERROR", f"Failed to get angle feedback for motor {motor_id}.")
                        return

                    if previous_angles[i] is not None and abs(angle_feedback - previous_angles[i]) < 0.1:
                        stability_counts[i] += 1
                        if stability_counts[i] >= required_stability:
                            motors_stopped[i] = True
                            stop_cmd = "stop5_6" if motor_id == "5" else f"stop{motor_id}"
                            self.control_pub.publish(String(data=stop_cmd))
                    else:
                        stability_counts[i] = 0

                    previous_angles[i] = angle_feedback

            time.sleep(0.5)

    def find_home(self):
        try:
            self.log_to_ui("INFO", "Starting homing sequence...")
            self.control_pub.publish(String(data="led_red"))
            self.disable_motor_buttons()
            self.homing = True

            end_stop_directions = {
                "motor1": "motor1_left",
                "motor2": "motor2_up",
                "motor3": "motor3_down",
                "motor4": "motor4_counterclockwise",
                "motor5": "motor5_6_down",
            }

            for motor, direction in end_stop_directions.items():
                self.abort_if_human_detected()

                self.control_pub.publish(String(data=f"set_home_{motor[-1]}"))
                self.log_to_ui("INFO", f"Finding {motor} end stop by moving {direction}...")
                self.move_to_end_stop(motor, direction)

                self.abort_if_human_detected()
                self.log_to_ui("INFO", f"Moving {motor} to home position...")
                self.move_to_actual_home(motor)

                self.abort_if_human_detected()
                self.control_pub.publish(String(data=f"set_home_{motor[-1]}"))

            self.hasBeenHomed = True    
            self.homing = False
            self.ui.goHomeButton.setEnabled(True)
            self.ui.goScanButton.setEnabled(True)
            self.ui.goTestButton.setEnabled(True)
            self.ui.goPlaceButton.setEnabled(True)
            self.ui.goPositionButton.setEnabled(True)
            self.enable_motor_buttons()
            time.sleep(8)
            self.control_pub.publish(String(data="led_green"))
            self.control_pub.publish(String(data="set_home_5"))
            self.log_to_ui("INFO", "Home position calibrated.")

        except Exception as e:
            # Already handled in abort_if_human_detected(), just log here
            self.log_to_ui("ERROR", str(e))

#6/26 added new version of move_to_end_stop function to handle human detection and pauses
    def move_to_end_stop(self, motor, direction):
        self.abort_if_human_detected()

        self.control_pub.publish(String(data=direction))
        time.sleep(0.5)
        previous_angle = None

        while True:
            self.abort_if_human_detected()

            angle_feedback = self.motor_angles.get(f"{motor[-1]}")
            if angle_feedback is None:
                self.log_to_ui("ERROR", f"Failed to get angle feedback for {motor}.")
                return

            if previous_angle is not None and abs(angle_feedback - previous_angle) < 0.02:
                self.log_to_ui("INFO", f"{motor} hit end stop at angle: {angle_feedback:.2f}")
                if motor == "motor5":
                    self.control_pub.publish(String(data="stop5_6"))
                    time.sleep(1)
                    self.control_pub.publish(String(data="set_home_5"))
                else:
                    self.control_pub.publish(String(data=f"stop{motor[-1]}"))
                    time.sleep(1)
                    self.control_pub.publish(String(data=f"set_home_{motor[-1]}"))
                time.sleep(1)
                break

            previous_angle = angle_feedback
            time.sleep(0.2 if motor == "motor5" else 1)

    def move_to_actual_home(self, motor):
        self.abort_if_human_detected()

        end_stop_angle = self.motor_angles.get(f"{motor[-1]}")
        if end_stop_angle is None:
            self.log_to_ui("ERROR", f"Failed to get angle feedback for {motor}.")
            return

        if motor == "motor5":
            self.control_pub.publish(String(data="offset_5"))
            self.control_pub.publish(String(data="offset_6"))
        else:
            self.control_pub.publish(String(data=f"offset_{motor[-1]}"))

        previous_angle = None
        while True:
            self.abort_if_human_detected()

            angle_feedback = self.motor_angles.get(f"{motor[-1]}")
            if angle_feedback is None:
                self.log_to_ui("ERROR", f"Failed to get angle feedback for {motor}.")
                return

            if previous_angle is not None and abs(angle_feedback - previous_angle) < 0.1:
                self.log_to_ui("INFO", f"{motor} hit home position at angle: {angle_feedback:.2f}")
                if motor == "motor5":
                    self.control_pub.publish(String(data="stop5_6"))
                    time.sleep(1)
                    self.control_pub.publish(String(data="set_home_5"))
                else:
                    self.control_pub.publish(String(data=f"stop{motor[-1]}"))
                    time.sleep(1)
                    self.control_pub.publish(String(data=f"set_home_{motor[-1]}"))
                time.sleep(1)
                break

            previous_angle = angle_feedback
            time.sleep(0.1 if motor == "motor5" else 1)

    # Function to go to a specific position
    def go_position(self):
        x = self.ui.xLineEdit.text()
        y = self.ui.yLineEdit.text()
        # Check if the input values are valid numbers
        if not x or not y:
            self.log_to_ui("WARN", "Please enter valid X and Y coordinates.")
            return
        try:
            x = float(x)
            y = float(y)
        except ValueError:
            self.log_to_ui("ERROR", "Invalid input. X and Y must be numeric values.")
            return
        # Send the CANBUS command to move the motor in the specified direction
        try:
            # Check if x and y are within the range of -6 to 6 and -6 to 5 respectively
            if not (-6 <= float(x) <= 6 and -6 <= float(y) <= 5):
                self.log_to_ui("WARN", "X and Y coordinates must be in the range of -6 to 6 and -6 to 5 respectively.")
                return
            # if x and y are integer values and in the range of (inclusive) -6 <= x <= 6 and -6 <= y <= 5, publish "known" to /aims/x_y_position
            elif (x.is_integer() and y.is_integer()):
                # Publish "known" with the integer x and y values
                self.last_command = f"known {int(x)} {int(y)}"
                self.control_pub.publish(String(data=f"known {int(x)} {int(y)}"))
            else: # if x and y are not an exact integer value (they are double or float) in the range of (inclusive) -6 <= x <= 6 and -6 <= y <= 5, publish "unknown" to /aims/x_y_position
                # Publish "unknown" with the float x and y values
                self.last_command = "unknown {x} {y}"
                self.control_pub.publish(String(data=f"unknown {x} {y}"))
        except ValueError:
            self.log_to_ui("ERROR", "Invalid input. X, Y, and Z must be numeric values.")
            return
        

        self.control_pub.publish(String(data="led_red"))
        self.disable_motor_buttons()  # Disable all motor buttons
        self.in_position_mode = True  # Set in position mode to True
        # Create an array of all 6 motor id numbers
        motors_stopped = [False] * 5
        previous_angles = [None] * 5
        stability_counts = [0] * 5
        required_stability = 3

        while True:
            #6/26: added pause if human detected
            self.pause_if_human_detected()
            if all(motors_stopped):
                self.control_pub.publish(String(data="led_green"))
                self.enable_motor_buttons()
                self.in_position_mode = False
                return
            else:
                for i in range(5):
                    motor_id = str(i + 1)
                    angle_feedback = self.motor_angles.get(motor_id)

                    if angle_feedback is None:
                        self.log_to_ui("ERROR", f"Failed to get angle feedback for motor {motor_id}.")
                        return

                    if previous_angles[i] is not None and abs(angle_feedback - previous_angles[i]) < 0.1:
                        stability_counts[i] += 1
                        if stability_counts[i] >= required_stability:
                            motors_stopped[i] = True
                            stop_cmd = "stop5_6" if motor_id == "5" else f"stop{motor_id}"
                            self.control_pub.publish(String(data=stop_cmd))
                    else:
                        stability_counts[i] = 0

                    previous_angles[i] = angle_feedback

            time.sleep(0.5)

    #6/26 new go_auto function that uses the new pause_if_human_detected and wait_with_pause functions - this is missing a pick 
    def go_auto(self):
        """
        FSM for:  gripper_open → move-to-PCB → gripper_close → half_raise
                → place → gripper_open → place_up → scan → done
        Pauses safely on human detection; resumes same step.
        """
        if self.go_auto_running:
            return
        self.go_auto_running = True

        # initialise
        self.last_auto_sequence = max(self.last_auto_sequence, 0)
        self.disable_motor_buttons()
        self.in_position_mode = True
        self.last_command = "auto"
        self.control_pub.publish(String(data="led_red"))

        try:
            while rclpy.ok() and self.last_auto_sequence != -1:

                # freeze while paused
                if self.is_paused:
                    time.sleep(0.1)
                    continue

                s = self.last_auto_sequence

                if s == 0:                           # 0 ▸ open gripper
                    self.control_pub.publish(String(data="gripper_open"))
                    self.wait_with_pause(1)
                    self.log_to_ui("INFO", "Gripper opened.")
                    self.last_auto_sequence = 1

                elif s == 1:                         # 1 ▸ move to pick
                    self.control_pub.publish(String(data=self.message))  # interpolated CAN macro
                    self.wait_for_stop()
                    self.log_to_ui("INFO", "Reached pick position.")
                    self.last_auto_sequence = 2

                elif s == 2:                         # 2 ▸ close gripper
                    self.control_pub.publish(String(data="gripper_close"))
                    self.wait_with_pause(1)
                    self.log_to_ui("INFO", "Gripper closed.")
                    self.last_auto_sequence = 3

                elif s == 3:                         # 3 ▸ half-raise (formerly “pick_up”)
                    self.control_pub.publish(String(data="half_raise"))     # or "half_raise"
                    self.wait_for_stop()
                    self.log_to_ui("INFO", "Half-raise complete.")
                    self.last_auto_sequence = 4

                elif s == 4:                         # 4 ▸ move to place
                    self.control_pub.publish(String(data="place"))
                    self.wait_for_stop()
                    self.log_to_ui("INFO", "Reached place position.")
                    self.last_auto_sequence = 5

                elif s == 5:                         # 5 ▸ release
                    self.control_pub.publish(String(data="gripper_open"))
                    self.wait_with_pause(1)
                    self.log_to_ui("INFO", "Released PCB.")
                    self.last_auto_sequence = 6

                elif s == 6:                         # 6 ▸ place_up (lift)
                    self.control_pub.publish(String(data="place_up"))
                    self.wait_for_stop()
                    self.log_to_ui("INFO", "Lifted from place position.")
                    self.last_auto_sequence = 7

                elif s == 7:                         # 7 ▸ scan
                    self.control_pub.publish(String(data="scan"))
                    self.wait_for_stop()
                    self.log_to_ui("INFO", "Returned to scan position.")
                    self.last_auto_sequence = 8

                elif s == 8:                         # 8 ▸ done
                    self.control_pub.publish(String(data="led_green"))
                    self.wait_with_pause(0.5)
                    self.is_picking_pub.publish(Bool(data=False))
                    self.enable_motor_buttons()
                    self.in_position_mode = False
                    self.last_auto_sequence = -1

        finally:
            self.go_auto_running = False

           


    # Function to update the speed
    def update_speed(self, value):
        msg = Int32()
        msg.data = value  # Publish the slider value as an integer
        self.node.create_publisher(Int32, '/aims/speed', 10).publish(msg)

    def disable_motor_buttons(self):
        # Disable all motor buttons
        self.ui.motor1LeftButton.setEnabled(False)
        self.ui.motor1RightButton.setEnabled(False)
        self.ui.motor2UpButton.setEnabled(False)
        self.ui.motor2DownButton.setEnabled(False)
        self.ui.motor3UpButton.setEnabled(False)
        self.ui.motor3DownButton.setEnabled(False)
        self.ui.motor4CwiseButton.setEnabled(False)
        self.ui.motor4CCwiseButton.setEnabled(False)
        self.ui.motor5_6UpButton.setEnabled(False)
        self.ui.motor5_6DownButton.setEnabled(False)
        self.ui.motor5_6CwiseButton.setEnabled(False)
        self.ui.motor5_6CCwiseButton.setEnabled(False)
        self.ui.gripperAdvOpenButton.setEnabled(False)
        self.ui.gripperAdvCloseButton.setEnabled(False)
        self.ui.speedSlider.setEnabled(False)
        self.ui.autonomousCheckBox.setEnabled(False)  # False autonomous mode checkbox

    def enable_motor_buttons(self):
        # Enable all motor buttons
        self.ui.motor1LeftButton.setEnabled(True)
        self.ui.motor1RightButton.setEnabled(True)
        self.ui.motor2UpButton.setEnabled(True)
        self.ui.motor2DownButton.setEnabled(True)
        self.ui.motor3UpButton.setEnabled(True)
        self.ui.motor3DownButton.setEnabled(True)
        self.ui.motor4CwiseButton.setEnabled(True)
        self.ui.motor4CCwiseButton.setEnabled(True)
        self.ui.motor5_6UpButton.setEnabled(True)
        self.ui.motor5_6DownButton.setEnabled(True)
        self.ui.motor5_6CwiseButton.setEnabled(True)
        self.ui.motor5_6CCwiseButton.setEnabled(True)
        self.ui.gripperAdvOpenButton.setEnabled(True)
        self.ui.gripperAdvCloseButton.setEnabled(True)
        self.ui.speedSlider.setEnabled(True)
        self.ui.autonomousCheckBox.setEnabled(True)  # True autonomous mode checkbox
 
    # <------------------------------------------>
    #           GENERAL BUTTON FUNCTIONS
    # <------------------------------------------>

    # Function to handle the start ROS button click
    def on_start_ros_clicked(self):
        try:
            # Get the path to the launch file using ament_index_python
            launch_file = os.path.join(get_package_share_directory('aims_controller'), 'launch', 'aims_launch.py')

            # Start the launch file using subprocess
            self.ros_process = subprocess.Popen(['ros2', 'launch', launch_file])
            self.log_to_ui("INFO", "Started ROS nodes using the launch file.")

            # Update button states
            self.ui.startROSButton.setEnabled(False)  # Disable the Start button
            self.ui.stopROSButton.setEnabled(True)   # Enable the Stop button
            self.ui.joystickCheckBox.setEnabled(True)  # Enable joystick mode checkbox
            self.ui.speedSlider.setEnabled(True)  # Enable the speed slider
            self.ui.feedbackCheckBox.setEnabled(True)  # True feedback mode checkbox
            self.ui.humanCheckBox.setEnabled(True)  # True human mode checkbox
            self.ui.startRobotCamButton.setEnabled(True) # True
            self.ui.startRobotCamButton.setToolTip("")
            self.enable_motor_buttons()  # Enable motor buttons
            if self.hasBeenHomed:
                self.ui.goHomeButton.setEnabled(True) # True
                self.ui.goScanButton.setEnabled(True) # True
                self.ui.goTestButton.setEnabled(True) # True
                self.ui.goPlaceButton.setEnabled(True) # True
            else:
                self.ui.goHomeButton.setEnabled(False) # False
                self.ui.goScanButton.setEnabled(False) # False
                self.ui.goTestButton.setEnabled(False) # False
                self.ui.goPlaceButton.setEnabled(False) # False

        except Exception as e:
            self.log_to_ui("ERROR", f"Failed to start ROS nodes: {e}")

    # Function to handle the Stop ROS button click
    def on_stop_ros_clicked(self):
        try:
            # Publish a shutdown command to the /aims/shutdown topic
            shutdown_msg = Bool()
            shutdown_msg.data = True
            self.shutdown_pub.publish(shutdown_msg)
            # self.log_to_ui("INFO", "Published shutdown command to /aims/shutdown.")

            # Terminate the USB camera process
            if self.background_cam_process:
                self.background_cam_process.terminate()
                try:
                    self.background_cam_process.wait(timeout=5)  # Wait for up to 5 seconds
                except subprocess.TimeoutExpired:
                    self.background_cam_process.kill()  # Forcefully kill the process
                self.background_cam_process = None
                self.log_to_ui("INFO", "Background camera node stopped.")

            # Terminate the robot camera process
            if self.robot_cam_process:
                self.robot_cam_process.terminate()
                try:
                    self.robot_cam_process.wait(timeout=5)  # Wait for up to 5 seconds
                except subprocess.TimeoutExpired:
                    self.robot_cam_process.kill()  # Forcefully kill the process
                self.robot_cam_process = None
                self.log_to_ui("INFO", "Robot camera node stopped.")

            # Terminate the ROS process
            if self.ros_process:
                self.ros_process.terminate()
                self.ros_process.wait()
                self.ros_process = None
                self.log_to_ui("INFO", "ROS nodes stopped.")

            subprocess.run(["pkill", "-f", "usb_cam_node_exe"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            # Update button states
            self.ui.startROSButton.setEnabled(True)  # Enable the Start button
            self.ui.stopROSButton.setEnabled(False)  # Disable the Stop button
            self.ui.joystickCheckBox.setEnabled(False)  # False joystick mode checkbox
            self.ui.speedSlider.setEnabled(False)  # Disable the speed slider
            self.ui.feedbackCheckBox.setEnabled(False)  # False feedback mode checkbox
            self.ui.humanCheckBox.setEnabled(False)  # False human mode checkbox
            self.ui.humanCheckBox.setChecked(False)  # Uncheck the human mode checkbox
            self.ui.joystickCheckBox.setChecked(False)  # Uncheck the joystick mode checkbox
            self.ui.feedbackCheckBox.setChecked(False)  # Uncheck the feedback mode checkbox
            self.ui.startRobotCamButton.setEnabled(False) # False
            self.ui.startRobotCamButton.setToolTip("Nodes must be started in order to start the Robot Cam.")
            self.disable_motor_buttons()  # Disable motor buttons
            self.ui.findHomeButton.setEnabled(False) # False
            self.ui.goHomeButton.setEnabled(False) # False
            self.ui.goScanButton.setEnabled(False) # False
            self.ui.goTestButton.setEnabled(False) # False
            self.ui.goPlaceButton.setEnabled(False) # False

        except Exception as e:
            self.log_to_ui("ERROR", f"Failed to stop ROS nodes: {e}")

    # Function to start the USB camera and rqt_image_view plugin
    def on_start_camera_clicked(self):
        self.start_background_camera()

    # Function to start the robot camera and its image view
    def on_start_robot_camera_clicked(self):
        self.start_robot_camera()

    # Function to send CANBUS messages
    def on_send_can_clicked(self):
        # Get the CAN message from the QLineEdit
        self.global_can = self.ui.canLineEdit.text().strip()

        # Validate the CAN message
        if not self.global_can:
            self.log_to_ui("WARN", "No CAN message entered.")
            return

        if len(self.global_can) % 2 != 0 or not all(c in "0123456789ABCDEF" for c in self.global_can):
            self.log_to_ui("ERROR", "Invalid CAN message format. Must be a valid hexadecimal string.")
            return

        # Start the serial connection and send the CAN message
        thread = threading.Thread(target=self.start_serial_connection)
        thread.start()

    # Function to start the serial connection and send one test CAN message
    def start_serial_connection(self):
        possible_ports = ["/dev/ttyACM0"]  # List of possible serial ports
        baudrate = 115200  # Change to your device's baud rate

        for port in possible_ports:
            try:
                self.log_to_ui("INFO", f"Attempting to connect to serial port: {port}")
                with serial.Serial(port, baudrate, timeout=1) as ser:
                    self.log_to_ui("INFO", f"Successfully connected to serial port: {port}")

                    if self.person_detected_flag == 0:  # No person detected
                        # Example CANBUS message to spin the motor
                        can_message = self.global_can  # Replace with desired CANBUS message
                        crc = calculate_crc([int(can_message[i:i+2], 16) for i in range(0, len(can_message), 2)])
                        
                        # Log the CRC value for debugging
                        self.log_to_ui("INFO", f"Calculated CRC: {crc:02X}")
                        
                        can_message_with_crc = can_message + format(crc, '02X')

                        self.log_to_ui("INFO", f"Sending CAN message: {can_message_with_crc}")
                        ser.write((can_message_with_crc + '\n').encode())  # Send CAN message

                        response = ser.readline().decode().strip()
                        if response:
                            self.log_to_ui("INFO", f"Received: {response}")
                        else:
                            self.log_to_ui("WARN", f"No response received for CAN message: {can_message_with_crc}")
                    elif self.person_detected_flag == 1 and self.ui.humanCheckBox.isChecked():
                        # If a person is detected and the human checkbox is checked, don't send the CAN message
                        self.log_to_ui("INFO", "Person detected and human detection on. No CAN message sent.")
                    return  # Exit the loop once a successful connection is made

            except serial.SerialException as e:
                self.log_to_ui("ERROR", f"Failed to connect to serial port {port}: {e}")
            except Exception as e:
                self.log_to_ui("ERROR", f"Unexpected error while connecting to {port}: {e}")

        # If no ports are available
        self.log_to_ui("ERROR", "Failed to connect to any serial port. Please check the device connection.")

    # Function to handle the joystick checkbox state change
    def on_joystick_checkbox_changed(self, state):
        msg = Bool()
        msg.data = state == Qt.Checked  # True if checked, False otherwise
        self.joystick_enabled_pub.publish(msg)

        if state == Qt.Checked:  # Joystick enabled
            self.log_to_ui("INFO", "Joystick control enabled. Disabling motor buttons.")
            self.disable_motor_buttons()
        else:  # Joystick disabled
            self.log_to_ui("INFO", "Joystick control disabled.")
            self.enable_motor_buttons()

    # Function to handle the human checkbox state change
    def on_human_checkbox_changed(self, state):
        if state == Qt.Checked: # Publish to /aims/stop_on_human_detected topic
            # publish true to /aims/stop_on_human_detected topic
            msg = Bool()
            msg.data = True
            self.stop_on_human_detected_pub.publish(msg)
        if state == Qt.Unchecked: # Publish to /aims/stop_on_human_detected topic
            # publish false to /aims/stop_on_human_detected topic
            msg = Bool()
            msg.data = False
            self.stop_on_human_detected_pub.publish(msg)
            self.enable_motor_buttons()  # Re-enable motor buttons when human detection is off

    # Function to handle the autonomous checkbox state change
    def on_autonomous_checkbox_changed(self, state):
        if state == Qt.Checked:
            msg = Bool()
            msg.data = True
            self.autonomous_pub.publish(msg)
        if state == Qt.Unchecked:
            msg = Bool()
            msg.data = False
            self.autonomous_pub.publish(msg)

    # <------------------------------------------>
    #              CAMERA FUNCTIONS
    # <------------------------------------------>

    # Function to start the background camera node
    def start_background_camera(self):
        try:
            # Check if the background camera node is already running
            active_nodes = self.node.get_node_names()
            if '/background_cam' not in active_nodes:
                # Get the path to the launch file using ament_index_python
                launch_file = os.path.join(get_package_share_directory('aims_controller'), 'launch', 'background_cam_launch.py')

                # Start the launch file using subprocess
                self.background_cam_process = subprocess.Popen(['ros2', 'launch', launch_file])
                self.log_to_ui("INFO", "Background camera node started successfully!")
                self.background_cam_started = True  # Set the flag to True
                self.ui.startCameraButton.setEnabled(False)
                self.ui.startCameraButton.setToolTip("Background camera is running. Please stop it before starting the background camera.")
            else:
                self.log_to_ui("WARN", "Background camera node is already running.")
        except Exception as e:
            self.log_to_ui("ERROR", f"Failed to start background camera node: {e}")
        # Monitor the process in a separate thread
        threading.Thread(target=self.monitor_background_cam_process, daemon=True).start()

    # Function to monitor the rqt_image_view process for the background camera
    def monitor_background_cam_process(self):
        if self.background_cam_process:
            self.background_cam_process.wait()  # Wait for the process to exit
            #self.log_to_ui("INFO", "Node for background camera has stopped.")
            self.background_cam_process = None
            self.background_cam_started = False

            # Re-enable the start button and clear the tooltip
            self.ui.startCameraButton.setEnabled(True)
            self.ui.startCameraButton.setToolTip("")

    # Function to start the robot camera node
    def start_robot_camera(self):
        try:
            # Check if the robot camera node is already running
            active_nodes = self.node.get_node_names()
            if '/robot_cam' not in active_nodes:
                # Get the path to the launch file using ament_index_python
                launch_file = os.path.join(get_package_share_directory('aims_controller'), 'launch', 'robot_cam_launch.py')

                # Start the launch file using subprocess
                self.robot_cam_process = subprocess.Popen(['ros2', 'launch', launch_file])
                self.log_to_ui("INFO", "Robot camera node started successfully!")
                self.robot_cam_started = True  # Set the flag to True
                self.ui.startRobotCamButton.setEnabled(False)
                self.ui.startRobotCamButton.setToolTip("Robot camera is running. Please stop it before starting the background camera.")
            else:
                self.log_to_ui("WARN", "Robot camera node is already running.")
        except Exception as e:
            self.log_to_ui("ERROR", f"Failed to start robot camera node: {e}")
        # Monitor the process in a separate thread
        threading.Thread(target=self.monitor_robot_cam_process, daemon=True).start()

    # Function to monitor the rqt_image_view process for the robot camera
    def monitor_robot_cam_process(self):
        if self.robot_cam_process:
            self.robot_cam_process.wait()  # Wait for the process to exit
            #self.log_to_ui("INFO", "rqt_image_view plugin for robot camera has exited.")
            self.robot_cam_process = None
            self.robot_cam_started = False

            # Re-enable the start button and clear the tooltip
            self.ui.startRobotCamButton.setEnabled(True)
            self.ui.startRobotCamButton.setToolTip("")

# <------------------------------------------
#MD ADDED FOR MOTOR CAPTURE 
    def log_pick_angles(self):
        try:
            x = self.ui.xLineEdit_2.text().strip()
            y = self.ui.yLineEdit_2.text().strip()

            if not x or not y:
                self.log_to_ui("ERROR", "Please enter X and Y grid coordinates first.")
                return

            grid_key = f"{x},{y}"

            # Extract angles from UI-tracked motor data
            angles = [round(self.motor_angles[str(i + 1)], 2) for i in range(6)]

            # Fixed path (no popup)
            save_path = os.path.expanduser("~/aims_ws/src/aims_controller/aims_controller/grid_to_joint_angles.json")

            # Load existing data
            if os.path.exists(save_path):
                with open(save_path, 'r') as f:
                    data = json.load(f)
            else:
                data = {}

            # Store data
            data[grid_key] = angles

            with open(save_path, 'w') as f:
                json.dump(data, f, indent=2)

            self.log_to_ui("INFO", f"Saved angles for grid cell ({grid_key}): {angles}")

            # Publish current x and y coordinates to the /aims/log_can topic
            log_msg = String()
            log_msg.data = f"{grid_key}"
            self.log_can_pub.publish(log_msg)

        except Exception as e:
            self.log_to_ui("ERROR", f"Failed to log angles: {e}")

    # <------------------------------------------>
    #             SHUTDOWN AND SETTINGS
    # <------------------------------------------>

    def shutdown_plugin(self):
        self.log_to_ui("INFO", "Shutting down AIMS plugin")

        # Stop the ROS spinning thread
        if self.executor is not None:
            self.executor.shutdown()
        if hasattr(self, 'ros_thread') and self.ros_thread.is_alive():
            self.ros_thread.join()  # Wait for the thread to finish

        # Terminate the ROS process
        if self.ros_process:
            self.ros_process.terminate()
            self.ros_process.wait()

        # Shutdown the ROS node
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
            if rclpy.ok():
                rclpy.shutdown()

        # Reset the instance_created flag
        AIMS.instance_created = False

    # Function to save and restore settings
    def save_settings(self, plugin_settings, instance_settings):
        pass

    # Function to save and restore settings
    def restore_settings(self, plugin_settings, instance_settings):
        pass

    #6/26: added helper function to wait for motors to stop moving
    def wait_for_stop(self):
        """
        Block until ALL five tracked joints have remained within ±0.05 °
        for `required_stability` consecutive samples **while the robot is NOT
        paused**.  If a human-detection pause happens mid-motion, we wipe all
        counters so the test restarts after the resume and a fresh motion
        must stabilise before we return.
        """
        required_stability = 10        # how many stable readings per motor
        tolerance          = 0.05      # deg
        sample_dt          = 0.5       # sec between polls

        motors_stopped   = [False] * 5
        stability_counts = [0] * 5
        previous_angles  = [None] * 5

        # helper: reset all counters and take a new baseline
        def _reset_trackers():
            nonlocal motors_stopped, stability_counts, previous_angles
            motors_stopped   = [False] * 5
            stability_counts = [0] * 5
            for i in range(5):
                motor_id = str(i + 1)
                previous_angles[i] = self.motor_angles.get(motor_id)

        # --- initial baseline before the first motion starts -----------
        time.sleep(1.0)                # let motors spin up
        _reset_trackers()
        time.sleep(0.5)                # extra settling

        # ---------------- Main monitoring loop -------------------------
        while rclpy.ok():
            # 1) If we are PAUSED → just idle, but *also* wipe counts so
            #    we require a fresh stability window after resume.
            while self.is_paused:
                time.sleep(0.1)
                _reset_trackers()      # discard any pre-pause counts

            # 2) Check each joint for stability
            if all(motors_stopped):
                self.log_to_ui("DEBUG", "All motors detected as stopped")
                return

            for i in range(5):
                motor_id = str(i + 1)
                angle = self.motor_angles.get(motor_id)
                if angle is None:
                    self.log_to_ui("ERROR",
                                   f"Failed to get angle feedback for motor {motor_id}.")
                    return

                if previous_angles[i] is not None and abs(angle - previous_angles[i]) < tolerance:
                    stability_counts[i] += 1
                    if stability_counts[i] >= required_stability:
                        motors_stopped[i] = True
                else:
                    stability_counts[i] = 0

                previous_angles[i] = angle

            time.sleep(sample_dt)