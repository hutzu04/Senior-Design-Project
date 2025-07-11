import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32, Float32  # Use Bool for the trigger message
import os
import serial  # Import serial for CAN BUS communication
import json  # Import json for parsing joystick speeds
from rqt_aims.send_g_code import calculate_crc
import threading
import signal
from geometry_msgs.msg import Point
import json
from pathlib import Path
import time

#6/26 moved these helper functions above the ControllerNode class to prevent circular import issues
def hex_to_signed_int24(hex_str):
    val = int(hex_str, 16)
    if val & 0x800000:
        val -= 0x1000000
    return val

def int_to_hex24(val):
    if val < 0:
        val += 0x1000000
    return f"{val:06X}"

def interpolate_can_commands(x, y, can_data):
    x0, x1 = int(x), int(x) + 1
    y0, y1 = int(y), int(y) + 1

    def key(x, y): return f"{x},{y}"
    for k in [key(x0, y0), key(x1, y0), key(x0, y1), key(x1, y1)]:
        if k not in can_data:
            raise ValueError(f"Missing key {k} in CAN data!")

    def extract_values(k):
        cmds = can_data[k]
        return [hex_to_signed_int24(cmd[-6:]) for cmd in cmds]

    Q11 = extract_values(key(x0, y0))
    Q21 = extract_values(key(x1, y0))
    Q12 = extract_values(key(x0, y1))
    Q22 = extract_values(key(x1, y1))

    dx = x - x0
    dy = y - y0

    result = []
    for i in range(6):
        interp_val = (
            Q11[i] * (1 - dx) * (1 - dy) +
            Q21[i] * dx * (1 - dy) +
            Q12[i] * (1 - dx) * dy +
            Q22[i] * dx * dy
        )
        int_val = round(interp_val)
        hex_val = int_to_hex24(int_val)
        motor_id = f"{i+1:02d}"
        can_cmd = f"{motor_id}FE0040AA{hex_val}"
        result.append(can_cmd)

    return result

class ControllerNode(Node):
    def __init__(self):

        super().__init__('controller_node')
        can_path = Path("/home/aims_user/aims_ws/src/aims_controller/aims_controller/grid_to_joint_can.json")
        with open(can_path, "r") as f:
            self.can_data = json.load(f)

        # Initialize the serial port and baud rate
        self.possible_ports = ["/dev/ttyACM0", "/dev/ttyACM1"]  # List of possible serial ports
        self.baudrate = 115200  # Baud rate for serial communication
        self.current_port = None  # Variable to store the current port
        self.max_speed = 0x040  # Maximum speed for the motors
        # make current_speed half of max_speed in hex
        self.current_speed = self.max_speed // 2  # Default speed (50% of max speed)
        # Publisher
        self.pub = self.create_publisher(String, '/aims/control', 10)
        # Publisher for log messages
        self.log_pub = self.create_publisher(String, '/aims/logs', 10)
        # Publisher for motor feedback
        self.feedback_pub = self.create_publisher(String, '/aims/motor_feedback', 10)

        # Subscriber to listen for person detection
        self.sub_person_detected = self.create_subscription(
            Int32,
            '/aims/person_detected',
            self.person_detected_callback,
            10
        )
        self.sub_stop_on_human_detected = self.create_subscription(
            Bool,
            '/aims/stop_on_human_detected',
            self.stop_on_human_detected_callback,
            10
        )
        # Subscriber to listen for speed updates
        self.sub_speed = self.create_subscription(
            Int32,
            '/aims/speed',
            self.speed_callback,
            10
        )
        # Subscriber to listen for G-Code file location
        self.sub_location = self.create_subscription(
            String,
            '/aims/g_code_location',
            self.location_callback,
            10
        )
        # Subscriber to listen for motor control commands
        self.sub_control = self.create_subscription(
            String,
            '/aims/control',
            self.control_callback,
            10
        )
        # Subscriber to listen for shutdown commands
        self.sub_shutdown = self.create_subscription(
            Bool,
            '/aims/shutdown',
            self.shutdown_callback,
            10
        )
        # Subscriber to listen for shutdown commands
        self.sub_serial = self.create_subscription(
            String,
            '/aims/serial_port',
            self.serial_port_callback,
            10
        )
        # Subscriber to enable/disable joystick input
        self.sub_joystick_enabled = self.create_subscription(
            Bool,
            '/aims/joystick_enabled',
            self.joystick_enabled_callback,
            10
        )
        # Subscribers to listen for joystick speed updates
        self.sub_joystick_speed_motor1 = self.create_subscription(
            Float32,
            '/aims/joystick_speed_motor1',
            lambda msg: self.update_motor_speed('motor1', msg.data),
            10
        )
        self.sub_joystick_speed_motor2 = self.create_subscription(
            Float32,
            '/aims/joystick_speed_motor2',
            lambda msg: self.update_motor_speed('motor2', msg.data),
            10
        )
        self.sub_joystick_speed_motor3 = self.create_subscription(
            Float32,
            '/aims/joystick_speed_motor3',
            lambda msg: self.update_motor_speed('motor3', msg.data),
            10
        )
        self.sub_joystick_speed_motor4 = self.create_subscription(
            Float32,
            '/aims/joystick_speed_motor4',
            lambda msg: self.update_motor_speed('motor4', msg.data),
            10
        )
        #MD ADDED 5/4 for pcb position from cv node
        self.sub_pcb_position = self.create_subscription(
            Point,
            '/aims/pcb_position',
            self.pcb_position_callback,
            10
        )
        self.sub_ik_commands = self.create_subscription(
            String,
            '/aims/ik_commands',
            self.cb_ik_commands,
            10
        )
        self.sub_log_can = self.create_subscription(
            String,
            '/aims/log_can',
            self.log_grid_position,
            10
        )

        # Acknowledge the node has started
        self.running = True

        # Check the serial ports immediately during initialization
        available_ports = [port for port in ["/dev/ttyACM0", "/dev/ttyACM1"] if os.path.exists(port)]
        if available_ports:
            # Serial connection is available
            self.current_port = available_ports[0]  # Use the first available port

        self.joystick_enabled = False  # Track if joystick is enabled
        self.joystick_speeds = {
            "motor1": 0.0,
            "motor2": 0.0,
            "motor3": 0.0,
            "motor4": 0.0,
        } # Initialize default speeds for joystick speeds
        self.precision_mode = False
        self.person_detected = 0  # Flag to track if a person is detected
        self.stop_on_human_detected = False  # Flag to track if stop on person detected is enabled

        # Initialize the CAN message map
        self.can_message_map = {}
        # Calculate the angle offset to the actual home position
        self.angle_offsets = {
            "1": -78.5,  # Example offset for motor 1
            "2": -51.6,  # Example offset for motor 2
            "3": -36.0,  # Example offset for motor 3
            "4": 0.0,   # Example offset for motor 4
            "5": 0.0,  # Example offset for motor 5
            "6": 0.0, # Example offset for motor 6
        }
        # State gear ratios for the 6 motors in an array
        self.gear_ratios = {
            "1": 13.5,
            "2": 150,
            "3": 150,
            "4": 48,
            "5": 67.82,
            "6": 67.82,
        }  # Example gear ratios for each motor
        
        # Print that the node has started
        self.log_to_ui("INFO", "Controller node started successfully.")

    #Captures the current can messages for the current angles of the motors and saves them to a JSON file
    #MD ADDED 5/8 for pcb position from cv node
    def log_grid_position(self, msg):
        # parse the grid cell from the message
        grid_cell = msg.data
        if not grid_cell:
            self.log_to_ui("ERROR", "Invalid grid cell received.")
            return
        
        #Parse the grid cell coordinates in the string, they are separated by a space
        grid_x, grid_y = map(int, grid_cell.split(","))
        
        self.log_to_ui("INFO", f"Logging angles and CAN positions for grid cell ({grid_x}, {grid_y})...")

        motor_ids = ["01", "02", "03", "04", "05", "06"]
        can_commands = []

        for motor_id in motor_ids:
            hex_cmd = f"{motor_id}33"
            response_hex = self.send_can_message(hex_cmd)
            if response_hex is None or not response_hex.startswith("Response: "):
                self.log_to_ui("ERROR", f"Failed to get CAN position for motor {motor_id}")
                return

            parts = response_hex.replace("Response: ", "").split(" ")
            if len(parts) < 7:
                self.log_to_ui("ERROR", f"Unexpected CAN feedback format: {response_hex}")
                return

            can_bytes = parts[3:6]
            raw_hex = ''.join(can_bytes)
            can_command = f"{motor_id}FE0040AA{raw_hex}"
            can_commands.append(can_command)

        can_path = Path(os.path.expanduser("~/aims_ws/src/aims_controller/aims_controller/grid_to_joint_can.json"))

        # Load and update CAN file
        if can_path.exists():
            with open(can_path, "r") as f:
                can_data = json.load(f)
        else:
            can_data = {}

        grid_key = f"{grid_x},{grid_y}"
        can_data[grid_key] = can_commands

        with open(can_path, "w") as f:
            json.dump(can_data, f, indent=2)

        self.log_to_ui("INFO", f"Saved grid ({grid_key}): CAN → {can_commands}")

    # Callback to handle G-Code file location updates
    def location_callback(self, msg):
        """Callback to update the G-Code file location."""
        new_path = os.path.expanduser(msg.data)  # Expand ~ if present in the path
        if os.path.isfile(new_path):
            self.file_path = new_path
            self.log_to_ui("INFO", f"G-Code file location updated to: {self.file_path}")
        else:
            self.log_to_ui("ERROR", f"Invalid G-Code file path received: {new_path}")

    # Callback to handle person detection 6/28 IMPORTANT 
    def person_detected_callback(self, msg):
        self.person_detected = msg.data
        if self.person_detected == 1 and self.stop_on_human_detected:
            #self.log_to_ui("INFO", "Person detected! Stopping all motors.")
            # Stop all motors
            self.control_callback(String(data="stop1"))
            self.control_callback(String(data="stop2"))
            self.control_callback(String(data="stop3"))
            self.control_callback(String(data="stop4"))
            self.control_callback(String(data="stop5_6"))

    # Callback to stop on person detection
    def stop_on_human_detected_callback(self, msg):
        self.stop_on_human_detected = msg.data

    # Update motor speed for a specific motor
    def update_motor_speed(self, command, speed):
        self.joystick_speeds[command] = speed
        #self.log_to_ui("INFO", f"Joystick speed for {command} updated to {speed}")

    # Callback to update joystick_enabled status
    def joystick_enabled_callback(self, msg):
        self.joystick_enabled = msg.data
        state = "enabled" if self.joystick_enabled else "disabled"
        #self.log_to_ui("INFO", f"Joystick control {state}.")

    # Callback to handle speed updates
    def speed_callback(self, msg):
        slider_value = msg.data  # Get the slider value (0–99)
        # Scale the slider value to the CAN speed range (0x01–0xmax_speed)
        self.current_speed = max(0x01, min(self.max_speed, int((slider_value / 99) * self.max_speed)))
        #self.log_to_ui("INFO", f"Speed updated to: {self.current_speed:#04x}")

    # Callback to handle updates to the serial port
    def serial_port_callback(self, msg):
        new_port = msg.data
        if new_port == "None":
            self.log_to_ui("WARNING", "No serial port available.")
            self.current_port = None
        else:
            self.log_to_ui("INFO", f"Serial port updated to: {new_port}")
            self.current_port = new_port

    # Callback to handle shutdown commands
    def shutdown_callback(self, msg):
        if msg.data:  # If the shutdown command is True
            self.log_to_ui("INFO", "Shutdown command received. Stopping Controller Node...")
            self.destroy_node()

    def cb_ik_commands(self, msg):
        # Split the aggregated message into individual commands
        commands = msg.data.split(";")
        self.log_to_ui("DEBUG", f"IK commands received: {commands}")

        # For now, set motor id 5 and 6 to have an angle of 000000
        commands[4] = "05FE0030AA000000"
        commands[5] = "06FE0030AA000000"
        # Store the commands in the can_message_map under the "ik" key
        self.can_message_map["ik"] = commands
        # Publish the "ik" command to the /aims/commands topic
        self.pub.publish(String(data="ik"))

    # Publish log messages to the /aims/logs topic
    def log_to_ui(self, level, message):
        log_msg = String()
        log_msg.data = f"[{level}] {message}"
        self.log_pub.publish(log_msg)

    # Callback to handle motor control and feedback commands
    def control_callback(self, msg):
        command = msg.data
        if (not self.stop_on_human_detected) or (self.stop_on_human_detected and self.person_detected == 0) or command.startswith("stop"):
            try:
                #self.log_to_ui("INFO", f"Received command: {command}")
                # If command starts with "known", get the command from the known x y command
                can_messages = None
                # Check if the command is a known command
                if command.startswith("known"):
                    x = command.split(" ")[1]
                    y = command.split(" ")[2]
                    # Print the command 
                    self.log_to_ui("INFO", f"Known command received for grid cell ({x}, {y})")
                    # Use x and y to get the command from the grid_to_joint_can.json file
                    can_path = Path(os.path.expanduser("~/aims_ws/src/aims_controller/aims_controller/grid_to_joint_can.json"))
                    if os.path.exists(can_path):
                        with open(can_path, "r") as f:
                            can_data = json.load(f)
                        grid_key = f"{x},{y}"
                        if grid_key in can_data:
                            can_messages = can_data[grid_key]
                            self.log_to_ui("INFO", f"CAN messages for grid cell ({x}, {y}): {can_messages}")
                        else:
                            self.log_to_ui("ERROR", f"Grid cell ({x}, {y}) not found in CAN data.")
                            return
                # Check if the command is a unknown command
                elif command.startswith("unknown"):
                    try:
                        x = float(command.split(" ")[1])
                        y = float(command.split(" ")[2])
                        # Print the command
                        self.log_to_ui("INFO", f"Unknown command received for grid cell ({x}, {y})")
                        interpolated_cmds = interpolate_can_commands(x, y, self.can_data)
                        can_messages = interpolated_cmds
                        self.log_to_ui("INFO", f"Interpolated CAN messages for ({x}, {y}): {can_messages}")

                    except Exception as e:
                        self.log_to_ui("ERROR", f"Interpolation failed: {e}")
                        return
                elif command == "pick":
                    self.drop_to_surface()
                    self.send_can_message("07FF")  # close gripper
                    time.sleep(1.0)
                    self.log_to_ui("INFO", "Gripper closed at surface.")
                    self.send_can_message("01FE0030AA000000")  # example: raise back up or go home

                    # Approximate the command using the grid cell can messages and interpolate the angles/hex values for the can messages
                    #if os.path.exists(can_path):
                else:
                    # Get the CAN message(s) for the command from the can_message_map
                    can_messages = self.get_can_message(command)
                if not can_messages:
                    self.log_to_ui("WARN", f"No CAN message mapped for command: {command}")
                    return
                
                if not isinstance(can_messages, list):
                    can_messages = [can_messages]

                # Attempt to send each CAN message
                for can_message in can_messages:
                    response = self.send_can_message(can_message)
                    
                    #if can_message.startswith("01FE") or can_message.startswith("02FE") or can_message.startswith("03FE") or can_message.startswith("04FE"):
                        #self.log_to_ui("INFO", f"message: {can_message}")
                    # Check if a response was received
                    if response is None:
                        self.log_to_ui("WARN", "No response received from CAN device.")
                        continue  # Skip processing this response

                    if response == "Gripper open" or response == "Gripper closed":
                        self.log_to_ui("INFO", f"Gripper command response: {response}")
                        continue

                    # Remove the "Response: " prefix if it exists
                    if response.startswith("Response: "):
                        response = response[len("Response: "):]
                        
                    # Split the feedback message into parts
                    parts = response.split(" ")
                    if len(parts) == 5:  # Motor speed feedback
                        motor_id, command_type, speed_high, speed_low, crc = parts  # Ignore the CRC field

                        # Check if the command type is "32" (speed feedback)
                        if command_type == "32":
                            self.publish_feedback(response)
                        else:
                            self.log_to_ui("INFO", f"Motor {motor_id} motion feedback received: {response}")
                    elif len(parts) == 4:  # Motor movement command response
                        motor_id, command_type, success, crc = parts
                        # Check if the success is "01" (success)
                        if success == "01":
                            self.log_to_ui("INFO", f"Motor movement feedback received: {response}")
                    elif len(parts) == 9:  # Motor movement command response
                        motor_id, command_type, angle6, angle5, angle4, angle3, angle2, angle1, crc = parts  # Ignore the CRC field
                        # Check if the success is "01" (success)
                        if command_type == "31":
                            self.publish_feedback(response)
                    else:
                        self.log_to_ui("WARN", f"Invalid feedback format: {response}")
            except Exception as e:
                self.log_to_ui("ERROR", f"Error in control_callback: {e}")

    # Send CAN message to the device
    def send_can_message(self, can_message):
        if not self.current_port:
            self.log_to_ui("ERROR", "No serial port available to send CAN message.")
            return None

        try:
            #self.log_to_ui("INFO", f"Attempting to connect to serial port: {self.current_port}")
            with serial.Serial(self.current_port, self.baudrate, timeout=1) as ser:
                crc = calculate_crc([int(can_message[i:i+2], 16) for i in range(0, len(can_message), 2)])
                can_message_with_crc = can_message + format(crc, '02X')
                #self.log_to_ui("INFO", f"Sending CAN message: {can_message_with_crc}")
                ser.write((can_message_with_crc + '\n').encode())

                response = ser.readline().decode().strip()
                if response:
                    return response
        except serial.SerialException as e:
            self.log_to_ui("ERROR", f"Failed to connect to serial port {self.current_port}: {e}")
        except Exception as e:
            self.log_to_ui("ERROR", f"Unexpected error in send_can_message: {e}")
        return None
    
    # Map G-Code commands to CAN messages
    def get_can_message(self, command):
        # Format: ID (2) + Mode (2) + Direction (1) + Speed (3) + Acceleration (2)
        base_speed = self.current_speed  # Base speed as an integer
        #speed_hex = f"{base_speed:03X}"  # Convert base speed to 3 hex digits

        # Define speed multipliers for specific motors
        motor_speed_multipliers = {
            "motor1_left": 0.25,  # Motor 1 spins slower, so reduce its speed
            "motor1_right": 0.25,
            "motor2_up": 2.0,  # Motor 2 spins slower, so double its speed
            "motor2_down": 2.0,
            "motor3_up": 2.0,  # Motor 3 spins slower, so increase its speed by 1.5x
            "motor3_down": 2.0,
            "motor4_clockwise": 2.0, 
            "motor4_counterclockwise": 2.0,
            "motor5_6_up": 2.0,
            "motor5_6_down": 2.0,
            "motor5_6_clockwise": 2.0,
            "motor5_6_counterclockwise": 2.0,
        }

        # Adjust speed for specific commands
        def adjust_speed(command_name):
            # Extract the motor ID from the command name (e.g., "motor1_left" -> "motor1")
            motor_id = command_name.split('_')[0]  # Get the first part of the command name
            multiplier = motor_speed_multipliers.get(command_name, 1.0)  # Default multiplier is 1.0

            if self.joystick_enabled:
                if motor_id not in self.joystick_speeds:
                    # This means it is motor 5 or 6, which is not variable speed with joystick
                    adjusted_speed = int(self.max_speed * multiplier)  # Scale joystick speed with multiplier
                else:
                    # Use joystick speed if joystick is enabled
                    joystick_speed = self.joystick_speeds.get(motor_id, 0.0)  # Default to 0.0
                    adjusted_speed = int(joystick_speed * self.max_speed * multiplier)  # Scale joystick speed with multiplier
            else:
                # Use normal speed calculation
                adjusted_speed = int(base_speed * multiplier)

            return f"{adjusted_speed:03X}"  # Convert back to 3 hex digits

        # CAN message map with adjusted speeds
        self.can_message_map = {
            "motor1_left": f"01F68{adjust_speed('motor1_left')}AA",  # Motor 1, forward, left (Counterclockwise)
            "motor1_right": f"01F60{adjust_speed('motor1_right')}AA",  # Motor 1, forward, right (Clockwise)
            "motor2_up": f"02F68{adjust_speed('motor2_up')}AA",  # Motor 2, forward, up (counterclockwise)
            "motor2_down": f"02F60{adjust_speed('motor2_down')}AA",  # Motor 2, forward, down (clockwise)
            "motor3_up": f"03F68{adjust_speed('motor3_up')}AA",  # Motor 3, forward, up (counterclockwise)
            "motor3_down": f"03F60{adjust_speed('motor3_down')}AA",  # Motor 3, forward, down (clockwise)
            "motor4_clockwise": f"04F68{adjust_speed('motor4_clockwise')}AA",  # Motor 4, forward, clockwise
            "motor4_counterclockwise": f"04F60{adjust_speed('motor4_counterclockwise')}AA",  # Motor 4, forward, counterclockwise
            "motor5_6_up": [f"05F60{adjust_speed('motor5_6_up')}AA", f"06F68{adjust_speed('motor5_6_up')}AA"],  # Motor 5 clockwise, Motor 6 counterclockwise
            "motor5_6_down": [f"05F68{adjust_speed('motor5_6_down')}AA", f"06F60{adjust_speed('motor5_6_down')}AA"],  # Motor 5 counterclockwise, Motor 6 clockwise
            "motor5_6_clockwise": [f"05F68{adjust_speed('motor5_6_clockwise')}AA", f"06F68{adjust_speed('motor5_6_clockwise')}AA"],  # Both clockwise
            "motor5_6_counterclockwise": [f"05F60{adjust_speed('motor5_6_counterclockwise')}AA", f"06F60{adjust_speed('motor5_6_counterclockwise')}AA"],  # Both counterclockwise
            "gripper_open": "07FF",  # Gripper open (fixed format)
            "gripper_close": "0700",  # Gripper close (fixed format)
            "stop1": "01F60000AA",  # Stop motor 1
            "stop2": "02F60000AA",  # Stop motor 2
            "stop3": "03F60000AA",  # Stop motor 3
            "stop4": "04F60000AA",  # Stop motor 4
            "stop5_6": ["05F60000AA", "06F60000AA"],  # Stop motors 5 and 6
            "stop_all": ["01FE0000AA000000", "02FE0000AA000000", "03FE0000AA000000", "04FE0000AA000000", "05FE0000AA000000", "06FE0000AA000000"],  # Stop all motors
            "speed_1": ["0132"],  # Request motor speed feedback
            "speed_2": ["0232"],  # Request motor speed feedback
            "speed_3": ["0332"],  # Request motor speed feedback
            "speed_4": ["0432"],  # Request motor speed feedback
            "speed_5": ["0532"],  # Request motor speed feedback
            "speed_6": ["0632"],  # Request motor speed feedback
            "angle_1": ["0131"],  # Request motor angle feedback
            "angle_2": ["0231"],  # Request motor angle feedback
            "angle_3": ["0331"],  # Request motor angle feedback
            "angle_4": ["0431"],  # Request motor angle feedback
            "angle_5": ["0531"],  # Request motor angle feedback
            "angle_6": ["0631"],  # Request motor angle feedback
            "home": ["01FE0020AA000000", "02FE00A0AA000000", "03FE00A0AA000000", "04FE00A0AA000000", "05FE00A0AA000000", "06FE00A0AA000000"],  # Home all motors
            "set_home_1": ["0192"],  # Set home for motor 1
            "set_home_2": ["0292"],  # Set home for motor 2
            "set_home_3": ["0392"],  # Set home for motor 3
            "set_home_4": ["0492"],  # Set home for motor 4
            "set_home_5": ["0592", "0692"],  # Set home for motor 5
            "test": ["01FE0020AA000000", "02FE0060AAFE8976", "03FE0060AA000FDE", "04FE0030AA000000", "05FE0060AA00150F", "06FE0060AAFFEAF1"],  # go to Scan on all motors
            "scan": ["01FE0040AA000000",   "02FE0040AAFFB9A6","03FE0040AA005DBF","04FE0040AA000000","05FE0040AA004F86","06FE0040AAFFA64B"],  # Scan command for all motors
            "offset_1": ["01FE0040AAFFED23"],  # Offset for motor 1
            "offset_2": ["02FD0040AA010B00"],  # Offset for motor 2
            "offset_3": ["03FE0040AA009D43"],  # Offset for motor 3 
            "offset_4": ["04FE0040AA009A33"],  # Offset for motor 4
            "offset_5": ["05FE0040AAFFA225"],  # Offset for motor 5
            "offset_6": ["06FE0040AA006664"],  # Offset for motor 6
            #"offset_pick":
            "led_red": "08FF",  # LED red
            "led_green": "0800",  # LED green
            "ultrasonic": "0900",  # Ultrasonic sensor
            "place": ["01FE0020AA000D81","02FE0040AAFE5D58","03FE0040AA009A1B","04FE0040AA000000","05FE0040AA0033EF","06FE0040AAFFF8FC"],  # Place command for all motors
            "place_up": ["01FE0020AA000D81","02FE0040AAFEA09B","03FE0040AA009A1B","04FE0040AA000000","05FE0040AA0033EF","06FE0040AAFFF8FC"],  # Place command for all motors        }
            "half_raise": ["01FE0040AA000000","02FE0040AAFF0763","03FE0040AA001E2D","04FE0040AAFFF7C9","05FE0040AA001E78","06FE0040AAFFD136"],  # Half raise command for all motors
        }

        return self.can_message_map.get(command)

    # Publish feedback to the /aims/motor_feedback topic
    def publish_feedback(self, feedback):
        feedback_msg = String()
        feedback_msg.data = feedback
        self.feedback_pub.publish(feedback_msg)
    
    # MD ADDED 5/4 for pcb position from cv node
    def pcb_position_callback(self, msg):
        self.log_to_ui(
        "INFO", f"Received PCB position: X={msg.x:.2f} m, Y={msg.y:.2f} m, Z={msg.z:.2f} m"
    )
        # Optionally store it for later use
        self.latest_pcb_position = msg

    # Override the destroy_node method to stop the thread
    def destroy_node(self):
        self.log_to_ui("INFO", "Shutting down ControllerNode...")
        self.running = False  # Stop the feedback thread
        if self.feedback_thread.is_alive():
            self.feedback_thread.join(timeout=5)  # Wait for up to 5 seconds
            if self.feedback_thread.is_alive():
                self.log_to_ui("ERROR", "Feedback thread did not terminate in time.")
        super().destroy_node()  # Call the parent class's destroy_node
#6/26 commented out the interpolation function to move up top and prevent circular import issues
# def hex_to_signed_int24(hex_str):
#     val = int(hex_str, 16)
#     if val & 0x800000:
#         val -= 0x1000000
#     return val

# def int_to_hex24(val):
#     if val < 0:
#         val += 0x1000000
#     return f"{val:06X}"

# def interpolate_can_commands(x, y, can_data):
#     x0, x1 = int(x), int(x) + 1
#     y0, y1 = int(y), int(y) + 1

#     def key(x, y): return f"{x},{y}"
#     for k in [key(x0, y0), key(x1, y0), key(x0, y1), key(x1, y1)]:
#         if k not in can_data:
#             raise ValueError(f"Missing key {k} in CAN data!")

#     def extract_values(k):
#         cmds = can_data[k]
#         return [hex_to_signed_int24(cmd[-6:]) for cmd in cmds]

#     Q11 = extract_values(key(x0, y0))
#     Q21 = extract_values(key(x1, y0))
#     Q12 = extract_values(key(x0, y1))
#     Q22 = extract_values(key(x1, y1))

#     dx = x - x0
#     dy = y - y0

#     result = []
#     for i in range(6):
#         interp_val = (
#             Q11[i] * (1 - dx) * (1 - dy) +
#             Q21[i] * dx * (1 - dy) +
#             Q12[i] * (1 - dx) * dy +
#             Q22[i] * dx * dy
#         )
#         int_val = round(interp_val)
#         hex_val = int_to_hex24(int_val)
#         motor_id = f"{i+1:02d}"
#         can_cmd = f"{motor_id}FE0040AA{hex_val}"
#         result.append(can_cmd)

#     return result

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()

    # Define a signal handler to ensure proper cleanup
    def signal_handler(sig, frame):
        node.destroy_node()
        rclpy.shutdown()
        exit(0)

    # Register the signal handler for SIGINT (Ctrl+C) and SIGTERM (termination)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Function to ask for grid input
    # and log angles for the specified grid cell
    # MD ADDED 5/9 for pcb position from cv node
    def ask_for_grid_input():
        while rclpy.ok():
            grid_input = input("Enter grid cell (x,y) to log angles, or 'q' to quit: ")
            if grid_input.lower() == "q":
                break
            try:
                x, y = map(int, grid_input.split(","))
                node.log_grid_position(x, y)
            except Exception as e:
                print(f"Invalid input: {e}")

    # Start the logging thread
    import threading
    threading.Thread(target=ask_for_grid_input, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()