#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
import inputs  # type: ignore[import]
import threading
import signal
from time import sleep
from time import time

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')

        # Publisher for log messages
        self.log_pub = self.create_publisher(String, '/aims/logs', 10)
        # Publisher for joystick events
        self.joystick_enabled_pub = self.create_publisher(Bool, '/aims/joystick_enabled', 10)
        # Publishers for joystick speeds
        self.joystick_speed_motor1_pub = self.create_publisher(Float32, '/aims/joystick_speed_motor1', 10)
        self.joystick_speed_motor2_pub = self.create_publisher(Float32, '/aims/joystick_speed_motor2', 10)
        self.joystick_speed_motor3_pub = self.create_publisher(Float32, '/aims/joystick_speed_motor3', 10)
        self.joystick_speed_motor4_pub = self.create_publisher(Float32, '/aims/joystick_speed_motor4', 10)
        # Publisher for /aims/control (string)
        self.control_pub = self.create_publisher(String, '/aims/control', 10)

        # Subscriber to listen for shutdown commands
        self.sub_shutdown = self.create_subscription(
            Bool,
            '/aims/shutdown',
            self.shutdown_callback,
            10
        )

        # Subscriber to enable/disable joystick input
        self.sub_joystick_enabled = self.create_subscription(
            Bool,
            '/aims/joystick_enabled',
            self.joystick_enabled_callback,
            10
        )

        # Flag to track if joystick input is enabled
        self.joystick_enabled = True
        self.hasCheckedJoystick = False

        # Acknowledge the node has started
        self.log_to_ui("INFO", "Joystick Node has started successfully!")

        # Publish the initial state of the joystick as disabled
        initial_joystick_state = Bool()
        initial_joystick_state.data = False
        self.joystick_enabled_pub.publish(initial_joystick_state)

        # Track current motor speeds
        self.current_motor_speeds = {
            "motor1": 0.0,
            "motor2": 0.0,
            "motor3": 0.0,
            "motor4": 0.0,
        }

        # Define a threshold for significant speed changes
        self.speed_change_threshold = 0.10
        # Track the states of ABS_HAT0X and ABS_HAT0Y
        self.hat0x_state = 0
        self.hat0y_state = 0
        self.last_dpad_time = 0.0
        self.dpad_debounce_delay = 0.1  # 100ms debounce delay

        # Start the joystick monitoring thread
        self.running = True
        self.joystick_thread = threading.Thread(target=self.monitor_joystick, daemon=True)
        self.joystick_thread.start()

    # Callback to enable/disable joystick input
    def joystick_enabled_callback(self, msg):
        self.joystick_enabled = msg.data
        state = "enabled" if self.joystick_enabled else "disabled"

    # Publish log messages to the /aims/logs topic
    def log_to_ui(self, level, message):
        log_msg = String()
        log_msg.data = f"[{level}] {message}"
        self.log_pub.publish(log_msg)

    # Monitor joystick inputs in a separate thread
    def monitor_joystick(self):
        try:
            self.log_to_ui("INFO", "Monitoring joystick inputs...")
            while self.running:
                try:
                    events = inputs.get_gamepad()
                    for event in events:
                        if self.joystick_enabled:
                            self.process_joystick_event(event)
                    # Continuously read joystick events
                    if self.hasCheckedJoystick:
                        self.hasCheckedJoystick = False
                        self.log_to_ui("INFO", "Joystick detected. Monitoring inputs...")
                except Exception as e:
                    # Log any errors from inputs.get_gamepad()
                    if not self.hasCheckedJoystick:
                        self.log_to_ui("WARN", "No joystick detected. Please check the connection.")
                        self.hasCheckedJoystick = True
        except Exception as e:
            self.log_to_ui("ERROR", f"Error in joystick monitoring: {e}")
            self.running = False
        finally:
            self.log_to_ui("INFO", "Joystick monitoring thread exiting.")

    # Process joystick events and publish motor speeds
    def process_joystick_event(self, event):
        # Map joystick events to motor speeds
        if event.code == "ABS_X":
            self.update_motor_speed("motor4", event.state)
            self.publish_motor_command("motor4", event.state, event)
        elif event.code == "ABS_Y":
            self.update_motor_speed("motor3", event.state)
            self.publish_motor_command("motor3", event.state, event)
        elif event.code == "ABS_RX":
            self.update_motor_speed("motor1", event.state)
            self.publish_motor_command("motor1", event.state, event)
        elif event.code == "ABS_RY":
            self.update_motor_speed("motor2", event.state)
            self.publish_motor_command("motor2", event.state, event)
        elif event.code == "ABS_HAT0X":
            # Update the state of ABS_HAT0X
            self.hat0x_state = event.state
            self.handle_hat_event()
        elif event.code == "ABS_HAT0Y":
            # Update the state of ABS_HAT0Y
            self.hat0y_state = event.state
            self.handle_hat_event()
        elif event.code == "BTN_SOUTH" and event.state == 1:
            self.control_pub.publish(String(data="gripper_open"))
        elif event.code == "BTN_EAST" and event.state == 1:
            self.control_pub.publish(String(data="gripper_close"))

    def handle_hat_event(self):
        current_time = time()

        # Always handle release to stop motors
        if self.hat0x_state == 0 and self.hat0y_state == 0:
            self.control_pub.publish(String(data="stop5_6"))
            self.last_dpad_time = current_time  # update last time even for release
            return

        # Debounce for active directional inputs only
        if current_time - self.last_dpad_time < self.dpad_debounce_delay:
            return

        self.last_dpad_time = current_time  # Update time since we're processing input

        # Conflict check
        if abs(self.hat0x_state) == 1 and abs(self.hat0y_state) == 1:
            self.log_to_ui("WARNING", "Conflict: Both ABS_HAT0X and ABS_HAT0Y are active. Ignoring input.")
            self.control_pub.publish(String(data="stop5_6"))
            return

        # Directional logic
        if self.hat0y_state == -1:
            self.control_pub.publish(String(data="motor5_6_up"))
        elif self.hat0y_state == 1:
            self.control_pub.publish(String(data="motor5_6_down"))
        elif self.hat0x_state == 1:
            self.control_pub.publish(String(data="motor5_6_clockwise"))
        elif self.hat0x_state == -1:
            self.control_pub.publish(String(data="motor5_6_counterclockwise"))



    # Update motor speed and publish only if it changes significantly
    def update_motor_speed(self, motor_id, raw_value):
        # Normalize the joystick value
        new_speed = self.normalize_joystick_value(raw_value)

        # Check if the speed change exceeds the threshold
        if abs(new_speed - self.current_motor_speeds[motor_id]) < self.speed_change_threshold:
            # Update the current speed
            self.current_motor_speeds[motor_id] = 0.0

            # Publish the new speed
            publisher = getattr(self, f"joystick_speed_{motor_id}_pub")
            publisher.publish(Float32(data=0.0))

            # Log the change
            #self.log_to_ui("INFO", f"Motor {motor_id} speed updated to {new_speed}")
        elif abs(new_speed - self.current_motor_speeds[motor_id]) > self.speed_change_threshold or new_speed == 0.0:
            # Update the current speed
            self.current_motor_speeds[motor_id] = new_speed

            # Publish the new speed
            publisher = getattr(self, f"joystick_speed_{motor_id}_pub")
            publisher.publish(Float32(data=new_speed))

            # Log the change
            #self.log_to_ui("INFO", f"Motor {motor_id} speed updated to {new_speed}")

    # Publish motor commands to the /aims/control topic
    def publish_motor_command(self, motor_id, raw_value, event):
        # Determine the direction based on the raw joystick value
        # If the joystick value is 0 then the speed will be set to 0 anyway, stopping the motor.
        if raw_value > 0: # GREATER THAN 0
            if motor_id == "motor4":
                command = f"{motor_id}_clockwise"
            else:
                command = f"{motor_id}_right" if motor_id == "motor1" else f"{motor_id}_up"
        elif raw_value <= 0: # LESS THAN OR EQUAL TO 0
            if motor_id == "motor4":
                command = f"{motor_id}_counterclockwise"
            else:
                command = f"{motor_id}_left" if motor_id == "motor1" else f"{motor_id}_down"
        
        # Publish the command to the /aims/control topic
        control_msg = String()
        control_msg.data = command
        self.control_pub.publish(control_msg)

        # Log the published command
        #self.log_to_ui("INFO", f"Published command: {command}")

    # Normalize joystick values to a range of 0.0 to 1.0
    def normalize_joystick_value(self, value):
        # Normalize using the absolute value
        normalized_value = abs(value) / 32767.0  # Divide by the maximum positive value
        normalized_value = round(normalized_value, 2)  # Round to two decimal places
        return normalized_value

    # Callback to handle shutdown commands
    def shutdown_callback(self, msg):
        if msg.data:  # If the shutdown command is True
            self.log_to_ui("INFO", "Shutdown command received. Stopping Joystick Node...")
            self.destroy_node()

    # Override the destroy_node method to handle cleanup
    def destroy_node(self):
        self.log_to_ui("INFO", "Shutting down JoystickNode...")
        self.running = False  # Stop the joystick thread
        if self.joystick_thread.is_alive():
            self.joystick_thread.join(timeout=5)  # Wait for up to 5 seconds
            #if self.joystick_thread.is_alive():
                #self.log_to_ui("ERROR", "Joystick thread did not terminate in time.")
        super().destroy_node()  # Call the parent class's destroy_node

def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()

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
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__ == "__main__":
    main()