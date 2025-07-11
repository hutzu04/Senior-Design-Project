import os
import re
import serial
import time
import serial.tools.list_ports
import rclpy
import threading

# Parameters Section
# -------------------

# Gearbox ratios for each motor
gear_ratios = [1, 1, 1, 1, 1, 1]  # Replace with your actual gearbox ratios

# Direction inversion for each motor (True/False)
invert_direction = [True, True, False, False, False, False]  # Set True for motors where direction should be inverted

# Initialize zero positions and last positions
initial_positions = [0] * 6
last_positions = [0] * 6

# -------------------

def calculate_crc(data):
    crc = sum(data) & 0xFF
    return crc

def convert_to_can_message(axis_id, speed, position, gear_ratio, invert_direction=False):
    can_id = format(axis_id, '02X')
    speed_hex = format(speed, '04X')

    # Calculate relative position based on the initial position
    rel_position = int((position * gear_ratio - initial_positions[axis_id - 1]) * 100)

    # Handle signed 24-bit integer using two's complement representation
    rel_position_hex = format(rel_position & 0xFFFFFF, '06X')

    # Update last_position for the axis
    last_positions[axis_id - 1] = position * gear_ratio

    return can_id + 'F4' + speed_hex + '02' + rel_position_hex

def find_serial_port():
    """
    Finds the port for the serial device.

    Returns:
        The device name of the serial port, or None if not found.
    """
    rclpy.loginfo("Searching for available serial ports...")
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if "USB" in port.description or "ACM" in port.description:
            rclpy.loginfo("Found serial port: {}".format(port.device))
            return port.device
    rclpy.logwarn("No suitable serial port found.")
    return None

def send_can_messages(serial_port, baudrate, file_path):
    """
    Sends CAN messages converted from G-code commands to a device over a serial connection.

    Args:
        serial_port: The serial port to which the device is connected (e.g., '/dev/ttyUSB0').
        baudrate: The baud rate for the serial connection (e.g., 115200).
        file_path: The path to the file containing G-code commands.
    """
    try:
        rclpy.loginfo("Attempting to connect to serial port: {}".format(serial_port))
        with serial.Serial(serial_port, baudrate, timeout=1) as ser:
            rclpy.loginfo("Successfully connected to serial port.")
            with open(file_path, 'r') as input_file:
                speed = 0

                for line in input_file:
                    speed_match = re.search(r'F(\d+)', line)
                    if speed_match:
                        try:
                            speed = int(speed_match.group(1))
                        except ValueError:
                            continue

                    if line.startswith("G90"):
                        values = [float(value) if '.' in value else int(value) for value in re.findall(r'[-+]?\d*\.\d+|[-+]?\d+', line)]

                        if len(values) >= 7:
                            for axis_id, position in enumerate(values[1:7], start=1):
                                gear_ratio = gear_ratios[axis_id - 1]
                                invert_dir = invert_direction[axis_id - 1]
                                can_message = convert_to_can_message(axis_id, speed, position, gear_ratio, invert_dir)
                                crc = calculate_crc([int(can_message[i:i+2], 16) for i in range(0, len(can_message), 2)])
                                can_message_with_crc = can_message + format(crc, '02X')
                                

                                # --DEBUG--
                                # Log the CAN message before sending it
                                #rclpy.loginfo("Prepared CAN message: {}".format(can_message_with_crc))
                                
                                rclpy.loginfo("Sending CAN message: {}".format(can_message_with_crc))
                                ser.write((can_message_with_crc + '\n').encode())  # Send CAN message

                                response = ser.readline().decode().strip()
                                if response:
                                    rclpy.loginfo("Received: {}".format(response))
                                else:
                                    rclpy.logwarn("No response received for CAN message: {}".format(can_message_with_crc))

                                time.sleep(0.1)  # Add delay if necessary

    except serial.SerialException as e:
        rclpy.logerr("Serial error: {}".format(e))
    except FileNotFoundError:
        rclpy.logerr("G-code file not found.")
    except Exception as e:
        rclpy.logerr("Unexpected error: {}".format(e))

def main():
    script_directory = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(script_directory, "/home/aims/Documents/RoboDK/Programs/Pick_And_Place_AIMS.gcode")
    baudrate = 115200  # Change to your device's baud rate

    if not os.path.exists(file_path):
        rclpy.logerr("File not found in the script directory.")
        return

    serial_port = "/dev/ttyACM0"  # Set to CAN port
    if serial_port is None:
        rclpy.logerr("No suitable serial port found. Please check your connection.")
        return

    rclpy.loginfo("Using serial port: {}".format(serial_port))
    send_can_messages(serial_port, baudrate, file_path)

def start_send_gcode_thread():
    thread = threading.Thread(target=main)
    thread.start()

if __name__ == "__main__":
    rclpy.init_node('send_gcode_node')
    main()