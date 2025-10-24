# fmt: off
import serial
import serial.tools.list_ports
import time
import math
import coordinateconverter
# fmt: on

PORT = "COM3"
BAUDRATE = 9600
INITIALANGLES = [90, 205, 45, 100] 
INITIAL_GRABBER_ANGLE =  50 # Grabber Open
GRABANGLE = 100 # Grabber Closed
ANGLECORRECTIONS = [-17, 7, 7] # Base, Shoulder, Elbow corrections
SERVOPINS = [2, 3, 4, 5] # Just for reference, Arduino code uses these pins

# Global variable for the serial connection
arduino_serial = None
# Store the last sent angles [Base, Shoulder, Elbow, Grabber]
current_angles = INITIALANGLES


def list_serial_ports():
    """ Lists serial port names """
    ports = serial.tools.list_ports.comports()
    available_ports = []
    for port, desc, hwid in sorted(ports):
        available_ports.append(port)
    return available_ports

def connect(port, baudrate=BAUDRATE):
    """ Establishes serial connection to the Arduino """
    global arduino_serial, current_angles
    try:
        # Close previous connection if exists
        if arduino_serial and arduino_serial.is_open:
            arduino_serial.close()

        arduino_serial = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        time.sleep(2) # Wait for connection to establish and Arduino to reset

        print(f"Connected to Arduino on {port}")
        # Reset to initial position
        set_arm_angles(INITIALANGLES[0], INITIALANGLES[1], INITIALANGLES[2], INITIALANGLES[3])
        current_angles = INITIALANGLES # Update current angles
        return True
    except serial.SerialException as e:
        print(f"Error connecting to {port}: {e}")
        arduino_serial = None
        return False
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        arduino_serial = None
        return False

def disconnect():
    """ Closes the serial connection """
    global arduino_serial
    if arduino_serial and arduino_serial.is_open:
        set_arm_angles(INITIALANGLES[0], INITIALANGLES[1], INITIALANGLES[2], INITIALANGLES[3])
        time.sleep(0.5)
        arduino_serial.close()
        arduino_serial = None
        print("Disconnected from Arduino.")

def map_range(x, in_min, in_max, out_min, out_max):
    """ Maps a value from one range to another """
    # Ensure x is within the input range
    x = max(in_min, min(x, in_max))
    # Avoid division by zero
    if in_max == in_min:
        return out_min
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def set_arm_angles(servo1_base, servo2_shoulder, servo3_elbow, servo4_grabber):
    """ Sends the target angles for all four servos to the Arduino """
    global arduino_serial, current_angles
    if not arduino_serial or not arduino_serial.is_open:
        print("Error: Arduino not connected.")
        return

    # Apply corrections and limits from original sendData/rotateServo logic
    # Base Servo (Servo 1)
    s1 = servo1_base
    if (s1 > 90): s1 += 3
    if (s1 < 90): s1 -= 3
    s1 = map_range((s1 + ANGLECORRECTIONS[0]), 0, 180, 0, 153) # Original mapping range
    s1 = max(0, min(180, s1)) # Ensure 0-180

    # Shoulder Servo (Servo 2)
    s2 = servo2_shoulder
    s2 = max(90, min(250, s2)) # Original limits
    s2 = map_range((s2 - 90 + ANGLECORRECTIONS[1]), 0, 180, 0, 153) # Original mapping range
    s2 = max(0, min(180, s2)) # Ensure 0-180

    # Elbow Servo (Servo 3)
    s3 = servo3_elbow
    s3 = max(0, s3) # Original limit
    s3 = map_range((s3-50 + ANGLECORRECTIONS[2]), 0, 180, 0, 160) # Original mapping range
    s3 = max(0, min(180, s3)) # Ensure 0-180

    # Grabber Servo (Servo 4)
    s4 = servo4_grabber
    s4 = max(0, min(GRABANGLE + 10, s4)) # Apply limits (allow slightly past GRABANGLE)
    s4 = max(0, min(180, s4)) # Ensure 0-180

    # Format: "B{base_angle},S{shoulder_angle},E{elbow_angle},G{grabber_angle}\n"
    command = f"B{int(s1)},S{int(s2)},E{int(s3)},G{int(s4)}\n"

    try:
        arduino_serial.write(command.encode('utf-8'))
        # Store the angles *before* mapping/correction for internal logic consistency
        current_angles = [servo1_base, servo2_shoulder, servo3_elbow, servo4_grabber]
        # Optional: Wait for acknowledgment from Arduino?
        # response = arduino_serial.readline().decode('utf-8').strip()
        # print("Arduino response:", response)
        time.sleep(0.02) # Small delay to allow servo to move / prevent overwhelming serial buffer
    except serial.SerialException as e:
        print(f"Error writing to serial port: {e}")
        disconnect() # Disconnect if writing fails
    except Exception as e:
        print(f"Error sending command: {e}")


def guiControl(servo1Angle, servo2Angle, servo3Angle, servo4Angle):
    """ Used by GUI sliders for direct control """
    set_arm_angles(servo1Angle, servo2Angle, servo3Angle, servo4Angle)

def drawFromCoordinates(coordinate, previousCoordinate, pen_down=True):
    """ Moves the arm for drawing, assumes pen is up or down """
    servoAngles = coordinateconverter.convertCoordstoAngles(coordinate)
    grabberAngle = 170 if pen_down else 110 # Example: 180=down, 120=up
    targetAngles = servoAngles + [grabberAngle]

    # Use smooth movement if distance is significant
    if (math.dist(coordinate, previousCoordinate) > 0.5):
        previousAngles = coordinateconverter.convertCoordstoAngles(previousCoordinate) + [current_angles[3]] # Use current grabber angle
        smoothAngleExecution(previousAngles, targetAngles) # Make sure smoothAngleExecution uses set_arm_angles
    else:
        set_arm_angles(targetAngles[0], targetAngles[1], targetAngles[2], targetAngles[3])
        time.sleep(0.01)

def stabilizeAngles(servoAngles, previousAngles):
    """ Helper for smooth movement - Moves base first, then others """
    # This function needs updating to use set_arm_angles correctly with 4 angles
    # Example: Move to intermediate height first
    set_arm_angles(previousAngles[0]  + 15 , servoAngles[1], previousAngles[2], previousAngles[3])
    time.sleep(0.05)

    incrementer = 1 if previousAngles[0] <= servoAngles[0] else -1
    # Move base incrementally
    for base_angle in range(int(previousAngles[0]), int(servoAngles[0]), incrementer):
        set_arm_angles(base_angle, servoAngles[1] + 15, servoAngles[2], servoAngles[3])
        time.sleep(0.05) # Adjust timing as needed
    # Move to final position
    set_arm_angles(servoAngles[0], servoAngles[1], servoAngles[2], servoAngles[3])


def smoothAngleExecution(previousAngles, targetAngles):
    """ Smoothly transitions between two sets of angles using sine interpolation """
    # previousAngles and targetAngles should be [base, shoulder, elbow, grabber]
    distances = [t - p for p, t in zip(previousAngles, targetAngles)]

    temp_angles = list(previousAngles) # Make a mutable copy

    # Interpolate using sin from 0 to 90 degrees (0 to 1)
    for i in range(0, 91, 5): # Increase step for faster movement (e.g., 5)
        rad = math.radians(i)
        sine_factor = math.sin(rad)
        temp_angles[0] = int(previousAngles[0] + distances[0] * sine_factor)
        temp_angles[1] = int(previousAngles[1] + distances[1] * sine_factor)
        temp_angles[2] = int(previousAngles[2] + distances[2] * sine_factor)
        temp_angles[3] = int(previousAngles[3] + distances[3] * sine_factor) # Interpolate grabber too if needed

        set_arm_angles(temp_angles[0], temp_angles[1], temp_angles[2], temp_angles[3])
        # time.sleep(0.01) # Delay is now inside set_arm_angles

    # Ensure final position is reached
    set_arm_angles(targetAngles[0], targetAngles[1], targetAngles[2], targetAngles[3])
    return targetAngles # Return the final target state


def down(previousAngles, targetAngles):
    """ Moves down: Interpolate shoulder/elbow more """
    # This requires careful planning based on desired movement path.
    # Simplified: Just use smoothAngleExecution for now.
    smoothAngleExecution(previousAngles, targetAngles)

def up(previousAngles, targetAngles):
    """ Moves up: Interpolate shoulder/elbow more """
    # Simplified: Just use smoothAngleExecution for now.
    smoothAngleExecution(previousAngles, targetAngles)

def angleTest(servo_index): # Pass servo index 0-3
    """ Allows manual testing of a single servo via console input """
    global current_angles
    print(f"Testing Servo Index {servo_index}. Current Angles: {current_angles}")
    while True:
        try:
            angle_str = input(f"Enter angle for servo {servo_index} (0-180, 'q' to quit): ")
            if angle_str.lower() == 'q':
                break
            angle = int(angle_str)
            if 0 <= angle <= 180:
                 # Create a copy of current angles and modify only the target servo
                temp_angles = list(current_angles)
                temp_angles[servo_index] = angle
                set_arm_angles(temp_angles[0], temp_angles[1], temp_angles[2], temp_angles[3])
            else:
                print("Angle out of range (0-180)")
        except ValueError:
            print("Invalid input. Please enter a number or 'q'.")
        except KeyboardInterrupt:
            break
    print("Exiting angle test.")

def grabObject():
    """ Closes the grabber smoothly with proper stopping """
    global current_angles
    # Keep other servos at current position
    base, shoulder, elbow, current_grabber = current_angles
    
    # Only move if we're not already at grab position
    if current_grabber >= GRABANGLE:
        return  # Already closed
    
    # Move incrementally to GRABANGLE
    for i in range(int(current_grabber), GRABANGLE + 1, 2):  # Step by 2 for speed
        if i >= GRABANGLE:  # Stop exactly at GRABANGLE
            set_arm_angles(base, shoulder, elbow, GRABANGLE)
            break
        set_arm_angles(base, shoulder, elbow, i)
        time.sleep(0.02)  # Short delay for grabber movement
    
    # Ensure we're exactly at GRABANGLE and update current_angles
    set_arm_angles(base, shoulder, elbow, GRABANGLE)
    print(f"Gripper closed to angle: {GRABANGLE}")

def releaseObject():
    """ Opens the grabber smoothly with proper stopping """
    global current_angles
    # Keep other servos at current position
    base, shoulder, elbow, current_grabber = current_angles
    
    # Only move if we're not already at open position
    if current_grabber <= INITIAL_GRABBER_ANGLE:
        return  # Already open
    
    # Move incrementally to INITIAL_GRABBER_ANGLE
    for i in range(int(current_grabber), INITIAL_GRABBER_ANGLE - 1, -2):  # Step by -2
        if i <= INITIAL_GRABBER_ANGLE:  # Stop exactly at INITIAL_GRABBER_ANGLE
            set_arm_angles(base, shoulder, elbow, INITIAL_GRABBER_ANGLE)
            break
        set_arm_angles(base, shoulder, elbow, i)
        time.sleep(0.02)
    
    # Ensure we're exactly at INITIAL_GRABBER_ANGLE and update current_angles
    set_arm_angles(base, shoulder, elbow, INITIAL_GRABBER_ANGLE)
    print(f"Gripper opened to angle: {INITIAL_GRABBER_ANGLE}")

def pickObject(coordinates):
    """ Sequence to pick up an object at given coordinates """
    global current_angles
    print(f"Starting pick sequence at coordinates: {coordinates}")

    # 1. Ensure gripper is open and arm is at initial height
    print("Opening gripper and moving to initial position...")
    set_arm_angles(current_angles[0], current_angles[1], current_angles[2], INITIAL_GRABBER_ANGLE)
    time.sleep(0.3)  # Wait for gripper to open
    
    up_position = INITIALANGLES + [INITIAL_GRABBER_ANGLE]  # Starting position
    
    # 2. Calculate target angles to grab the object
    targetAnglesBase = coordinateconverter.convertCoordstoAngles(coordinates)
    target_down_position = targetAnglesBase + [INITIAL_GRABBER_ANGLE]  # Target position with open gripper
    
    # 3. Move arm above the object
    target_above_position = [target_down_position[0], target_down_position[1] + 40, target_down_position[2], INITIAL_GRABBER_ANGLE]
    print("Moving above object...")
    smoothAngleExecution(up_position, target_above_position)
    time.sleep(0.3)
    
    # 4. Move down to the object
    print("Moving down to object...")
    down(target_above_position, target_down_position)
    time.sleep(0.5)  # Wait for arm to be in position
    
    # 5. Close gripper - this is where the fix is important
    print("Closing gripper to grab object...")
    grabObject()  # Use the fixed grabObject function
    time.sleep(0.5)  # Wait for object to be grabbed
    
    # 6. Lift the object
    print("Lifting object...")
    current_angle = [target_above_position[0], target_above_position[1] + 40, target_above_position[2], GRABANGLE]
    up(current_angles, current_angle)
    time.sleep(0.5)
    up(current_angle, up_position)
    print("Pick sequence completed!")

def placeObject(coordinates):
    """ Sequence to place an object at given coordinates """
    global current_angles
    print(f"Starting place sequence at coordinates: {coordinates}")

    # 1. Start from current position (should have object in gripper)
    up_position = INITIALANGLES + [GRABANGLE]  # Starting position with closed gripper
    
    # 2. Calculate target angles to place the object
    targetAnglesBase = coordinateconverter.convertCoordstoAngles(coordinates)
    target_down_position = targetAnglesBase + [GRABANGLE]  # Target position with closed gripper
    
    # 3. Move arm above the target location
    target_above_position = [target_down_position[0], target_down_position[1] + 40, target_down_position[2], GRABANGLE]
    print("Moving above target location...")
    smoothAngleExecution(up_position, target_above_position)
    time.sleep(0.3)
    
    ## 4. Move down to place the object
    #print("Moving down to place object...")
    #down(target_above_position, target_down_position)
    #time.sleep(0.5)  # Wait for arm to be in position
    #
    ## 5. Open gripper to release object - this is where the fix is important
    #print("Opening gripper to release object...")
    releaseObject()  # Use the fixed releaseObject function
    time.sleep(0.5)  # Wait for object to be released
    #
    ## 6. Lift arm back up
    print("Lifting arm after placing object...")
    final_up_position = INITIALANGLES + [INITIAL_GRABBER_ANGLE]
    current_angle = [target_above_position[0], target_above_position[1] + 40, target_above_position[2], INITIAL_GRABBER_ANGLE]
    up(current_angles, current_angle)
    time.sleep(0.5)
    up(current_angle, final_up_position)
    print("Place sequence completed!")

if (__name__ == "__main__"):
    # Example usage: Connect and test servo 0 (Base)
    available_ports = list_serial_ports()
    print("Available ports:", available_ports)
    if available_ports:
        port_to_use = input(f"Enter port to use (default {PORT}): ") or PORT
        if connect(port_to_use):
            try:
                # Test pick and place cycle
                print("Testing Pick and Place...")
                pickObject([-8, 13]) # Pick from left
                time.sleep(1)
                placeObject([8, 13]) # Place on right
                time.sleep(1)
                pickObject([8, 13]) # Pick from right
                time.sleep(1)
                placeObject([-8, 13]) # Place on left
                time.sleep(1)

                # Test manual angle setting
                # angleTest(0) # Test Base servo

            except KeyboardInterrupt:
                print("\nExiting test.")
            finally:
                # Go back to initial position before disconnecting
                print("Returning to initial position...")
                smoothAngleExecution(current_angles, INITIALANGLES + [INITIAL_GRABBER_ANGLE])
                time.sleep(1)
                disconnect()
        else:
            print("Connection failed.")
    else:
        print("No serial ports found.")