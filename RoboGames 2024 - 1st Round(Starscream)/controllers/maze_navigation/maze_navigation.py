from controller import Robot

# Constants
TIME_STEP = 32
WALL_THRESHOLD = 180
FRONT_THRESHOLD = 100  # Threshold for distance sensors to detect walls
BASE_SPEED = 6.0

TARGET_COLORS = {
    "RED": (255, 0, 0),
    "YELLOW": (255, 255, 0),
    "PINK": (255, 0, 255),
    "BROWN": (165, 105, 30),
    "GREEN": (0, 255, 0)}

# Initialize robot
robot = Robot()

# Initialize motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Initialize sensors
distance_sensors = []
sensor_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for name in sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    distance_sensors.append(sensor)

camera = robot.getDevice('camera')
camera.enable(TIME_STEP)

# Function to calculate Euclidean distance between two colors
def color_distance(color1, color2):
    return sum((color1[i] - color2[i])**2 for i in range(3))**0.5

# Function to detect the dominant color in the camera view
def detect_color():
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()

    for x in range(width):
        for y in range(height):
            # Get the RGB values of the pixel
            r = camera.imageGetRed(image, width, x, y)
            g = camera.imageGetGreen(image, width, x, y)
            b = camera.imageGetBlue(image, width, x, y)
            pixel_color = (r, g, b)
            

            # Check which target color the pixel is closest to
            for color_name, target_rgb in TARGET_COLORS.items():
                if color_distance(pixel_color, target_rgb) < 40:  # Adjust threshold if necessary
                    return color_name
    return None

# Function to check if the wall is on the right
def is_wall_on_right():

    return distance_sensors[1].getValue() > WALL_THRESHOLD
# Function to check if the front is clear
def is_front_clear():
    return distance_sensors[0].getValue() < FRONT_THRESHOLD

# Function to stop the robot
def stop_robot():
    for speed in range(int(BASE_SPEED), -1, -1):
        left_motor.setVelocity(speed / 10.0)
        right_motor.setVelocity(speed / 10.0)
        robot.step(TIME_STEP)

# Main function
def main():
    global BASE_SPEED
    detected_red = False  # Flag for detecting RED
    last_detected = None
    current_color = None
    on_path = False  # Variable to track the last detected color
    
    while robot.step(TIME_STEP) != -1:
        # Check the current color
        if is_wall_on_right():
            current_color = detect_color()
        
        if current_color and detected_red and current_color != last_detected:
            print(f"{current_color} detected!")
            last_detected = current_color

        if not detected_red and current_color == "RED":
            print("RED detected! Continuing to GREEN...")
            detected_red = True
            last_detected = current_color  # Mark RED as detected
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            robot.step(2000)  # Pause for 2 seconds
            BASE_SPEED = 6.0


        elif detected_red and current_color == "GREEN":
            print("Stopping...!")
            stop_robot()
            break

        # Wall-following logic
        if not on_path:
            if is_front_clear() and not is_wall_on_right():
                left_motor.setVelocity(BASE_SPEED)
                right_motor.setVelocity(BASE_SPEED)
            else:
                on_path = True
                
        else:
            if not is_front_clear():  # If there's a wall in front, turn left
                left_motor.setVelocity(-BASE_SPEED)
                right_motor.setVelocity(BASE_SPEED)
            elif not is_wall_on_right():  # If there's no wall on the right, turn right
                left_motor.setVelocity(BASE_SPEED)
                right_motor.setVelocity(BASE_SPEED / 16)
            else:  # Otherwise, move forward
                left_motor.setVelocity(BASE_SPEED)
                right_motor.setVelocity(BASE_SPEED)

# Run the main function
main()
