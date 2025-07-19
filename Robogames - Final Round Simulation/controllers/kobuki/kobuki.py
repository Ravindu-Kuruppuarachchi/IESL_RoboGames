import cv2
import numpy as np
from controller import Robot

class VideoController:
    def __init__(self, camera, left_motor, right_motor):
        self.camera = camera
        self.left_motor = left_motor
        self.right_motor = right_motor
        
        # List of HSV color ranges: Red, Green, Blue, Yellow
        self.colors = [
            #((100, 150, 0), (140, 255, 255)),  # Blue     
            #((0, 100, 100), (10, 255, 255)),   # Red             
            ((35, 100, 100), (85, 255, 255)),  # Green
            ((20, 100, 100), (30, 255, 255)),  # Yellow
        ]
        
        # State flags
        self.cube_acquired = False
        self.avoiding_obstacle = False
        self.avoiding_direction = None
        self.drop_clearing = False
        self.spot_clearing = False

    def get_frames(self):
        video_frame = self.camera.getImage()
        if video_frame is None:
            return None

        # Webots camera typically gives 4 channels: RGBA
        # Suppose it's 480x640, 4 channels:
        np_image = np.frombuffer(video_frame, np.uint8).reshape((480, 640, 4))
        
        # Convert RGBA -> BGR

        
        # Then BGR -> HSV
        hsv_frame = cv2.cvtColor(np_image, cv2.COLOR_BGR2HSV)
        return hsv_frame

    def distance_from_mid(self, contour):
        x, y, w, h = cv2.boundingRect(contour)
        mid_x = x + w / 2
        return abs(mid_x - 320)
        
    # Get the blob with the smallest distance from the middle, with the specified color
    def get_smallest_blob(self, color, frame):
        lower, upper = color
        mask = cv2.inRange(frame, lower, upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours, _ = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        # Filter out small contours
        contours = [c for c in contours if cv2.contourArea(c) > 300]  # 🔴
        if not contours:
            return None, None
        smallest_contour = min(contours, key=self.distance_from_mid)
        x, y, w, h = cv2.boundingRect(smallest_contour)
        return (x, y, w, h), smallest_contour

    def get_obstacles(self, frame):
        obstacles = []
        # We'll look for bright "white" region plus any of the other colors
        # The idea is to detect possible obstacles (like the color cubes, etc.)
        obstacle_colors = [((0, 0, 100), (180, 40, 255)), *self.colors[1:]] 

        for lower, upper in obstacle_colors:
            lower = np.array(lower, dtype=np.uint8)
            upper = np.array(upper, dtype=np.uint8)
            mask = cv2.inRange(frame, lower, upper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            contours, _ = cv2.findContours(
                mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for cnt in contours:
                if cv2.contourArea(cnt) > 300:
                    x, y, w, h = cv2.boundingRect(cnt)
                    # Example rule: if it's low in the image, it's an obstacle
                    if y + h > 260:
                        obstacles.append((x, y, w, h))
        return obstacles

    def cube_dropped(self, color, frame):
        lower, upper = color
        # Check the region near the bottom center for presence of that color
        cropped_frame = frame[360:480, 200:420]
        mask = cv2.inRange(cropped_frame, lower, upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours, _ = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        # If color occupies >90% of that cropped region, consider it "dropped"
        needed_area = (120 * 220) * 0.98  # 90% of region
        large_contours = [c for c in contours if cv2.contourArea(c) > needed_area]
        return len(large_contours) > 0

    def bottom_cleared(self, color, frame):
        lower, upper = color
        cropped_frame = frame[380:480, 0:640]
        mask = cv2.inRange(cropped_frame, lower, upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours, _ = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        # If there's a significant contour, not cleared
        contours = [c for c in contours if cv2.contourArea(c) > 300]
        return len(contours) == 0  # True if bottom is free of that color
        
    def right_cleared(self, color, frame):
        lower, upper = color
        cropped_frame = frame[0:480, 0:640]
        mask = cv2.inRange(cropped_frame, lower, upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours, _ = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        contours = [c for c in contours if cv2.contourArea(c) > 300]
        return len(contours) == 0
    
    def is_spot(self, contour):
        if cv2.contourArea(contour) > 40000:
            return True
        return False


                            
    # ==================== Movement / Motor Commands =====================

    def turn_right(self):
        # Example turning logic: left motor forward, right motor backward
        self.left_motor.setVelocity(-1.0)
        self.right_motor.setVelocity(1.0)

    def turn_left(self):
        # Example turning logic: left motor backward, right motor forward
        self.left_motor.setVelocity(1.0)
        self.right_motor.setVelocity(-1.0)
    
    def curve(self):
        
        self.left_motor.setVelocity(-1.0)
        self.right_motor.setVelocity(-2.0)

    def move(self, speed, radius=None):
        velo = -speed/50
        # Simple forward or backward: positive speed is forward, negative is reverse
        # For Webots, this sets the velocity in [rad/s] or [units/s] depending on your config.
        # The 'radius' argument is not used here, but we keep the signature for consistency.
        # Increase or adjust factors as needed for your robot.
        self.left_motor.setVelocity(velo)
        self.right_motor.setVelocity(velo)

    # ====================== Single-Step Logic ===========================
    def run(self):
        """
        Performs one iteration of the color-search / obstacle-avoidance logic.
        This function must be called repeatedly from the main Webots loop.
        """

        # 1) If no more colors left, stop.
        if not self.colors:
            self.move(0, 0)
            return

        frames = self.get_frames()
        if frames is None:
            self.move(0, 0)
            return
        
        hsv_frame = frames
        current_color = self.colors[0]

        # If no special states, proceed with normal search or drop logic
        if not self.avoiding_obstacle and not self.drop_clearing and not self.spot_clearing:
            
            # A) Decide which portion of the frame to look at, depending on whether we have the cube
            if not self.cube_acquired:
                focus_frame = hsv_frame[0:480, 0:640]
                print("Not detected")
                divider_threshold = 380
            else:
                focus_frame = hsv_frame[0:360, 0:640]
                print("detected")
                divider_threshold = 320

            blob, contour = self.get_smallest_blob(current_color, focus_frame)
            
            if blob is not None:
                x, y, w, h = blob
                
                # 1) If blob is up high in the image
                if y + h < divider_threshold:
                    # Check if it's left, right or center
                    mid_x = x + w / 2
                    if mid_x < 300:
                        self.turn_left()
                        return
                    elif mid_x > 340:
                        self.turn_right()
                        return
                    else:
                        # Center => Check for obstacles
                        obstacles = self.get_obstacles(focus_frame)
                        if obstacles:
                            # Start obstacle avoidance
                            self.avoiding_obstacle = True
                            obstacle_mid_x = obstacles[0][0] + obstacles[0][2]/2
                            if obstacle_mid_x < 320:
                                self.avoiding_direction = "right"
                                self.turn_right()
                                return
                            else:
                                self.avoiding_direction = "left"
                                self.turn_left()
                                return
                        else:
                            # Move forward
                            self.move(100, 0)
                            return

                # 2) If blob is in the lower part
                else:
                    if not self.cube_acquired:
                        if self.is_spot(contour):
                            # Contour is a spot
                            self.spot_clearing = True
                            self.turn_left()
                            return
                        # Trying to pick up the cube
                        if y + h >= 460:
                            # Very close => pick it up
                            self.move(0, 0)
                            self.cube_acquired = True
                            return
                        else:
                            # Move forward some more
                            self.move(100, 0)
                            return
                    else:
                        # Already holding a cube => check if we can drop it
                        if self.cube_dropped(current_color, hsv_frame):
                            self.cube_acquired = False
                            self.drop_clearing = True
                            self.move(0, 0)
                            return
                        else:
                            # Not dropped yet => keep going
                            self.move(100, 0)
                            return

            else:
                # No blob => rotate right, searching
                self.turn_left()
                return

        # B) If we are currently avoiding an obstacle
        elif self.avoiding_obstacle:
            obstacles = self.get_obstacles(hsv_frame)
            if obstacles:
                # Still see obstacles => keep turning
                if self.avoiding_direction == "right":
                    self.turn_right()
                    return
                else:
                    self.turn_left()
                    return
            else:
                # No obstacle => do we see the color again?
                blob, _ = self.get_smallest_blob(self.colors[0], hsv_frame)
                if blob is not None:
                    # Return to normal operation
                    self.avoiding_obstacle = False
                    self.avoiding_direction = None
                    return
                else:
                    # Still can't see color => do a detour
                    self.curve()
                    return

        # C) If we just dropped a cube => check if we cleared that color from the bottom
        elif self.drop_clearing:
            if self.bottom_cleared(current_color, hsv_frame):
                # Once bottom is clear, remove this color
                self.drop_clearing = False
                self.cube_acquired = False
                self.colors.pop(0)  # done with this color
                self.move(0, 0)
                return
            else:
                # Not cleared => back up
                self.move(-100, 0)
                return

        # D) If we are "spot clearing"
        else:
            # This was apparently another similar clearing procedure:
            if self.right_cleared(current_color, hsv_frame):
                self.spot_clearing = False
                self.turn_left()
            else:
                # Not cleared => do a turn or something
                self.turn_left()
            return

        # If we ever get here, do nothing special
        self.move(0, 0)

# =======================================================================
if __name__ == "__main__":
    # Create the Robot instance
    robot = Robot()

    # Time step in milliseconds for the simulation
    time_step = 64

    # Get and enable the camera
    camera = robot.getDevice('camera')
    camera.enable(time_step)

    # Get the motors
    left_motor = robot.getDevice("left_wheel")
    right_motor = robot.getDevice("right_wheel")

    # Set infinite position so we can control velocity directly
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))

    # Initialize velocity to zero
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    video_controller = VideoController(camera, left_motor, right_motor)

    # Main simulation loop
    while robot.step(time_step) != -1:
        # Call run() once each step to do exactly one iteration of logic
        video_controller.run()