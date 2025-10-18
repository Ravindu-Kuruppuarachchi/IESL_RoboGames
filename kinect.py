import cv2
import numpy as np
import freenect
from frame_convert2 import video_cv, pretty_depth_cv

def is_cube(contour):
    """Check if the contour could be a cube (3D) rather than a flat square."""
    area = cv2.contourArea(contour)
    if area < 300:  # Too small
        return False
    _, (w, h), _ = cv2.minAreaRect(contour)
    aspect_ratio = float(w) / h if h != 0 else 0
    return 0.8 < aspect_ratio < 1.2  # Roughly square, helps reject non-cube shapes

def get_object_distance(depth, contour):
    """Get the average depth inside the bounding box of the contour."""
    x, y, w, h = cv2.boundingRect(contour)
    roi = depth[y:y+h, x:x+w]
    if roi.size == 0:
        return -1
    return int(np.median(roi[roi > 0]))  # Ignore zero (invalid depth)

def detect_colored_cubes(rgb, depth):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

    # Define color ranges for detection (you can expand this)
    color_ranges = {
        'red': ((0, 100, 100), (10, 255, 255)),
        'green': ((40, 70, 70), (80, 255, 255)),
        'blue': ((100, 150, 0), (140, 255, 255)),
    }

    cubes_info = []

    for color_name, (lower, upper) in color_ranges.items():
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if is_cube(cnt):
                distance = get_object_distance(depth, cnt)
                if distance > 0:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cubes_info.append({
                        "color": color_name,
                        "bounding_box": (x, y, w, h),
                        "distance_mm": distance
                    })
                    # Optionally draw for debugging
                    cv2.rectangle(rgb, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(rgb, f"{color_name} {distance}mm", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    return rgb, cubes_info

def main():
    while True:
        video, _ = freenect.sync_get_video()
        depth, _ = freenect.sync_get_depth()

        if video is None or depth is None:
            print("No video or depth data received.")
            continue

    

        rgb = cv2.cvtColor(video, cv2.COLOR_RGB2BGR)
        depth_mm = depth.astype(np.uint16)

        processed_frame, cubes = detect_colored_cubes(rgb, depth_mm)

        print("Detected Cubes:", cubes)
        cv2.imshow("Detected Cubes", processed_frame)
        #cv2.imshow("Depth", pretty_depth_cv(depth))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    freenect.sync_stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()



