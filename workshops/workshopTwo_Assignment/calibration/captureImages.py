import cv2
import os
import time

def gstreamer_pipeline(sensor_id=0):
    flip_method = 0
    if sensor_id == 0:
        return ('nvarguscamerasrc sensor-id=0 ! '
                'video/x-raw(memory:NVMM), format=(string)NV12, framerate=30/1 ! '
                'nvvidconv flip-method=%d ! '
                'video/x-raw, format=(string)BGRx ! '
                'videoconvert ! '
                'video/x-raw, format=(string)BGR ! appsink' % flip_method)
    
    elif sensor_id == 1:
        return ('v4l2src device=/dev/video1 ! '
                'video/x-raw, format=(string)YUY2, framerate=30/1 ! '
                'videoconvert ! '
                'video/x-raw, format=BGR ! appsink')
    
    else:
        return None

# Create the folder if it doesn't exist
save_folder = "frames"
if not os.path.exists(save_folder):
    os.makedirs(save_folder)

# Open the camera
cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

frame_count = 0

cv2.namedWindow("Captured Frame", cv2.WINDOW_AUTOSIZE)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        cv2.imshow("Captured Frame", frame)  # Show captured image

        filename = os.path.join(save_folder, "frame_{:04d}.jpg".format(frame_count))
        cv2.imwrite(filename, frame)
        print("Saved:", filename)

        frame_count += 1
        time.sleep(1)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    cap.release()
    cv2.destroyAllWindows()
