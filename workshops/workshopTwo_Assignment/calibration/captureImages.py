import cv2
import os
import time

def gstreamer_pipeline():
    return ('nvarguscamerasrc sensor-id=0 ! '
            'video/x-raw(memory:NVMM), format=(string)NV12, framerate=(fraction)1/1 ! '
            'queue max-size-buffers=1 leaky=downstream ! '
            'nvvidconv ! video/x-raw, format=(string)I420 ! '
            'videoconvert ! video/x-raw, format=(string)BGR ! '
            'queue max-size-buffers=1 leaky=downstream ! '
            'appsink drop=true sync=false')


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
