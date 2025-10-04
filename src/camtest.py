
# capture_gst.py
import cv2

rtsp_url = "rtsp://camera:c4mp4ss!@192.168.1.131:554/h264Preview_01_main"
gst_pipeline = (
    f'rtspsrc location={rtsp_url} latency=200 ! rtph264depay ! h264parse ! '
    'avdec_h264 ! videoconvert ! appsink max-buffers=1 drop=true'
)

#cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
cap = cv2.VideoCapture(rtsp_url)
if not cap.isOpened():
    raise SystemExit("Failed to open RTSP stream")

while True:
    ret, frame = cap.read()
    if not ret:
        break
    # resize to reduce CPU + inference load
    frame = cv2.resize(frame, (1280, 360))
    # show or send frame to inference pipeline
    cv2.imshow("test", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
