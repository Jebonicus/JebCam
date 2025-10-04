import cv2
import numpy as np
from hailo_sdk2 import HailoNetwork, Device

# -----------------------
# Parameters
# -----------------------
RTSP_URL = "rtsp://camera:c4mp4ss!@192.168.1.131:554/h264Preview_01_main"
CONF_THRESHOLD = 0.5  # confidence threshold for detection

# -----------------------
# Initialize Hailo Device and Network
# -----------------------
device = Device.create()
# Replace 'model.hailo' with your compiled Hailo network
network = HailoNetwork.load(device, "/usr/local/hailo/resources/yolov6n_h8l.hef")

# -----------------------
# Open RTSP stream via OpenCV
# -----------------------
cap = cv2.VideoCapture(RTSP_URL)
if not cap.isOpened():
    raise RuntimeError("Failed to open RTSP stream")

# -----------------------
# Label names for your model
# -----------------------
LABELS = ["person", "car"]  # adapt based on your Hailo model

# -----------------------
# Inference + display loop
# -----------------------
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Resize / preprocess for network input
    input_frame = cv2.resize(frame, (network.input_width, network.input_height))
    input_frame = input_frame.astype(np.float32) / 255.0  # normalize
    input_frame = np.transpose(input_frame, (2, 0, 1))  # HWC → CHW
    input_frame = np.expand_dims(input_frame, 0)

    # Run inference
    results = network.infer(input_frame)

    # Post-process detections
    # results format depends on your Hailo model (usually [x1, y1, x2, y2, conf, class])
    for det in results:
        x1, y1, x2, y2, conf, cls_id = det
        if conf < CONF_THRESHOLD:
            continue
        if LABELS[int(cls_id)] not in ["person", "car"]:
            continue
        label = LABELS[int(cls_id)]
        # Scale coordinates back to original frame size
        h_ratio = frame.shape[0] / network.input_height
        w_ratio = frame.shape[1] / network.input_width
        x1, y1, x2, y2 = int(x1*w_ratio), int(y1*h_ratio), int(x2*w_ratio), int(y2*h_ratio)

        # Draw rectangle and label
        color = (0, 255, 0) if label == "person" else (0, 0, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # Display annotated frame
    cv2.imshow("RTSP Hailo Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
network.close()
device.close()
