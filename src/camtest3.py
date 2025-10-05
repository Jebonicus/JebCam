#!/usr/bin/env python3
import os
import sys
import signal
import gi
import hailo
import numpy as np
import cv2
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GObject, GLib, GstVideo
from tracker import Tracker
from jebsecrets import Secrets
import threading
import time
import argparse

latest_frame = None
latest_lock = threading.Lock()

class VisualizerThread(threading.Thread):
    def __init__(self, tracker, interval=1.0):
        super().__init__(daemon=True)
        self.tracker = tracker
        self.interval = interval
        self.running = True
        print(f'VisualizerThread() ctor')

    def run(self):
        global latest_lock, latest_frame
        print(f'VisualizerThread.run()')
        #cv2.namedWindow("Track Visualizer", cv2.WINDOW_NORMAL)
        while self.running:
            #print(f'VisualizerThread.run() A')
            with latest_lock:
                #print(f'VisualizerThread.run() B')
                if latest_frame is not None:
                    vis = latest_frame.copy()
                    draw_tracks(vis, self.tracker.get_all_tracks())
                    cv2.imshow("Track Visualizer", vis)
                    cv2.waitKey(1)
                #else:
                #    "Latest frame empty"
            time.sleep(self.interval)

    def stop(self):
        self.running = False
        cv2.destroyAllWindows()

Gst.init(None)

# ===== CONFIGURE =====
HEF_PATH = "/usr/local/hailo/resources/models/hailo8l/yolov8m.hef"   # adjust
#HEF_PATH = "yolov5m6_6.1.hef"
PP_SO = "/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so"  # adjust
#INPUT = "/usr/local/hailo/resources/videos/example_640.mp4"  # or "rtsp://..."
INPUT = "cam_s.mp4"
RTSP_URL = Secrets.RTSP_URL
ORIG_WIDTH=1920
ORIG_HEIGHT=576
CROP_AMOUNT_L=600
CROP_AMOUNT_R=450
TARGET_WIDTH=ORIG_WIDTH-CROP_AMOUNT_L-CROP_AMOUNT_R
WIDTH = 640
HEIGHT = 640
FPS_NUM = 20
FPS = f"{FPS_NUM}/1"
BATCH_SIZE = 4
VIDEO_SINK = os.environ.get("GST_VIDEOSINK", "ximagesink") #  autovideosink # set GST_VIDEOSINK=ximagesink if using X11 forwarding
OUTPUT_FILE = "out.mp4"
RTSP_MOUNT = "/stream"
RTSP_PORT = 8554
# =====================
tracker = Tracker(dist_threshold=120.0, max_age=int(4*FPS_NUM), min_hits=1, dt=1.0)

#1920×576

def buildPipelineStr(enable_rtsp):
    pipeline_str = (
        f'uridecodebin uri="{RTSP_URL}"  ! '
        #f'filesrc location="{INPUT}" name=source !'
        #f'queue leaky=no max-size-buffers=4 max-size-bytes=0 max-size-time=0 ! decodebin ! '
        f'videoconvert ! videoscale ! video/x-raw,format=RGB,width={ORIG_WIDTH},height={ORIG_HEIGHT},framerate={FPS} ! '
        f'videocrop left={CROP_AMOUNT_L} right={CROP_AMOUNT_R} top=0 bottom=0 ! '
        f'videoscale add-borders=true ! video/x-raw,format=RGB,width={WIDTH},height={HEIGHT},framerate={FPS} ! '
        f'queue name=source_scale_q leaky=no max-size-buffers=5 max-size-bytes=0 max-size-time=0 ! '
        f'videoconvert qos=false n-threads=3 ! '
        f'video/x-raw,format=RGB,width={WIDTH},height={HEIGHT},framerate={FPS} ! '
        f'videorate name=source_videorate ! '
        f'queue name=inf_scale_q leaky=no max-size-buffers=4 max-size-bytes=0 max-size-time=0 ! '
        f'videoscale name=inference_videoscale n-threads=2 qos=false ! '
        f'videoconvert ! video/x-raw,width={WIDTH},height={HEIGHT},format=RGB ! '   # <--- Force HEF input size
        f'queue name=inf_hailonet_q leaky=no max-size-buffers=4 max-size-bytes=0 max-size-time=0 ! '
        f'hailonet hef-path={HEF_PATH} batch-size={BATCH_SIZE} vdevice-group-id=1 '
        f'nms-score-threshold=0.3 nms-iou-threshold=0.35 output-format-type=HAILO_FORMAT_TYPE_FLOAT32 force-writable=true ! '
        f'queue name=inf_hailofilter_q leaky=no max-size-buffers=4 max-size-bytes=0 max-size-time=0 ! '
        f'hailofilter so-path={PP_SO} function-name=filter qos=false ! '
        f'queue name=inf_out_q leaky=no max-size-buffers=4 max-size-bytes=0 max-size-time=0 ! '
        f'identity name=identity_callback ! '
        f'queue name=hailo_display_overlay_q leaky=no max-size-buffers=4 max-size-bytes=0 max-size-time=0 ! '
        f'videoscale ! videoconvert ! video/x-raw,width={TARGET_WIDTH},height={ORIG_HEIGHT},format=RGB,framerate={FPS} ! '  # <--- Force overlay size
        f'hailooverlay name=hailo_display_overlay ! '
        f'videoconvert n-threads=2 qos=false ! '
        f'queue name=hailo_display_q leaky=no  max-size-buffers=10 max-size-bytes=0 max-size-time=0 ! '
    )
    if enable_rtsp:
        pipeline_str += (
            #f'appsink name=mysink emit-signals=true max-buffers=1 drop=true'
            f'tee name=t ! '
            f'video/x-raw,width={TARGET_WIDTH},height={ORIG_HEIGHT},format=RGB,framerate={FPS} ! '
            #f'queue ! fakesink sync=false ' # Just consume the frames to keep pipeline running
            f'videoconvert n-threads=2 qos=false ! '
            f'video/x-raw,width={TARGET_WIDTH},height={ORIG_HEIGHT},format=BGR,framerate={FPS} ! '
            f'queue leaky=no max-size-buffers=5 ! appsink name=mysink emit-signals=true max-buffers=1 drop=true '
            #f't. ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! videoconvert ! x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast key-int-max=20 ! rtph264pay config-interval=1 name=pay0 pt=96'
            f't. ! '
            f'queue leaky=no max-size-buffers=5 ! '
            f'video/x-raw,width={TARGET_WIDTH},height={ORIG_HEIGHT},format=RGB,framerate={FPS} ! '
            f'videoconvert  n-threads=2 qos=false ! '
            f'video/x-raw,width={TARGET_WIDTH},height={ORIG_HEIGHT},format=I420,framerate={FPS} ! '
            f'x264enc bitrate=2000 speed-preset=ultrafast tune=zerolatency ! rtph264pay config-interval=1 name=pay0 pt=96 '
            #f'{VIDEO_SINK} sync=true'
            #f'videoconvert ! x264enc bitrate=2000 speed-preset=ultrafast tune=zerolatency ! mp4mux ! filesink location="{OUTPUT_FILE}"'
        )
    else:
        pipeline_str += (
            f'videoconvert n-threads=2 qos=false ! '
            f'video/x-raw,width={TARGET_WIDTH},height={ORIG_HEIGHT},format=BGR,framerate={FPS} ! '
            f'queue leaky=no max-size-buffers=5 ! appsink name=mysink emit-signals=true max-buffers=1 drop=true '
        )
    return pipeline_str

# === Forward frames from tee branch to RTSP appsrc ===
def draw_tracks(frame, tracks):
    # frame: HxWx3 BGR
    if len(tracks) > 0:
        print(f"draw_tracks: {len(tracks)}")
    for t in tracks:
        x,y,w,h = t.get_state().astype(int)
        cv2.rectangle(frame, (x,y), (x+int(w), y+int(h)), (0,255,0), 2)
        cv2.putText(frame, f"ID:{t.id}, {t.label}", (x, y-6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

def on_new_sample(sink, factory=None):
    global tracker, latest_lock, latest_frame
    sample = sink.emit("pull-sample")
    if not sample:
        return Gst.FlowReturn.OK
    buf = sample.get_buffer()
    caps = sample.get_caps()
    info = GstVideo.VideoInfo.new_from_caps(caps)

    n_channels = 3  # RGB
    width = info.width
    height = info.height
    stride = info.stride[0]  # bytes per row (may include padding)

    # Map buffer for reading
    success, map_info = buf.map(Gst.MapFlags.READ)
    if not success:
        return Gst.FlowReturn.ERROR

    # Convert to numpy array
    frame = np.frombuffer(map_info.data, dtype=np.uint8)
    frame = frame[:stride*height]           # keep only valid bytes

    # Reshape using stride
    frame = frame.reshape((height, stride))

    # Crop padding bytes
    frame = frame[:, :width*n_channels]

    # Reshape to HWC
    frame = frame.reshape((height, width, n_channels))

    buf.unmap(map_info)
        # Store the latest frame safely for visualization
    with latest_lock:
        latest_frame = frame.copy()

    #tracks = tracker.get_all_tracks()
    #draw_tracks(frame, tracks)
    #factory.appsrc.emit("push-buffer", buf)
    #print(f'Sending frame...')
    return Gst.FlowReturn.OK

ALLOWED_CLASSES = ["person", "dog", "cat"]
def app_callback(identity_element, buffer):
    #string_to_print = ""
    frame_w = TARGET_WIDTH   # the pixel width your hailo inference is referencing
    frame_h = ORIG_HEIGHT    # pixel height for overlay / tracker coords
    #print(f'app_callback')
    det_boxes = []
    if buffer is None:  # Check if the buffer is valid
        print("Buffer invalid")
        return Gst.PadProbeReturn.OK
    for detection in hailo.get_roi_from_buffer(buffer).get_objects_typed(hailo.HAILO_DETECTION):  # Get the detections from the buffer & Parse the detections
        label = detection.get_label()
        if label not in ALLOWED_CLASSES:
            continue  # Ignore unwanted classes
        bbox = detection.get_bbox()
        x = bbox.xmin() * frame_w
        y = bbox.ymin() * frame_h
        w = bbox.width() * frame_w
        h = bbox.height() * frame_h
        det_boxes.append( {'box':(x, y, w, h), 'label':label} )
        #centerX = bbox.xmin()+(bbox.width()/2)
        #centerY = bbox.ymin()+(bbox.height()/2)

    tracks = tracker.update(det_boxes)
    # print/act on confirmed tracks
    for t in tracks:
        box = t.get_state()   # xywh pixels of predicted/current box
        print(f"TRACK {t.id} label={t.label} box={box} age={t.age} since_update={t.time_since_update}")
        #string_to_print += (f"Detection: {label} Confidence: {detection.get_confidence():.2f} Center: {centerX:.2f},{centerY:.2f} BBox: {bbox.xmin():.2f},{bbox.ymin():.2f},{bbox.width():.2f},{bbox.height():.2f}\n")
    #if len(string_to_print)>0:
    #  print(string_to_print)
    #else:
      #print("No detections")
    return Gst.PadProbeReturn.OK

# ===== RTSP SERVER SETUP =====
class RtspFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, pipeline_str):
        super().__init__()
        self.set_shared(True)
        self.set_suspend_mode(GstRtspServer.RTSPSuspendMode.NONE)
        self.set_latency(0)
        self.set_launch(pipeline_str)
        self.pipeline_str=pipeline_str
        
    def do_create_element(self, url):
        pipeline = Gst.parse_launch(self.pipeline_str)
        print(f'Starting server with pipeline={self.pipeline_str}')
        identity_element = pipeline.get_by_name("identity_callback")
        identity_element.connect("handoff", app_callback)

        appsink = pipeline.get_by_name("mysink")
        if appsink is not None:
            appsink.connect("new-sample", on_new_sample, self)
        print(f'do_create_element done')

        return pipeline

class MyFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self):
        super().__init__()
        self.launch_str = f"appsrc name=appsrc is-live=true format=time do-timestamp=true ! videoconvert ! video/x-raw,format=I420 ! x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast ! rtph264pay name=pay0 pt=96"
        self.set_launch(self.launch_str)
        self.set_shared(True)
        self.set_suspend_mode(GstRtspServer.RTSPSuspendMode.NONE)
        self.set_latency(0)
        self.appsrc = None

    def do_create_element(self, url):
        elem = Gst.parse_launch(self.get_launch())
        self.appsrc = elem.get_child_by_name("appsrc")
        caps = Gst.Caps.from_string(
            f"video/x-raw,format=I420,width={TARGET_WIDTH},height={ORIG_HEIGHT},framerate={FPS}"
        )
        self.appsrc.set_property("caps", caps)
        print(f"Client connected")
        return elem

    def do_configure(self, rtsp_media):
        self.appsrc = rtsp_media.get_element().get_child_by_name("appsrc")
        self.appsrc.set_property("format", Gst.Format.TIME)
        self.appsrc.set_property("is-live", True)
        self.appsrc.set_property("do-timestamp", True)

def main():
    parser = argparse.ArgumentParser(description="JebCam")

    # Define optional boolean flags
    parser.add_argument('--split', action='store_true', help='Enable split pipeline mode (buggy, slow)')
    parser.add_argument('--rtsp', action='store_true', help='Enable RTSP output')

    args = parser.parse_args()

    # Access as boolean variables
    split = args.split
    enable_rtsp = args.rtsp

    pipeline_str = buildPipelineStr(enable_rtsp)
    if split:
        rtsp_factory = MyFactory()
        pipeline = Gst.parse_launch(pipeline_str)

        print(f'Starting split server with pipeline={pipeline_str}')
        identity_element = pipeline.get_by_name("identity_callback")
        identity_element.connect("handoff", app_callback)

        appsink = pipeline.get_by_name("mysink")

        appsink.connect("new-sample", on_new_sample, rtsp_factory)
    elif not enable_rtsp:
        
        pipeline = Gst.parse_launch(pipeline_str)
        identity_element = pipeline.get_by_name("identity_callback")
        identity_element.connect("handoff", app_callback)

        appsink = pipeline.get_by_name("mysink")
        if appsink is not None:
            appsink.connect("new-sample", on_new_sample)
    else:
        rtsp_factory = RtspFactory(pipeline_str)


    if enable_rtsp:
        # Start RTSP server
        server = GstRtspServer.RTSPServer()
        mounts = server.get_mount_points()
        mounts.add_factory(RTSP_MOUNT, rtsp_factory)
        server.attach(None)
        #pipeline.set_state(Gst.State.PLAYING)
        print(f"RTSP server running at rtsp://192.168.1.138:{RTSP_PORT}{RTSP_MOUNT}")

    if not enable_rtsp or split:
        pipeline.set_state(Gst.State.PLAYING)

    # ===== MAIN LOOP =====
    loop = GLib.MainLoop()
    def _sigint(*_):
        print("Caught SIGINT — stopping")
        loop.quit()

    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    cv2.namedWindow("Track Visualizer", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Track Visualizer", TARGET_WIDTH, ORIG_HEIGHT+50)  # width=1280, height=720
    visualizer = VisualizerThread(tracker, interval=0.5)
    visualizer.daemon = True
    visualizer.start()
    loop.run()

if __name__ == "__main__":
    main()