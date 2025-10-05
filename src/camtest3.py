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

Gst.init(None)

# ===== CONFIGURE =====
HEF_PATH = "/usr/local/hailo/resources/models/hailo8l/yolov8m.hef"   # adjust
#HEF_PATH = "yolov5m6_6.1.hef"
PP_SO = "/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so"  # adjust
#INPUT = "/usr/local/hailo/resources/videos/example_640.mp4"  # or "rtsp://..."
INPUT = "cam_s.mp4"
RTSP_URL = "rtsp://camera:c4mp4ss!@192.168.1.131:554/h264Preview_01_sub"
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

#1920×576

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
    f'video/x-raw,width={TARGET_WIDTH},height={ORIG_HEIGHT},format=I420,framerate={FPS} ! '  # <--- Force overlay size'
    f'queue name=hailo_display_q leaky=no max-size-buffers=10 max-size-bytes=0 max-size-time=0 ! '
    f'appsink name=mysink emit-signals=true max-buffers=1 drop=true'
    #f'tee name=t ! '
    #f'queue ! fakesink sync=false ' # Just consume the frames to keep pipeline running
    #f'queue leaky=downstream max-size-buffers=5 ! appsink name=mysink emit-signals=true max-buffers=1 drop=true '
    #f't. ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! videoconvert ! x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast key-int-max=20 ! rtph264pay config-interval=1 name=pay0 pt=96'
    #f'videoconvert ! x264enc bitrate=2000 speed-preset=ultrafast tune=zerolatency ! rtph264pay config-interval=1 name=pay0 pt=96'
    #f'{VIDEO_SINK} sync=true'
    #f'videoconvert ! x264enc bitrate=2000 speed-preset=ultrafast tune=zerolatency ! mp4mux ! filesink location="{OUTPUT_FILE}"'
)
# === Forward frames from tee branch to RTSP appsrc ===
def on_new_sample(sink, factory):
    sample = sink.emit("pull-sample")
    if not sample or not factory.appsrc:
        return Gst.FlowReturn.OK

    buf = sample.get_buffer()
    factory.appsrc.emit("push-buffer", buf)
    print(f'Sending frame...')
    return Gst.FlowReturn.OK

ALLOWED_CLASSES = ["person", "dog", "cat"]
def app_callback(identity_element, buffer):
    string_to_print = ""
    if buffer is None:  # Check if the buffer is valid
        print("Buffer invalid")
        return Gst.PadProbeReturn.OK
    for detection in hailo.get_roi_from_buffer(buffer).get_objects_typed(hailo.HAILO_DETECTION):  # Get the detections from the buffer & Parse the detections
        label = detection.get_label()
        if label not in ALLOWED_CLASSES:
            continue  # Ignore unwanted classes
        bbox = detection.get_bbox()
        centerX = bbox.xmin()+(bbox.width()/2)
        centerY = bbox.ymin()+(bbox.height()/2)
        string_to_print += (f"Detection: {label} Confidence: {detection.get_confidence():.2f} Center: {centerX:.2f},{centerY:.2f} BBox: {bbox.xmin():.2f},{bbox.ymin():.2f},{bbox.width():.2f},{bbox.height():.2f}\n")
    if len(string_to_print)>0:
      print(string_to_print)
    else:
      print("No detections")
    return Gst.PadProbeReturn.OK

# ===== RTSP SERVER SETUP =====
class RtspFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self):
        super().__init__()
        self.set_shared(True)
        self.set_suspend_mode(GstRtspServer.RTSPSuspendMode.NONE)
        self.set_latency(0)
        self.set_launch(pipeline_str)
        
    def do_create_element(self, url):
        pipeline = Gst.parse_launch(pipeline_str)
        print(f'Starting server with pipeline={pipeline_str}')
        identity_element = pipeline.get_by_name("identity_callback")
        identity_element.connect("handoff", app_callback)
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

rtsp_factory = MyFactory()

pipeline = Gst.parse_launch(pipeline_str)

print(f'Starting server with pipeline={pipeline_str}')
identity_element = pipeline.get_by_name("identity_callback")
identity_element.connect("handoff", app_callback)

appsink = pipeline.get_by_name("mysink")

appsink.connect("new-sample", on_new_sample, rtsp_factory)

# Start RTSP server
server = GstRtspServer.RTSPServer()
mounts = server.get_mount_points()
mounts.add_factory(RTSP_MOUNT, rtsp_factory)
server.attach(None)

pipeline.set_state(Gst.State.PLAYING)

#pipeline.set_state(Gst.State.PLAYING)
print(f"RTSP server running at rtsp://192.168.1.138:{RTSP_PORT}{RTSP_MOUNT}")
# ===== MAIN LOOP =====
loop = GLib.MainLoop()
def _sigint(*_):
    print("Caught SIGINT — stopping")
    loop.quit()

signal.signal(signal.SIGINT, _sigint)
signal.signal(signal.SIGTERM, _sigint)

loop.run()