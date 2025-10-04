#!/usr/bin/env python3
import os
import sys
import signal
import gi
import hailo
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject, GLib

Gst.init(None)

# ===== CONFIGURE =====
HEF_PATH = "/usr/local/hailo/resources/models/hailo8l/yolov6n.hef"   # adjust
PP_SO = "/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so"  # adjust
INPUT = "/usr/local/hailo/resources/videos/example_640.mp4"  # or "rtsp://..."
RTSP_URL = "rtsp://camera:c4mp4ss!@192.168.1.131:554/h264Preview_01_sub"
WIDTH = 640
HEIGHT = 640
FPS = "30/1"
BATCH_SIZE = 1
VIDEO_SINK = os.environ.get("GST_VIDEOSINK", "autovideosink") #   # set GST_VIDEOSINK=ximagesink if using X11 forwarding
# =====================

pipeline_str = (
    f'rtspsrc location="{RTSP_URL}" latency=200 ! decodebin !'      # Replace filesrc
    f'videoconvert ! videoscale ! video/x-raw,format=RGB,width=640,height=640,framerate=20/1 ! '
    #f'filesrc location="{INPUT}" name=source !'
    #f'queue leaky=no max-size-buffers=3 max-size-bytes=0 max-size-time=0 ! decodebin ! '
    f'queue name=source_scale_q leaky=no max-size-buffers=3 max-size-bytes=0 max-size-time=0 !'
    f'videoscale add-borders=false name=scale n-threads=2 !'
    f'queue name=source_convert_q leaky=no max-size-buffers=3 max-size-bytes=0 max-size-time=0 !'
    f'videoconvert qos=false n-threads=3 !'
    f'video/x-raw,format=RGB,width={WIDTH},height={HEIGHT},framerate={FPS} ! '
    f'videorate name=source_videorate !'
    f'capsfilter name=source_fps_caps caps="video/x-raw, framerate=20/1" !'
    f'queue name=inf_scale_q leaky=no max-size-buffers=3 max-size-bytes=0 max-size-time=0 !'
    f'videoscale name=inference_videoscale n-threads=2 qos=false ! '
    f'video/x-raw !'
    f'videoconvert name=inference_videoconvert n-threads=2  ! '
    f'queue name=inf_hailonet_q leaky=no max-size-buffers=3 max-size-bytes=0 max-size-time=0 !'
    f'hailonet hef-path={HEF_PATH} batch-size={BATCH_SIZE} vdevice-group-id=1 nms-score-threshold=0.3 nms-iou-threshold=0.45 output-format-type=HAILO_FORMAT_TYPE_FLOAT32 force-writable=true ! '
    f'queue name=inf_hailofilter_q leaky=no max-size-buffers=3 max-size-bytes=0 max-size-time=0 !'
    f'hailofilter so-path={PP_SO} function-name=filter qos=false ! '
    f'queue name=inf_out_q leaky=no max-size-buffers=3 max-size-bytes=0 max-size-time=0 !'
    f'queue name=identity_callback_q leaky=no max-size-buffers=3 max-size-bytes=0 max-size-time=0 !'
    f'identity name=identity_callback !'
    f'queue name=hailo_display_overlay_q leaky=no max-size-buffers=3 max-size-bytes=0 max-size-time=0 !'
    f'hailooverlay name=hailo_display_overlay !'
    f'queue name=hailo_display_videoconvert_q leaky=no max-size-buffers=3 max-size-bytes=0 max-size-time=0 ! '
    f'videoconvert n-threads=2 qos=false  !'
    f'queue name=hailo_display_q leaky=no max-size-buffers=3 max-size-bytes=0 max-size-time=0 ! '
    f'autovideosink sync=true'
)

#pipeline_str=pipeline_str_broken
# User-defined callback function: This is the callback function that will be called when data is available from the pipeline
def app_callback(identity_element, buffer): #pad, info, user_data):
    
    #user_data.increment()  # Using the user_data to count the number of frames
    #string_to_print = f"Frame count: {user_data.get_count()}\n"
    #buffer = info.get_buffer()  # Get the GstBuffer from the probe info
    string_to_print = ""
    if buffer is None:  # Check if the buffer is valid
        print("Buffer invalid")
        return Gst.PadProbeReturn.OK
    for detection in hailo.get_roi_from_buffer(buffer).get_objects_typed(hailo.HAILO_DETECTION):  # Get the detections from the buffer & Parse the detections
        bbox = detection.get_bbox()
        string_to_print += (f"Detection: {detection.get_label()} Confidence: {detection.get_confidence():.2f} BBox: {bbox.xmin():.2f},{bbox.ymin():.2f},{bbox.width():.2f},{bbox.height():.2f}\n")
    print(string_to_print)
    return Gst.PadProbeReturn.OK


print("Pipeline:")
print(pipeline_str)
print("Launching pipeline...")

pipeline = Gst.parse_launch(pipeline_str)

identity_element = pipeline.get_by_name("identity_callback")
identity_element.connect("handoff", app_callback)  # "handoff" signal is fired for each buffer
#identity_element.set_property("app_callback", app_callback)

#identity = Gst.ElementFactory.make("identity", "identity_callback")
#pipeline.add(identity)
#identity_sinkpad = identity.get_static_pad("sink")
#identity_sinkpad.add_probe(Gst.PadProbeType.BUFFER, app_callback, None)


# Bus / messages
bus = pipeline.get_bus()
bus.add_signal_watch()

def on_message(bus, message):
    t = message.type
    if t == Gst.MessageType.EOS:
        print("End-Of-Stream")
        loop.quit()
    elif t == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print("Gst ERROR:", err, debug)
        loop.quit()
print("1")
bus.connect("message", on_message)
print("2")
# Start pipeline and run GLib main loop

pipeline.set_state(Gst.State.PLAYING)
print("3")
loop = GLib.MainLoop()
print("4")
def _sigint(*_):
    print("Caught SIGINT — stopping pipeline")
    pipeline.set_state(Gst.State.NULL)
    loop.quit()

signal.signal(signal.SIGINT, _sigint)
signal.signal(signal.SIGTERM, _sigint)

try:
    loop.run()
finally:
    pipeline.set_state(Gst.State.NULL)
    print("Pipeline stopped, exiting")
