from jebsecrets import Secrets
from tracker import Tracker
import math
import paho.mqtt.client as mqtt
import time

class HeadActuator:
    def __init__(self, reference_point=(500, 350), default_angle=90):
        self.default_angle = default_angle
        self.targetAngle = -2
        self.reference_point = reference_point
        self.last_change_time = 0
        self.min_mqtt_wait_time_ms = 0.04
        self.lastEyes = -1

        # Connect to MQTT server
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(Secrets.mqtt_user, Secrets.mqtt_pass)

        try:
            self.mqtt_client.connect(Secrets.mqtt_server, Secrets.mqtt_port, keepalive=60)
            self.mqtt_client.loop_start()
            self.mqtt_connected = True
            print(f"HeadActuator(): Connected to MQTT broker {Secrets.mqtt_server}:{Secrets.mqtt_port}")
        except Exception as e:
            print(f"HeadActuator(): MQTT connection failed: {e}")
            self.mqtt_connected = False

    def update(self, targetTrack):
        changed=False
        if targetTrack is None:
            newAngle = self.default_angle
            if self.mqtt_connected and self.lastEyes != 0:
                try:
                    self.mqtt_client.publish("halloween/eyes", 0)
                    self.lastEyes = 0
                except Exception as e:
                    print(f"MQTT publish failed: {e}")
        else:
            x, y, w, h = targetTrack.get_state().astype(int)
            bottomMiddleCoord = (x + w / 2, y + h)
            headCoord = self.reference_point
            dx = bottomMiddleCoord[0] - headCoord[0]
            dy = headCoord[1] - bottomMiddleCoord[1]
            # atan2 returns angle from X-axis, so 0° = right; we shift and flip it to 0°=up, 180°=right
            raw_angle = math.degrees(math.atan2(dy, dx))  # -180..180
            if raw_angle < -90:
                raw_angle = 180
            adjusted_angle = (raw_angle)  # make 0° up
            newAngle = max(0, min(180, adjusted_angle))  # clamp to 0–180°
            print(f'HeadActuator.update() TRACK {targetTrack.id}: [{dx:.1f},{dy:.1f}]->{raw_angle}°->{newAngle:.1f}°')


        time_since_change = time.time() - self.last_change_time
        if time_since_change > self.min_mqtt_wait_time_ms and abs(newAngle-self.targetAngle)>=1:
            self.targetAngle = newAngle
            self.last_change_time = time.time()
            # Send to MQTT if connected
            if self.mqtt_connected:
                try:
                    self.mqtt_client.publish("halloween/head_angle", round(self.targetAngle))
                    # print(f"Published {self.targetAngle:.1f}° to halloween/head_angle")
                    if self.lastEyes != 1:
                        self.mqtt_client.publish("halloween/eyes", 1)
                        self.lastEyes = 1
                except Exception as e:
                    print(f"MQTT publish failed: {e}")