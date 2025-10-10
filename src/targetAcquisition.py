from tracker import Tracker
import time
import math
import threading

class TargetAcquisition(threading.Thread):
    def __init__(self, tracker, targetCallback=None, interval=0.1, hysteresis=10.0, reference_point=(500, 350)): #434, 575
        """
        tracker: the Tracker instance
        interval: seconds between updates
        hysteresis: cost buffer before switching targets
        reference_point: (x, y) coordinate for 'preferred' target zone, typically lower-middle of frame
        """
        super().__init__(daemon=True)
        self.tracker = tracker
        self.interval = interval
        self.hysteresis = hysteresis
        self.reference_point = reference_point
        self.targetCallback = targetCallback
        self.running = True
        self.targetTrack = None
        self.last_switch_time = 0
        self.min_lock_time = 5.0  # seconds, or however long ye want to hold target
        print(f'TargetAcquisition() ctor')

    def run(self):
        while self.running:
            tracks = self.tracker.get_all_tracks()
            track_costs = {}
            time_since_switch = time.time() - self.last_switch_time
            
            # Compute cost for each track
            for t in tracks:
                cost = self.computeTrackCost(t)
                track_costs[t] = cost

            
            # Sort by ascending cost
            sorted_tracks = sorted(track_costs.items(), key=lambda kv: kv[1])

            # Select best target
            if not sorted_tracks:
                self.targetTrack = None
            else:
                best_track, best_cost = sorted_tracks[0]
                current_cost = track_costs.get(self.targetTrack, float('inf'))

                # Detect if current target disappeared from tracking list
                target_lost = (self.targetTrack is not None and self.targetTrack not in track_costs)

                # Switch immediately if target lost,
                # or switch normally using hysteresis + min_lock_time
                should_switch = (
                    self.targetTrack is None
                    or target_lost
                    or (best_cost + self.hysteresis < current_cost and time_since_switch > self.min_lock_time)
                )

                if should_switch:
                    if self.targetTrack != best_track:
                        reason = "target lost" if target_lost else "better track"
                        print(f"[TargetAcquisition] Switching target to track {best_track.id} ({reason}, cost={best_cost:.2f})")
                        self.last_switch_time = time.time()
                        best_track.targetLockDuration = 0.0
                    self.targetTrack = best_track

                # Update lock duration
                if self.targetTrack is not None:
                    self.targetTrack.targetLockDuration += self.interval
                    print(f"[TargetAcquisition] Locked target {self.targetTrack.id} for {self.targetTrack.targetLockDuration:.2f}s (cost={best_cost:.2f})")


            if self.targetCallback is not None:
                self.targetCallback(self.targetTrack)

            time.sleep(self.interval)

    def stop(self):
        self.running = False

    def getTargetTrack(self):
        return self.targetTrack

    def computeTrackCost(self, t):
        """
        Weighted cost combining:
            - distance from reference_point
            - age (frames)
            - since_update (frames)
        Lower is better.
        """
        x, y, w, h = t.get_state().astype(int)
        x_c = x + w / 2
        y_c = y + h / 2
        ref_x, ref_y = self.reference_point

        # Distance from reference point (pixels)
        distance = math.sqrt((x_c - ref_x) ** 2 + (y_c - ref_y) ** 2)

        # Weights for cost components (tune to yer liking, Cap’n)
        w_dist = 1.0      # per pixel
        w_dist_pow = 1.5  # Make distance cost non-linear
        w_age = 2.0       # per frame
        w_update = 5.0    # per frame since last update
        w_sametrack = 100.0

        cost = (w_dist * pow(distance, w_dist_pow)) + (w_age * t.age) + (w_update * t.time_since_update) + (t.targetLockDuration*w_sametrack)
        return cost