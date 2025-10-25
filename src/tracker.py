# tracker.py
import numpy as np
import math
import itertools
import time

_next_id = itertools.count(1)

def _xywh_to_cxcy(box):
    # box: (x, y, w, h) with x,y in pixels (top-left)
    x, y, w, h = box
    return np.array([x + w/2.0, y + h/2.0, w, h], dtype=float)

def _cxcy_to_xywh(cx, cy, w, h):
    return np.array([cx - w/2.0, cy - h/2.0, w, h], dtype=float)

class KalmanFilterCV:
    """
    Simple constant-velocity Kalman filter for centroid (cx,cy) and velocities vx,vy.
    State vector: [cx, cy, vx, vy]^T
    Measurement: [cx, cy]^T
    """
    def __init__(self, dt=1.0, process_var=1.0, meas_var=4.0):
        # dt: time step (assume 1 per frame unless you pass FPS)
        self.dt = dt
        # State transition
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=float)
        # Measurement matrix
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=float)
        # Process noise covariance
        q = process_var
        G = np.array([[dt*dt/2, 0],[0, dt*dt/2],[dt,0],[0,dt]], dtype=float)
        self.Q = (G @ G.T) * q
        # Measurement noise covariance
        self.R = np.eye(2, dtype=float) * meas_var
        # Covariance
        self.P = np.eye(4, dtype=float) * 500.0
        # state
        self.x = np.zeros((4,1), dtype=float)

    def initiate(self, cx, cy, vx=0.0, vy=0.0):
        self.x = np.array([[cx],[cy],[vx],[vy]], dtype=float)
        self.P = np.eye(4, dtype=float) * 10.0

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x.copy()

    def update(self, meas):
        # meas: [cx, cy]
        z = np.array(meas, dtype=float).reshape((2,1))
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        y = z - self.H @ self.x
        self.x = self.x + K @ y
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.H) @ self.P

    def current_state(self):
        return self.x.flatten()  # cx, cy, vx, vy

class Track:
    def __init__(self, data, frame_idx, dt=1.0, extrapolate_time=0.0):
        """
        bbox_xywh: (x,y,w,h,label) top-left pixel coords
        extrapolate_time: How far in the future to predict the track's location as for get_state
        """
        bbox_xywh = data['box']
        self.id = next(_next_id)
        self.kf = KalmanFilterCV(dt=dt)
        cxcywh = _xywh_to_cxcy(bbox_xywh)
        self.label = data['label']
        cx, cy, w, h = cxcywh
        self.kf.initiate(cx, cy, 0.0, 0.0)
        self.w = float(w)
        self.h = float(h)
        self.age = 1
        self.hits = 1      # number of total hits
        self.time_since_update = 0
        self.last_frame = frame_idx
        self.history = []  # optional: store past boxes
        self.targetLockDuration = 0.0
        self.extrapolate_time = extrapolate_time

    def predict(self):
        s = self.kf.predict().flatten()
        cx, cy = s[0], s[1]
        return _cxcy_to_xywh(cx, cy, self.w, self.h)

    def update(self, bbox_xywh, frame_idx):
        cxcywh = _xywh_to_cxcy(bbox_xywh)
        cx, cy, w, h = cxcywh
        self.kf.update([cx, cy])
        # Update box size as simple smoothing
        self.w = 0.8 * self.w + 0.2 * float(w)
        self.h = 0.8 * self.h + 0.2 * float(h)
        self.time_since_update = 0
        self.hits += 1
        self.age += 1
        self.last_frame = frame_idx
        self.history.append((frame_idx, bbox_xywh))

    def mark_missed(self):
        self.time_since_update += 1
        self.age += 1

    def get_state(self, dt: float = None):
        """
        Return current or future predicted bounding box.
        If dt > 0, predict position dt seconds into the future using constant velocity model.
        """
        if dt is None:
            dt = getattr(self, "extrapolate_time", 0.0)
        s = self.kf.current_state()
        cx, cy, vx, vy = s

        if dt > 0.0:
            cx += vx * dt
            cy += vy * dt

        return _cxcy_to_xywh(cx, cy, self.w, self.h)

class Tracker:
    def __init__(self,
                 iou_threshold=0.3,
                 dist_threshold=100.0,
                 max_age=30,
                 min_hits=3,
                 dt=1.0,
                 exclusions=[],
                 extrapolate_time=0.0):
        """
        - iou_threshold: optional if using IoU matching
        - dist_threshold: maximum centroid distance for matching (pixels)
        - max_age: frames to keep a track without updates
        - min_hits: frames required to confirm a track
        - dt: time step for KF (1.0 per frame by default; set 1/fps if you have accurate FPS)
        - exclusions: array of (x,y,w,h) boxes to use as exclusion zones for suppressing creation of new tracks
        - extrapolate_time: Duration(s) in which to extrapolate future positions for get_state calls
        """
        self.tracks = []
        self.iou_th = iou_threshold
        self.dist_th = dist_threshold
        self.max_age = max_age
        self.min_hits = min_hits
        self.frame_idx = 0
        self.dt = dt
        self.exclusions = exclusions
        self.extrapolate_time = extrapolate_time

    @staticmethod
    def _centroid(box):
        x, y, w, h = box
        return np.array([x + w/2.0, y + h/2.0], dtype=float)

    @staticmethod
    def _euclidean(a, b):
        return np.linalg.norm(a - b)

    def update(self, detections_xywh):
        """
        detections_xywh: list of boxes (x,y,w,h) in pixels for this frame
        Returns: list of active tracks (only confirmed ones)
        """
        self.frame_idx += 1
        # Predict all tracks
        for t in self.tracks:
            t.predict()
            t.mark_missed()  # will be reset on update if matched

        if len(detections_xywh) == 0:
            # No detections: age all tracks and remove old ones
            self.tracks = [t for t in self.tracks if t.time_since_update <= self.max_age]
            return [t for t in self.tracks if t.hits >= self.min_hits]

        # Build cost matrix (centroid distance)
        det_centroids = np.array([self._centroid(d['box']) for d in detections_xywh])
        tr_centroids = np.array([self._centroid(t.get_state(0)) for t in self.tracks]) if len(self.tracks) > 0 else np.array([])

        if len(self.tracks) == 0:
            # create tracks for all detections
            for d in detections_xywh:
                if not self.is_excluded(d):
                    self.tracks.append(Track(d, frame_idx=self.frame_idx, dt=self.dt, extrapolate_time=self.extrapolate_time))
            return [t for t in self.tracks if t.hits >= self.min_hits]

        cost = np.zeros((len(self.tracks), len(detections_xywh)), dtype=float)
        for i, tr in enumerate(self.tracks):
            trc = tr_centroids[i]
            for j, dc in enumerate(det_centroids):
                cost[i, j] = self._euclidean(trc, dc)

        # Greedy assignment: for simplicity and robustness on Pi (no scipy)
        # We'll perform greedy matching with threshold
        matched_tr = set()
        matched_det = set()
        pairs = []
        # Flatten pairs by increasing cost
        inds = np.dstack(np.unravel_index(np.argsort(cost, axis=None), cost.shape))[0]
        for (i, j) in inds:
            if i in matched_tr or j in matched_det:
                continue
            if cost[i, j] > self.dist_th:
                continue
            matched_tr.add(i); matched_det.add(j)
            pairs.append((i, j))

        # Update matched
        for (i, j) in pairs:
            self.tracks[i].update(detections_xywh[j]['box'], frame_idx=self.frame_idx)

        # Unmatched detections -> create new tracks
        for j, det in enumerate(detections_xywh):
            if j not in matched_det and not self.is_excluded(det):
                self.tracks.append(Track(det, frame_idx=self.frame_idx, dt=self.dt, extrapolate_time=self.extrapolate_time))

        # Remove dead tracks
        self.tracks = [t for t in self.tracks if t.time_since_update <= self.max_age]

        # Return confirmed tracks
        confirmed = [t for t in self.tracks if t.hits >= self.min_hits]
        return confirmed
    
    def is_excluded(self, det_xywh, iou_threshold=0.3):
        """
        Return True if IOU of det_xywh is greater than a threshold
        for any of the 'exclusions' boxes.
        self.exclusions is a list of (x, y, w, h) bounding boxes.
        """
        x, y, w, h = det_xywh['box']
        det_x1, det_y1, det_x2, det_y2 = x, y, x + w, y + h

        for ex in self.exclusions:
            ex_x, ex_y, ex_w, ex_h = ex
            ex_x1, ex_y1, ex_x2, ex_y2 = ex_x, ex_y, ex_x + ex_w, ex_y + ex_h

            # Compute intersection
            inter_x1 = max(det_x1, ex_x1)
            inter_y1 = max(det_y1, ex_y1)
            inter_x2 = min(det_x2, ex_x2)
            inter_y2 = min(det_y2, ex_y2)

            inter_w = max(0, inter_x2 - inter_x1)
            inter_h = max(0, inter_y2 - inter_y1)
            inter_area = inter_w * inter_h

            # Compute areas and IoU
            det_area = w * h
            ex_area = ex_w * ex_h
            union_area = det_area + ex_area - inter_area
            iou = inter_area / union_area if union_area > 0 else 0.0

            # Check against threshold
            if iou > iou_threshold:
                # Arrr! Ye be in forbidden waters!
                #print(f"[Exclusion] Detection excluded (IoU={iou:.2f} > {iou_threshold:.2f}) with {det_xywh['box']} -> exclusion box {ex}")
                return True
            #else:
                #print(f"[Exclusion] nope (IoU={iou:.2f} > {iou_threshold:.2f}) with {det_xywh['box']} -> exclusion box {ex}")
        return False

    def get_all_tracks(self):
        """Return all tracks (including unconfirmed)"""
        return list(self.tracks)
