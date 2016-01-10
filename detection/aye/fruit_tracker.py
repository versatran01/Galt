from __future__ import print_function, division, absolute_import

from aye.visualization import *
from aye.fruit_track import FruitTrack
from aye.assignment import hungarian_assignment
from aye.blob_analysis import num_peaks_in_blob
from aye.bounding_box import bboxes_assignment_cost
from aye.optical_flow import calc_bboxes_flow, calc_average_flow


def get_tracks_bboxes(tracks, filtered=False):
    bboxes = []
    for track in tracks:
        if filtered:
            bboxes.append(track.bbox_filtered)
        else:
            bboxes.append(track.bbox)
    return np.array(bboxes, int)


def predict_tracks_with_flows(tracks, flows, sts):
    valid_tracks = []
    invalid_tracks = []

    for track, flow, st in zip(tracks, flows, sts):
        if st.ravel():
            track.predict(flow)
            valid_tracks.append(track)
        else:
            invalid_tracks.append(track)
    return valid_tracks, invalid_tracks


def add_new_tracks(tracks, blobs, v, min_area):
    for blob in blobs:
        num_fruits = num_peaks_in_blob(blob, v, min_area=min_area)
        track = FruitTrack(blob, num_fruits=num_fruits)
        tracks.append(track)


class FruitTracker(object):
    def __init__(self):
        # Tracking
        self.tracks = []
        self.s = None
        self.blobs = None
        self.min_age = 3
        self.total_counts = 0

        # Optical flow
        self.gray_prev = None
        self.flow_mean = None

        # visualization
        self.disp = None

    def track(self, s, blobs, bw):
        self.s = s
        self.blobs = blobs
        self.disp = np.array(s.im_raw, copy=True)

        # Draw detections
        draw_bboxes(self.disp, blobs['bbox'], Colors.detection)

        h, w = np.shape(bw)
        min_area = (h * w) / (50.0 ** 2)

        if not self.initialized:
            v_bw = np.array(self.v_channel, copy=True)
            v_bw[~(bw > 0)] = 0
            add_new_tracks(self.tracks, blobs, v_bw, min_area=min_area)
            self.gray_prev = cv2.cvtColor(self.s.im_raw, cv2.COLOR_BGR2GRAY)
            return

        # Starting here we do the following things
        # 1. predict new location of each track
        #   a. calculate optical flow at the track's location
        #   b. move tracks to invalid_tracks if optical flow is bad
        #   c. use kalman filter to predict the new location of valid_tracks
        # 2. find correspondences between tracks and detections
        # 3. update tracks
        #   a. for matched tracks and detections, use kalman filter to update
        #      new locations
        #   b. for new detections, spawn a new track and add to valid_tracks
        #   c. for lost tracks, move to invalid_tracks
        #   d. for all invalid_tracks, we see if they can be added to total
        #      counts and then delete them
        # Calculate optical flow from the tracks bounding boxes

        # 1.a
        bboxes_track = get_tracks_bboxes(self.tracks)
        gray = cv2.cvtColor(self.s.im_raw, cv2.COLOR_BGR2GRAY)
        # if self.flow_mean is None:
        # p1s, p2s, sts = calc_bboxes_flow(self.gray_prev, gray, bboxes_track)
        # else:
        h, w = gray.shape
        d = np.sqrt(h * w)
        win_size = int(d / 16)
        if self.flow_mean is None:
            self.flow_mean = np.array([w / 6, 0], np.float32)
        p1s, p2s, sts = calc_bboxes_flow(self.gray_prev, gray, bboxes_track,
                                         win_size=win_size, max_level=4,
                                         guess=self.flow_mean)
        # p1s, p2s, sts = calc_bboxes_flow(self.gray_prev, gray, bboxes_track,
        #                                  win_size=win_size, max_level=4)
        flows = p2s - p1s
        self.flow_mean = calc_average_flow(flows, sts)
        self.gray_prev = gray

        # Draw optical flow
        draw_optical_flow(self.disp, p1s, p2s, color=Colors.optical_flow)

        # 1.b and 1.c
        valid_tracks = []
        invalid_tracks = []
        for track, flow, st in zip(self.tracks, flows, sts):
            if st.ravel():
                track.predict(flow)
                valid_tracks.append(track)
            else:
                invalid_tracks.append(track)

        # Draw prediction
        bboxes_prediction = get_tracks_bboxes(valid_tracks, filtered=True)
        draw_bboxes(self.disp, bboxes_prediction, color=Colors.prediction)

        # 2
        self.tracks = valid_tracks
        bboxes_detection = blobs['bbox']
        cost = bboxes_assignment_cost(bboxes_prediction, bboxes_detection)
        matches, lost_tracks, new_detections = hungarian_assignment(cost)

        # Draw matches
        draw_bboxes_matches(self.disp, matches, bboxes_prediction,
                            bboxes_detection, color=Colors.match)

        # 3.a
        valid_tracks = []
        for match in matches:
            i1, i2 = match
            track = self.tracks[i1]
            blob = self.blobs[i2]
            track.correct(blob)
            valid_tracks.append(track)

        # Draw kalman filter update and tracks that contains multiple fruits
        for track in valid_tracks:
            x, y = np.array(track.x, dtype=int)
            cv2.circle(self.disp, (x, y), 2, color=Colors.correction,
                       thickness=-1)
            if track.num_fruits > 1:
                cv2.putText(self.disp, str(track.num_fruits), (x, y),
                            cv2.FONT_HERSHEY_PLAIN, 1, color=Colors.text)

        # 3.b
        blobs_new = blobs[new_detections]
        v_bw = np.array(self.v_channel, copy=True)
        v_bw[~(bw > 0)] = 0
        add_new_tracks(valid_tracks, blobs_new, v_bw, min_area=min_area)

        # Draw new tracks
        bboxes_new = blobs_new['bbox']
        draw_bboxes(self.disp, bboxes_new, color=Colors.new)

        # 3.c
        for i in lost_tracks:
            invalid_tracks.append(self.tracks[i])

        # 3.d
        self.count_fruits_in_tracks(invalid_tracks)

        self.tracks = valid_tracks

    def finish(self):
        self.count_fruits_in_tracks(self.tracks)

    def count_fruits_in_tracks(self, tracks):
        for track in tracks:
            if track.age >= self.min_age:
                self.total_counts += track.num_fruits

    def get_initialized(self):
        return self.gray_prev is not None

    def get_v_channel(self):
        return self.s.im_hsv[:, :, -1]

    initialized = property(get_initialized)
    v_channel = property(get_v_channel)
