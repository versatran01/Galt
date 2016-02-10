from __future__ import (print_function, division, absolute_import)

import logging

from scpye.assignment import hungarian_assignment
from scpye.bounding_box import bboxes_assignment_cost, bbox_center
from scpye.fruit_track import FruitTrack
from scpye.optical_flow import calc_optical_flow
from scpye.visualization import *

logging.basicConfig(level=logging.DEBUG)


class FruitTracker(object):
    def __init__(self, min_age=3):
        """
        :param min_age: minimum age of a track to be considered for counting
        """
        # Tracking
        self.tracks = []
        self.min_age = min_age
        self.total_counts = 0

        self.gray_prev = None
        self.win_size = 0
        self.max_level = 3
        self.init_flow = np.zeros(2, np.int)

        self.disp = None
        self.logger = logging.getLogger('fruit_tracker')

    @property
    def initialized(self):
        return self.gray_prev is not None

    def add_new_tracks(self, tracks, fruits):
        """
        Add new fruits to tracks
        :param tracks: a list of tracks
        :param fruits: a list of fruits
        """
        for fruit in fruits:
            track = FruitTrack(fruit, self.init_flow)
            tracks.append(track)
        self.logger.debug('Add {0} new tracks'.format(len(fruits)))

    @staticmethod
    def calc_win_size(gray, k=15):
        """
        Calculate window size for
        :param gray:
        :param k:
        :return: window size
        """
        h, w = gray.shape
        d = np.sqrt(h * w)
        win_size = int(d / k)
        return win_size | 1

    def track(self, image, fruits):
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.disp = image.copy()
        draw_bboxes(self.disp, fruits[:, :4], color=Colors.detect)

        if not self.initialized:
            self.logger.info('Initializing fruit tracker')

            self.gray_prev = gray
            self.win_size = self.calc_win_size(gray)
            self.init_flow = np.array([self.win_size, 0], np.int)
            self.logger.debug(
                'win_size: {0}, init_flow: {1}'.format(self.win_size,
                                                       self.init_flow))

            self.add_new_tracks(self.tracks, fruits)
            self.logger.info('Fruit tracker initialized')
            return

        # Starting here we do the following things
        # 1. predict new location of each track
        self.logger.info('Predict new location of tracks')

        valid_tracks, invalid_tracks = self.predict_tracks(gray)

        self.logger.debug(
            'Tracks valid/invalid: {0}/{1}'.format(len(valid_tracks),
                                                   len(invalid_tracks)))

    def predict_tracks(self, gray):
        # for each track, get center points and flow
        points1 = np.array([bbox_center(track.bbox) for track in self.tracks])
        prev_flows = np.array([track.flow for track in self.tracks])
        points2 = points1 + prev_flows

        # Do optical flow
        # NOTE: dimension of points1 and points2 are 3 because of opencv
        points1, points2, status = calc_optical_flow(self.gray_prev, gray,
                                                     points1, points2,
                                                     self.win_size,
                                                     self.max_level)
        # New optical flow
        flows = points2 - points1
        self.logger.debug('Average flow: {0}'.format(np.mean(flows, axis=0)))

        draw_optical_flow(self.disp, points1, points2, color=Colors.flow)

        valid_tracks = []
        invalid_tracks = []
        for track, flow, stat in zip(self.tracks, flows, status):
            if stat:
                track.predict(np.ravel(flow))
                valid_tracks.append(track)
            else:
                invalid_tracks.append(track)

        # Update gray_prev
        self.gray_prev = gray
        return valid_tracks, invalid_tracks

    def track_old(self, gray, fruits):

        # Starting here we do the following things
        # 1. predict new location of each track
        #   a. calculate optical flow at the track's location
        #   b. move tracks to invalid_tracks if optical flow is bad
        #   c. use kalman filter to predict the new location of valid_tracks
        # 2. find correspondences between valid tracks and fruits
        # 3. update tracks
        #   a. for matched tracks and detections, use kalman filter to update
        #      new locations
        #   b. for new detections, spawn a new track and add to valid_tracks
        #   c. for lost tracks, move to invalid_tracks
        #   d. for all invalid_tracks, we see if they can be added to total
        #      counts and then delete them
        # Calculate optical flow from the tracks bounding boxes

        # Draw prediction
        bboxes_prediction = get_tracks_bboxes(valid_tracks, filtered=True)
        draw_bboxes(self.disp, bboxes_prediction, color=Colors.predict)

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
            cv2.circle(self.disp, (x, y), 2, color=Colors.correct,
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


def get_tracks_bboxes(tracks, filtered=False):
    bboxes = []
    for track in tracks:
        if filtered:
            bboxes.append(track.bbox_filtered)
        else:
            bboxes.append(track.bbox)
    return np.array(bboxes, dtype=np.int)


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
