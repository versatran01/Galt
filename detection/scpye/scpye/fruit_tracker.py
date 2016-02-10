from __future__ import (print_function, division, absolute_import)

import logging

from scpye.assignment import hungarian_assignment
from scpye.bounding_box import bboxes_assignment_cost, bbox_center
from scpye.fruit_track import FruitTrack
from scpye.optical_flow import calc_optical_flow
from scpye.visualization import *

logging.basicConfig(level=logging.INFO)


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
        """
        Main tracking step
        :param image: gray scale image
        :param fruits: new fruits
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.disp = image.copy()

        # ===== DRAW DETECTION =====
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
            return

        valid_tracks, invalid_tracks = self.predict_tracks(gray)

        self.logger.debug(
            'Tracks valid/invalid: {0}/{1}'.format(len(valid_tracks),
                                                   len(invalid_tracks)))
        matched_tracks, lost_tracks = self.match_tracks(valid_tracks, fruits)

        # Update matched tracks
        self.tracks = matched_tracks

        # Count fruits in invalid tracks
        invalid_tracks.extend(lost_tracks)
        self.count_fruits_in_tracks(invalid_tracks)

    def predict_tracks(self, gray):
        """
        Predict tracks using optical flow
        :param gray: gray scale image
        """
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
        # New optical flow, used to update init_flow
        flows = points2 - points1
        self.init_flow = np.squeeze(np.mean(flows, axis=0))
        self.logger.debug('Average flow: {0}'.format(self.init_flow))

        # ===== DRAW OPTICAL FLOW =====
        draw_optical_flows(self.disp, points1, points2, color=Colors.flow)

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

    def match_tracks(self, tracks, fruits):
        """
        Match tracks to fruits
        :param tracks: list of valid tracks
        :param fruits: list of detected fruits
        :return: matched tracks (with new tracks) and lost tracks
        """
        # Get prediction and detection bboxes
        bboxes_prediction = np.array([t.bbox for t in tracks])
        bboxes_detection = fruits[:, :4]

        cost = bboxes_assignment_cost(bboxes_prediction, bboxes_detection)
        match_inds, lost_inds, new_inds = hungarian_assignment(cost)

        # ===== DRAW MATCHES =====
        draw_bboxes_matches(self.disp, match_inds, bboxes_prediction,
                            bboxes_detection, color=Colors.match)

        # Update matched tracks
        matched_tracks = []
        for match in match_inds:
            i_track, i_fruit = match
            track = tracks[i_track]
            fruit = fruits[i_fruit]
            track.correct(fruit)
            matched_tracks.append(track)

        self.logger.debug(
            'Matched/lost/new: {0}/{1}/{2}'.format(len(match_inds),
                                                   len(lost_inds),
                                                   len(new_inds)))

        # Add new tracks
        new_fruits = fruits[new_inds]
        self.add_new_tracks(matched_tracks, new_fruits)

        # ===== DRAW NEW FRUITS =====
        draw_bboxes(self.disp, new_fruits[:, :4], color=Colors.new)

        # Get lost tracks
        lost_tracks = [tracks[ind] for ind in lost_inds]

        return matched_tracks, lost_tracks

    def finish(self):
        """
        Count what's left in tracks, call after final image
        """
        self.count_fruits_in_tracks(self.tracks)

    def count_fruits_in_tracks(self, tracks):
        """
        Count how many fruits there are in tracks
        :param tracks: list of tracks
        """
        temp_sum = 0
        for track in tracks:
            if track.age >= self.min_age:
                temp_sum += track.num

        self.logger.debug('Lost tracks sum: {0}'.format(temp_sum))
        self.total_counts += temp_sum

        # ===== DRAW TOTAL COUNTS =====
        draw_text(self.disp, self.total_counts, (0, len(self.disp) - 5),
                  scale=1, color=Colors.text)
        self.logger.debug('Total counts: {0}'.format(self.total_counts))
