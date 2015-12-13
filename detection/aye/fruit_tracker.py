from __future__ import print_function, division, absolute_import
from aye.fruit_track import FruitTrack
from aye.blob_analysis import num_peaks_in_blob


class FruitTracker(object):
    def __init__(self):
        self.tracks = []
        self.s = None

        # Optical flow
        self.prev_gray = None

        # visualization
        self.disp_color = None
        self.disp_bw = None

    def track(self, s, blobs):
        self.s = s
        if not self.initialized:
            self.add_new_tracks(self.tracks, blobs)
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

        print('a')

    def add_new_tracks(self, tracks, blobs):
        for blob in blobs:
            num_fruits = num_peaks_in_blob(blob, self.v_channel)
            track = FruitTrack(blob, num_fruits=num_fruits)
            tracks.append(track)

    def get_initialized(self):
        return self.prev_gray is not None

    def get_v_channel(self):
        return self.s.im_hsv[:, :, -1]

    initialized = property(get_initialized)
    v_channel = property(get_v_channel)
