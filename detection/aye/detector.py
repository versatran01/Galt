from __future__ import print_function, division, absolute_import
from aye.preprocessing import Samples


class Detector(object):
    def __init__(self, clf, scaler):
        self.clf = clf
        self.scaler = scaler

    def detect(self, im_bgr):
        s = Samples(im_bgr)
        X = self.scaler.transform(s.X())
        y = self.clf.predict(X)
        bw = s.y_to_bw(y, to_gray=True)
        # TODO: should bw goes into Samples?
        return s, bw
