from __future__ import print_function, division, absolute_import

from old import Samples


class FruitDetector(object):
    def __init__(self, clf, scaler, roi, k):
        self.clf = clf
        self.scaler = scaler
        self.roi = roi
        self.k = k

    def detect(self, im_bgr):
        s = Samples(im_bgr, roi=self.roi, k=self.k)
        X = self.scaler.transform(s.X())
        y = self.clf.predict(X)
        bw = s.y_to_bw(y, to_gray=True)
        return s, bw
