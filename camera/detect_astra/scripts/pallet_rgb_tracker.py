"""Track a YOLO-seeded pallet in new RGB frames without loading YOLO.

Sparse optical flow transports a semantic mask, with independent support and
appearance checks on BOTH front legs. Depth geometry is checked by the caller
on the current registered frame, never carried forward from the seed.
"""
import math

import cv2
import numpy as np

from pallet_target_geometry import _leg_components


class PalletRGBTracker:
    def __init__(self):
        self.reset()

    def reset(self):
        self.active = False
        self.stamp_ns = 0
        self.frames = 0

    def seed(self, image, mask, bbox, stamp_ns):
        if mask.shape != image.shape[:2] or not mask.any():
            raise ValueError('seed RGB/mask shape mismatch or empty mask')
        self.seed_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.seed_mask = mask.astype(np.uint8)
        self.seed_bbox = np.asarray(bbox, np.float32)
        self.seed_legs = []
        for c in _leg_components(mask, bbox):
            leg = np.zeros(mask.shape, np.uint8)
            leg[c['v'], c['u']] = 1
            self.seed_legs.append(leg)
        self.transform = np.eye(3, dtype=np.float64)
        self.gray = self.seed_gray.copy()
        self.mask = self.seed_mask.copy()
        self.bbox = self.seed_bbox.copy()
        self.legs = [leg.copy() for leg in self.seed_legs]
        self._features()
        self.seed_points = self.points.copy()
        self.seed_labels = self.labels.copy()
        self.stamp_ns = stamp_ns
        self.frames = 0
        self.active = True
        return self.mask.astype(bool), self.bbox.copy()

    def _features(self):
        points, labels = [], []
        for index, leg in enumerate(self.legs):
            support = cv2.dilate(leg, np.ones((5, 5), np.uint8))
            found = cv2.goodFeaturesToTrack(self.gray, maxCorners=60, qualityLevel=.01,
                                           minDistance=3, mask=support, blockSize=3)
            if found is None or len(found) < 4:
                raise ValueError('front leg has insufficient RGB tracking features')
            points.append(found)
            labels.extend([index] * len(found))
        self.points = np.concatenate(points).astype(np.float32)
        self.labels = np.array(labels)

    def update(self, image, stamp_ns):
        try:
            return self._update(image, stamp_ns)
        except (ValueError, cv2.error) as exc:
            # A single bad image must invalidate this measurement without
            # permanently discarding a still-recent visual reference. Retry
            # against the same original mask; the last GOOD timestamp stays
            # unchanged, so sustained occlusion expires within 500 ms.
            retryable = isinstance(exc, ValueError) and str(exc).startswith((
                'lost RGB support', 'front legs do not share',
                'front-leg RGB appearance lost', 'RGB optical flow unavailable',
                'RGB reverse optical flow unavailable', 'RGB pallet transform unavailable'))
            recent = self.active and 0 < stamp_ns-self.stamp_ns <= 500_000_000
            if not retryable or not recent:
                self.reset()
            raise

    def _update(self, image, stamp_ns):
        if not self.active:
            raise ValueError('no YOLO seed; enable recognition to acquire a pallet')
        dt = (stamp_ns - self.stamp_ns) / 1e9
        if not 0 < dt <= .5:
            raise ValueError('RGB tracking gap exceeded 500 ms or timestamp went backwards')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        try:
            transform, prior, after, labels, inliers = self._flow_transform(
                self.seed_gray, gray, self.seed_points, self.seed_labels, self.transform)
        except ValueError as original_error:
            # Compensate brightness changes using the last verified leg ROIs.
            # Match the ORIGINAL reference again; never accumulate transforms
            # from a sequence of recent references or relax the gap limit.
            try:
                expected = cv2.warpAffine(self.seed_gray, self.transform[:2],
                    (gray.shape[1], gray.shape[0]), flags=cv2.INTER_LINEAR)
                support = (self.legs[0] | self.legs[1]).astype(bool)
                offset = float(np.median(gray[support].astype(float)-expected[support]))
                reference = np.clip(self.seed_gray.astype(float)+offset,0,255).astype(np.uint8)
                transform, prior, after, labels, inliers = self._flow_transform(
                    reference, gray, self.seed_points, self.seed_labels, self.transform)
            except (ValueError, cv2.error):
                raise original_error
        delta = transform @ np.linalg.inv(self.transform)
        scales = np.linalg.svd(delta[:2, :2], compute_uv=False)
        rotation = math.degrees(math.atan2(delta[1, 0], delta[0, 0]))
        if (np.linalg.det(transform[:2, :2]) <= 0 or scales.min() < .85 or scales.max() > 1.18
                or abs(rotation) > 8 or np.max(np.linalg.norm(after - prior, axis=1)) > 60):
            raise ValueError('RGB pallet motion outside tracking range')
        h, w = gray.shape
        warp = lambda src, flags: cv2.warpAffine(src, transform[:2], (w, h), flags=flags)
        mask = warp(self.seed_mask, cv2.INTER_NEAREST)
        legs = [warp(leg, cv2.INTER_NEAREST) for leg in self.seed_legs]
        expected = warp(self.seed_gray, cv2.INTER_LINEAR)
        corners = np.array([[self.seed_bbox[0], self.seed_bbox[1]],
                            [self.seed_bbox[2], self.seed_bbox[1]],
                            [self.seed_bbox[2], self.seed_bbox[3]],
                            [self.seed_bbox[0], self.seed_bbox[3]]], np.float32)
        corners = cv2.transform(corners[None], transform[:2])[0]
        bbox = np.r_[corners.min(axis=0), corners.max(axis=0)]
        if bbox[0] < 2 or bbox[1] < 2 or bbox[2] > w - 2 or bbox[3] > h - 2:
            raise ValueError('pallet leaving camera view; reacquisition required')
        scores = []
        for leg in legs:
            support = cv2.dilate(leg, np.ones((7, 7), np.uint8)).astype(bool)
            if support.sum() < 80:
                raise ValueError('front leg too small for RGB tracking')
            a, b = expected[support].astype(float), gray[support].astype(float)
            a -= a.mean()
            b -= b.mean()
            denom = np.linalg.norm(a) * np.linalg.norm(b)
            score = float(a @ b / denom) if denom > 1 else 0.0
            if score < .75:
                raise ValueError(f'front-leg RGB appearance lost (correlation {score:.2f})')
            scores.append(score)
        self.gray, self.mask, self.bbox, self.legs = gray, mask, bbox, legs
        self.transform = transform
        self.points, self.labels = after[inliers].reshape(-1, 1, 2), labels[inliers]
        self.frames += 1
        self.stamp_ns = stamp_ns
        self.scores = scores
        return mask.astype(bool), bbox.copy()

    def _flow_transform(self, reference, gray, points, labels, guess):
        flow_args = dict(winSize=(15, 15), maxLevel=3,
                         flags=cv2.OPTFLOW_USE_INITIAL_FLOW,
                         criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 25, .01))
        # Register each frame against the original seed, using the previous
        # transform only as an initial guess. Incrementally warping the last
        # mask can accumulate drift even when the robot is stationary.
        predicted = cv2.transform(points, guess[:2]).astype(np.float32)
        forward, ok, error = cv2.calcOpticalFlowPyrLK(
            reference, gray, points, predicted, **flow_args)
        if forward is None or ok is None:
            raise ValueError('RGB optical flow unavailable')
        backward, back_ok, _ = cv2.calcOpticalFlowPyrLK(
            gray, reference, forward, points.copy(), **flow_args)
        if backward is None or back_ok is None:
            raise ValueError('RGB reverse optical flow unavailable')
        good = ((ok.ravel() != 0) & (back_ok.ravel() != 0)
                & (error.ravel() < 25)
                & (np.linalg.norm(backward - points, axis=2).ravel() <= .8))
        before, after = points[good].reshape(-1, 2), forward[good].reshape(-1, 2)
        labels = labels[good]
        if len(before) < 8 or any(np.sum(labels == i) < 3 for i in (0, 1)):
            raise ValueError('lost RGB support on one or both front legs')
        affine, inliers = cv2.estimateAffine2D(before, after, method=cv2.RANSAC,
                                             ransacReprojThreshold=1.2, maxIters=500,
                                             confidence=.99, refineIters=10)
        if affine is None or inliers is None or not np.isfinite(affine).all():
            raise ValueError('RGB pallet transform unavailable')
        inliers = inliers.ravel().astype(bool)
        if inliers.mean() < .7 or any(np.sum(labels[inliers] == i) < 3 for i in (0, 1)):
            raise ValueError('front legs do not share a consistent RGB motion')
        transform = np.vstack((affine, [0, 0, 1]))
        prior = cv2.transform(before[None], guess[:2])[0]
        return transform, prior, after, labels, inliers
