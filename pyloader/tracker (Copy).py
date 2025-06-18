import argparse
import hashlib
import os
import shelve
import time
import torch

import cv2
import numpy as np
import numba as nb
from adaptivenms import square_covering_adaptive_nms
from lightglue import ALIKED

try:
    from trt_runner import TAPNextTRT
except ImportError:
    print("TAPNextTRT not found, using ONNX runner instead.")
    TAPNextTRT = None
from onnx_runner import TAPNextONNX

# Camera parameters
# K = np.array(
#     [
#         [1058.1744780806393, 0, 675.570437960496],
#         [0, 1058.4470113647467, 334.6606098486689],
#         [0, 0, 1],
#     ]
# )
# D = np.array(
#     [
#         -0.393966826253237,
#         0.15803099915873642,
#         2.698911244537257e-06,
#         0.000535534045712845,
#     ]
# )
# img_size = (1280, 800)

K = np.array(
    [
        [4.6115862106007575e02, 0, 3.6265929181685937e02],
        [0, 4.5975286598073296e02, 2.4852105668448124e02],
        [0, 0, 1],
    ]
)
D = np.array(
    [
        -2.9545645106987750e-01,
        8.6623215640186171e-02,
        2.0132892276082517e-06,
        1.3924531371276508e-05,
    ]
)
img_size = (752, 480)

global_id_counter = 0


class TrackerTAPNext:
    def __init__(
        self,
        onnx_path="/datasets/tapnext_128.onnx",
        engine_path="/datasets/tapnext_128_fp16.engine",
        n_tracks=128,
        reset_every=100,
        other_reset=False,
        anms=True,
        outlier_elimination=False,
    ):
        self.fast = cv2.FastFeatureDetector_create()
        self.fast.setNonmaxSuppression(True)
        self.fast.setThreshold(1)
        if TAPNextTRT is not None:
            self.model = TAPNextTRT(onnx_path, engine_path, n_tracks=n_tracks)
        else:
            self.model = TAPNextONNX(onnx_path, n_tracks=n_tracks)
        self.is_first_frame = True
        self.n_tracks = n_tracks
        self.reset_every = reset_every
        self.other_reset = other_reset
        self.outlier_elimination = outlier_elimination
        self.anms = anms

        self.K = K
        self.D = D
        self.new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, img_size, 0)
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            K, D, None, self.new_K, img_size, cv2.CV_32FC1
        )
        self.prev_frame = None

        self.aliked = ALIKED(
            pretrained=True, model_name="aliked-n16rot", detection_threshold=0.001
        )
        self.aliked = self.aliked.cuda()
        self.aliked.eval()

    def redistort_points(self, undistorted_pts):
        if len(undistorted_pts) == 0:
            return np.zeros((0, 2), dtype=np.float32)

        undistorted_pts = undistorted_pts.astype(np.float32).reshape(-1, 1, 2)
        norm_pts = cv2.undistortPoints(undistorted_pts, self.new_K, None)

        # Convert (N,1,2) -> (N,3) with z=1
        norm_pts_3d = np.concatenate(
            [
                norm_pts.reshape(-1, 2),
                np.ones((norm_pts.shape[0], 1), dtype=norm_pts.dtype),
            ],
            axis=1,
        )

        distorted = cv2.projectPoints(
            norm_pts_3d, np.zeros(3), np.zeros(3), self.K, self.D
        )[0].reshape(-1, 2)

        return distorted

    def detect_keypoints(self, image):
        # fast
        kpts = self.fast.detect(image, None)
        responses = np.array([kp.response for kp in kpts], dtype=np.float32)
        kpts = np.array([kp.pt for kp in kpts], dtype=np.float32)
        kpts = kpts[:, [1, 0]]
        descs = np.ones(len(kpts))

        # aliked
        # with torch.no_grad():
        #     image = self.image_to_tensor(image)
        #     result = self.aliked.forward({"image": image})
        #     kpts = result["keypoints"][0].cpu().numpy()
        #     kpts = kpts[:, [1, 0]]
        #     responses = result["keypoint_scores"][0].cpu().numpy()
        #     descs = result["descriptors"][0].cpu().numpy()

        print("Detected keypoints:", kpts.shape[0])
        print("Keypoint responses:", responses.shape)

        return kpts, responses, descs

    def reset(self, frame):
        global global_id_counter

        self.width, self.height = frame.shape[1], frame.shape[0]

        self.is_valid = np.zeros_like(self.is_valid)
        # reduce by validity
        tracks_old = self.tracks[self.is_valid]
        ids_old = self.ids[self.is_valid]
        cnt_old = self.cnt[self.is_valid]
        descs_old = self.descs[self.is_valid]
        n_old = len(tracks_old)

        # prioritize old tracks
        responses_old = cnt_old * np.ones(len(tracks_old), dtype=np.float32) + 1000

        # generate new query points
        kpts, responses, descs = self.detect_keypoints(frame)
        print(f"Detected {len(kpts)} keypoints in the first frame.")

        candidate_kpts = np.concatenate([tracks_old, kpts], axis=0)
        candidate_responses = np.concatenate([responses_old, responses], axis=0)
        candidate_ids = np.concatenate(
            [ids_old, -1 * np.ones(len(kpts), dtype=np.int32)],
            axis=0,
        )
        candidate_cnt = np.concatenate(
            [cnt_old, np.zeros(len(kpts), dtype=np.int32)],
            axis=0,
        )
        candidate_descs = np.concatenate(
            [descs_old, descs],
            axis=0,
        )

        if self.anms:
            idx = square_covering_adaptive_nms(
                candidate_kpts[:, [1, 0]],
                candidate_responses,
                self.width,
                self.height,
                target_num_kpts=self.n_tracks,
                up_tol=10,
                indices_only=True,
                max_num_iter=10,
            )[: self.n_tracks]
        else:
            idx = simple_nms(candidate_kpts, candidate_responses, max_num=self.n_tracks)

        if len(idx) < self.n_tracks:
            diff_idx = np.setdiff1d(np.arange(len(candidate_kpts)), idx)[
                : self.n_tracks - len(idx)
            ]
            idx = np.concatenate([idx, diff_idx])

        idx_old = idx[idx < n_old]
        idx_new = idx[idx >= n_old]

        mask_old = np.zeros(len(candidate_kpts), dtype=np.bool_)
        mask_old[idx_old] = True
        # count_old = len(idx_old)
        kpts_old = candidate_kpts[mask_old]
        ids_old = candidate_ids[mask_old]
        cnt_old = candidate_cnt[mask_old]
        descs_old = candidate_descs[mask_old]

        mask_new = np.zeros(len(candidate_kpts), dtype=np.bool_)
        mask_new[idx_new] = True
        count_new = len(idx_new)
        kpts_new = candidate_kpts[mask_new]
        ids_new = np.arange(
            global_id_counter, global_id_counter + count_new, dtype=np.int32
        )
        cnt_new = np.zeros(count_new, dtype=np.int32)
        descs_new = candidate_descs[mask_new]
        global_id_counter += count_new

        self.tracks = np.concatenate([kpts_old, kpts_new], axis=0)
        self.tracks_prev = self.tracks.copy()
        self.ids = np.concatenate([ids_old, ids_new], axis=0)
        self.is_valid = np.ones(len(self.tracks), dtype=np.bool_)
        self.cnt = np.concatenate([cnt_old, cnt_new], axis=0)
        self.descs = np.concatenate([descs_old, descs_new], axis=0)

        # ensure every id is unique
        assert len(self.ids) == np.unique(self.ids).size, (
            "Duplicate IDs found in tracks."
        )

        # Reset model with new query points
        self.last_reset = 0
        self.model.reset(width=self.width, height=self.height, query_points=self.tracks)
        # self.model.run(self.prev_frame)

    def track_image(self, frame_dist, max_cnt=None):
        global global_id_counter
        # assert max_cnt == self.n_tracks

        frame = cv2.remap(
            frame_dist, self.map1, self.map2, interpolation=cv2.INTER_LINEAR
        )
        # self.extract_dense_features(frame)

        if self.is_first_frame:
            self.is_first_frame = False
            return self.first_frame(frame)

        if self.last_reset >= self.reset_every:
            print("#### Resetting model with new query points due to timeout... ####")
            self.reset(frame)

        if self.other_reset:
            cond_min_x = self.tracks[:, 1].max() < self.width * 0.7
            cond_max_x = self.tracks[:, 1].min() > self.width * 0.3
            cond_min_y = self.tracks[:, 0].max() < self.height * 0.7
            cond_max_y = self.tracks[:, 0].min() > self.height * 0.3
            # wx = self.tracks[:, 1].max() - self.tracks[:, 1].min()
            # wy = self.tracks[:, 0].max() - self.tracks[:, 0].min()
            # wxy = wx * wy / (self.width * self.height)
            # cond_wxy = wxy < 0.7
            if self.last_reset >= 25 and np.sum(self.is_valid) < self.n_tracks * 0.4:
                print(
                    "#### Resetting model with new query points due to low visibility... ####"
                )
                self.reset(frame)

            elif self.last_reset >= 10 and np.sum(self.is_valid) < 10:
                print(
                    "#### Resetting model with new query points due to very low visibility... ####"
                )
                self.reset(frame)
            elif self.last_reset >= 25 and (
                cond_min_x or cond_max_x or cond_min_y or cond_max_y
            ):
                print(
                    "#### Resetting model with new query points due to x condition... ####"
                )
                self.reset(frame)

        self.last_reset += 1

        # Run model
        self.tracks, visibility = self.model.run(frame)
        # self.is_valid = np.ones(len(self.tracks), dtype=np.bool_)

        # Update validity of tracks
        self.is_valid = np.logical_and(self.is_valid, visibility)

        # Update validity based on bounds
        self.is_valid = np.logical_and(
            self.is_valid,
            (0 <= self.tracks[:, 0])
            & (self.tracks[:, 0] < self.height)
            & (0 <= self.tracks[:, 1])
            & (self.tracks[:, 1] < self.width),
        )

        # Update validity based on outlier elimination
        # We will use outlier elimination on only valid tracks
        if self.outlier_elimination:
            tracks_prev_valid = self.tracks_prev[self.is_valid][:, [1, 0]]
            tracks_valid = self.tracks[self.is_valid][:, [1, 0]]
            mask = outlier_elimination(tracks_prev_valid, tracks_valid)
            self.is_valid[self.is_valid] = mask

        # Update counts
        self.cnt[self.is_valid] += 1

        # Count valid tracks
        print(f"Visible tracks: {np.sum(self.is_valid)}/{self.n_tracks}")

        x = self.tracks[:, 1]
        y = self.tracks[:, 0]
        x_valid = x[self.is_valid]
        y_valid = y[self.is_valid]
        descs_valid = self.descs[self.is_valid]
        pts = np.stack([x_valid, y_valid], axis=-1)

        # pts_refined, descs_refined, valid_refined = self.refine_tracks(
        #     frame, pts, descs_valid
        # )
        # pts_refined = pts_refined[valid_refined]
        # descs_refined = descs_refined[valid_refined]
        # self.is_valid[self.is_valid] = valid_refined
        # self.descs[self.is_valid] = descs_refined
        # pts = pts[valid_refined]
        # self.tracks[self.is_valid] = pts_refined[:, [1, 0]]

        # redistort points
        pts = self.redistort_points(pts)
        x_valid, y_valid = pts[:, 0], pts[:, 1]

        # pts_refined = self.redistort_points(pts_refined)
        # x_refined, y_refined = pts_refined[:, 0], pts_refined[:, 1]

        ids_valid = self.ids[self.is_valid]
        cnt_valid = self.cnt[self.is_valid]

        self.prev_frame = frame
        self.tracks_prev = self.tracks.copy()

        return (
            x_valid,
            y_valid,
            ids_valid,
            cnt_valid,
        )

    def first_frame(self, frame):
        global global_id_counter
        kpts, responses, descs = self.detect_keypoints(frame)
        print(f"Detected {len(kpts)} keypoints in the first frame.")

        self.width, self.height = frame.shape[1], frame.shape[0]

        if self.anms:
            idx = square_covering_adaptive_nms(
                kpts[:, [1, 0]],
                responses,
                self.width,
                self.height,
                target_num_kpts=self.n_tracks,
                up_tol=10,
                indices_only=True,
                max_num_iter=100,
            )[: self.n_tracks]
        else:
            idx = simple_nms(kpts, responses, max_num=self.n_tracks)

        if len(idx) < self.n_tracks:
            diff_idx = np.setdiff1d(np.arange(len(kpts)), idx)[
                : self.n_tracks - len(idx)
            ]
            idx = np.concatenate([idx, diff_idx])
        kpts = kpts[idx]
        responses = responses[idx]
        descs = descs[idx]

        global_id_counter += len(kpts)
        self.ids = np.arange(len(kpts), dtype=np.int32)
        self.cnt = np.zeros(len(kpts), dtype=np.int32)
        self.is_valid = np.ones(len(kpts), dtype=np.bool_)

        # Generate query points from the first frame
        self.model.reset(width=self.width, height=self.height, query_points=kpts)
        self.model.run(frame)

        self.tracks_prev = kpts.copy()
        self.tracks = kpts.copy()
        self.descs = descs.copy()

        self.last_reset = 0

        x = self.tracks[:, 1]
        y = self.tracks[:, 0]
        ids = self.ids.copy()
        cnt = self.cnt.copy()

        # redistort points
        pts = np.stack([x, y], axis=-1)
        pts = self.redistort_points(pts)
        x, y = pts[:, 0], pts[:, 1]

        self.prev_frame = frame

        return x, y, ids, cnt

    def set_outliers(self, ids):
        print("set_outliers called", ids)
        # using self.ids, find which idx they are
        idx = np.isin(self.ids, ids)
        print("Outlier indices:", np.where(idx)[0])
        self.is_valid[idx] = False

    def image_to_tensor(self, frame):
        if isinstance(frame, torch.Tensor):
            return frame
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
        frame = torch.from_numpy(frame).float().permute(2, 0, 1)
        frame = frame.cuda().unsqueeze(0) / 255.0
        return frame

    def refine_tracks(self, frame, pts_prior, descs_prior, radius=10):
        # search prior keypoints in current frame
        kpts, _, descs = self.detect_keypoints(frame)
        kpts = kpts[:, [1, 0]]

        pts_refined = pts_prior.copy()
        descs_refined = descs_prior.copy()
        valid = np.ones(len(pts_refined), dtype=bool)

        refine_tracks(
            pts_prior,
            descs_prior,
            kpts,
            descs,
            pts_refined,
            descs_refined,
            valid,
            radius,
        )

        return pts_refined, descs_refined, valid

    # def extract_dense_features(self, frame):
    #     with torch.no_grad():
    #         frame = self.image_to_tensor(frame)
    #         features, scores = self.aliked.extract_dense_map(frame)
    #         print("features", features.shape, "scores", scores.shape)
    #         cv2.imshow("scores", (scores[0, 0].cpu().numpy() * 255.0).astype(np.uint8))
    #     return features, scores


class TrackerKLT:
    def __init__(self):
        self.prev_img = None
        self.prev_pts = np.empty((0, 2), dtype=np.float32)
        self.ids = np.empty((0,), dtype=np.int32)
        self.track_cnt = np.empty((0,), dtype=np.int32)

        # self.K = K
        # self.D = D
        # self.new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, img_size, 0)
        # self.map1, self.map2 = cv2.initUndistortRectifyMap(
        #     K, D, None, self.new_K, img_size, cv2.CV_32FC1
        # )

        # self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    # def redistort_points(self, undistorted_pts):
    #     if len(undistorted_pts) == 0:
    #         return np.zeros((0, 2), dtype=np.float32)

    #     undistorted_pts = undistorted_pts.astype(np.float32).reshape(-1, 1, 2)
    #     norm_pts = cv2.undistortPoints(undistorted_pts, self.new_K, None)

    #     # Convert (N,1,2) -> (N,3) with z=1
    #     norm_pts_3d = np.concatenate(
    #         [
    #             norm_pts.reshape(-1, 2),
    #             np.ones((norm_pts.shape[0], 1), dtype=norm_pts.dtype),
    #         ],
    #         axis=1,
    #     )

    #     distorted = cv2.projectPoints(
    #         norm_pts_3d, np.zeros(3), np.zeros(3), self.K, self.D
    #     )[0].reshape(-1, 2)

    #     return distorted

    def track_image(self, cur_img_dist, flow_back=True, max_cnt=150, min_dist=30):
        global global_id_counter
        # cur_img = cv2.remap(
        #     cur_img_dist, self.map1, self.map2, interpolation=cv2.INTER_LINEAR
        # )
        cur_img = cur_img_dist.copy()

        if len(cur_img.shape) == 3 and cur_img.shape[2] == 3:
            cur_img = cv2.cvtColor(cur_img, cv2.COLOR_RGB2GRAY)
        # cur_img = self.clahe.apply(cur_img)

        cur_pts = np.empty((0, 2), dtype=np.float32)

        if len(self.prev_pts) > 0:
            cur_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                self.prev_img,
                cur_img,
                self.prev_pts,
                None,
                winSize=(21, 21),
                maxLevel=3,
            )
            status = status.reshape(-1)

            if flow_back:
                reverse_pts = self.prev_pts.copy()
                reverse_pts, reverse_status, _ = cv2.calcOpticalFlowPyrLK(
                    cur_img,
                    self.prev_img,
                    cur_pts,
                    reverse_pts,
                    winSize=(21, 21),
                    maxLevel=1,
                    criteria=(
                        cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS,
                        30,
                        0.01,
                    ),
                    flags=cv2.OPTFLOW_USE_INITIAL_FLOW,
                )
                reverse_status = reverse_status.reshape(-1)
                status &= reverse_status & (
                    np.linalg.norm(self.prev_pts - reverse_pts, axis=1) <= 0.5
                )

            width, height = cur_img.shape[1], cur_img.shape[0]
            cur_pts_rounded = np.round(cur_pts).astype(np.int32)
            status &= cur_pts_rounded[:, 0] >= 1
            status &= cur_pts_rounded[:, 0] < width - 1
            status &= cur_pts_rounded[:, 1] >= 1
            status &= cur_pts_rounded[:, 1] < height - 1

            # fundamental matrix outlier elimination
            # if len(self.prev_pts) > 8:
            #     mask = outlier_elimination(
            #         self.prev_pts[status == 1], cur_pts[status == 1]
            #     )
            #     status[status == 1] = mask

            print(f"Status after filtering: {np.sum(status)} points are valid.")

            self.prev_pts = self.prev_pts[status == 1]
            cur_pts = cur_pts[status == 1]
            self.ids = self.ids[status == 1]
            self.track_cnt = self.track_cnt[status == 1]

        self.track_cnt += 1

        mask = np.full_like(cur_img, 255, dtype=np.uint8)

        if len(cur_pts) > 0:
            sorted_indices = np.argsort(self.track_cnt)[::-1]
            track_cnt_sorted = self.track_cnt[sorted_indices]
            ids_sorted = self.ids[sorted_indices]
            cur_pts_sorted = cur_pts[sorted_indices]

            self.track_cnt = []
            self.ids = []
            cur_pts = []
            for cnt, pt, id_ in zip(track_cnt_sorted, cur_pts_sorted, ids_sorted):
                pti = np.round(pt).astype(np.int32)
                if mask[pti[1], pti[0]] != 255:
                    continue
                cv2.circle(mask, (pti[0], pti[1]), min_dist, 0, -1)
                self.track_cnt.append(cnt)
                self.ids.append(id_)
                cur_pts.append(pt)

            self.track_cnt = np.array(self.track_cnt, dtype=np.int32)
            self.ids = np.array(self.ids, dtype=np.int32)
            cur_pts = np.array(cur_pts, dtype=np.float32)

        n_max_cnt = max_cnt - len(cur_pts)
        print(f"Finding new points, max count: {n_max_cnt}")
        if n_max_cnt > 0:
            new_pts = cv2.goodFeaturesToTrack(
                cur_img,
                maxCorners=n_max_cnt,
                qualityLevel=0.01,
                minDistance=min_dist,
                mask=mask,
            )
            if new_pts is None:
                new_pts = np.empty((0, 1, 2), dtype=np.float32)

            for pt in new_pts:
                pt = pt.ravel()
                cur_pts = np.vstack((cur_pts, pt))
                self.ids = np.append(self.ids, global_id_counter)
                self.track_cnt = np.append(self.track_cnt, 1)
                global_id_counter += 1

        self.prev_img = cur_img.copy()
        self.prev_pts = cur_pts.copy()

        x_out = cur_pts[:, 0].copy()
        y_out = cur_pts[:, 1].copy()
        ids_out = self.ids.copy()
        cnt_out = self.track_cnt.copy()

        # redistort points
        # pts = np.column_stack((x_out, y_out))
        # distorted_pts = self.redistort_points(pts)
        # x_out = distorted_pts[:, 0]
        # y_out = distorted_pts[:, 1]

        return x_out, y_out, ids_out, cnt_out

    def set_outliers(self, ids):
        pass


class TrackerALIKED:
    def __init__(
        self,
        aliked_type="aliked-n32",
        n_tracks=256,
        outlier_elimination=True,
        reset_every=100,
        other_reset=True,
        anms=False,
        onnx_path="/datasets/tapnext.onnx",
        engine_path="/datasets/tapnext_fp16.engine",
    ):
        if TAPNextTRT is not None:
            self.model = TAPNextTRT(onnx_path, engine_path, n_tracks=n_tracks)
        else:
            self.model = TAPNextONNX(onnx_path, n_tracks=n_tracks)
        self.reset_every = reset_every
        self.other_reset = other_reset
        self.anms = anms

        self.K = K
        self.D = D
        self.new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, img_size, 0)
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            K, D, None, self.new_K, img_size, cv2.CV_32FC1
        )
        self.n_tracks = n_tracks
        self.outlier_elimination = outlier_elimination

        self.kpts_prev = np.empty((0, 2), dtype=np.float32)
        self.responses_prev = np.empty((0,), dtype=np.float32)
        self.descs_prev = np.empty((0, 256), dtype=np.float32)
        self.ids_prev = np.empty((0,), dtype=np.int32)
        self.cnt_prev = np.empty((0,), dtype=np.int32)

        self.aliked = ALIKED(
            aliked_type=aliked_type, pretrained=True, detection_threshold=0.001
        )
        self.aliked = self.aliked.cuda()
        self.aliked.eval()

    def first_frame(self, frame):
        global global_id_counter

        kpts, responses, descs = self.detect_keypoints(frame, top=None)
        if not self.anms:
            idx = simple_nms(kpts, responses, max_num=self.n_tracks)
        else:
            idx = square_covering_adaptive_nms(
                kpts[:, [1, 0]],
                responses,
                self.width,
                self.height,
                target_num_kpts=self.n_tracks,
                up_tol=10,
                indices_only=True,
                max_num_iter=10,
            )[: self.n_tracks]
        if len(idx) < self.n_tracks:
            diff_idx = np.setdiff1d(np.arange(len(kpts)), idx)[
                : self.n_tracks - len(idx)
            ]
            idx = np.concatenate([idx, diff_idx])

        n = len(idx)
        self.kpts_prev = kpts[idx]
        self.responses_prev = responses[idx]
        self.descs_prev = descs[idx]
        self.ids_prev = np.arange(global_id_counter, global_id_counter + n)
        self.cnt_prev = np.zeros(n, dtype=np.int32)
        global_id_counter += n

        self.model.reset(
            width=self.width, height=self.height, query_points=self.kpts_prev
        )
        self.model.run(frame)
        self.ids_tapnext = self.ids_prev.copy()
        self.last_reset = 0

        pts = self.kpts_prev[:, [1, 0]]
        pts = self.redistort_points(pts)
        x = pts[:, 0]
        y = pts[:, 1]

        return (
            x,
            y,
            self.ids_prev,
            self.cnt_prev,
            # np.ones(len(self.ids_prev), dtype=bool),
        )

    def should_reset(self, kpts, visibility):
        if self.last_reset >= self.reset_every:
            return True

        if not self.other_reset:
            return False

        cond_min_x = kpts[:, 1].max() < self.width * 0.7
        cond_max_x = kpts[:, 1].min() > self.width * 0.3
        cond_min_y = kpts[:, 0].max() < self.height * 0.7
        cond_max_y = kpts[:, 0].min() > self.height * 0.3
        # wx = self.tracks[:, 1].max() - self.tracks[:, 1].min()
        # wy = self.tracks[:, 0].max() - self.tracks[:, 0].min()
        # wxy = wx * wy / (self.width * self.height)
        # cond_wxy = wxy < 0.7
        if self.last_reset >= 25 and np.sum(visibility) < self.n_tracks * 0.4:
            print(
                "#### Resetting model with new query points due to low visibility... ####"
            )
            return True

        elif self.last_reset >= 10 and np.sum(visibility) < 10:
            print(
                "#### Resetting model with new query points due to very low visibility... ####"
            )
            return True
        elif self.last_reset >= 25 and (
            cond_min_x or cond_max_x or cond_min_y or cond_max_y
        ):
            print(
                "#### Resetting model with new query points due to x condition... ####"
            )
            return True

        return False

    def track_image(self, frame_dist):
        global global_id_counter

        frame = cv2.remap(
            frame_dist, self.map1, self.map2, interpolation=cv2.INTER_LINEAR
        )

        self.width, self.height = frame.shape[1], frame.shape[0]

        if len(frame.shape) == 3 and frame.shape[2] == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        if len(self.kpts_prev) == 0:
            return self.first_frame(frame)

        kpts_new, responses_new, descs_new = self.detect_keypoints(frame, top=None)
        print(f"Detected {len(kpts_new)} keypoints in the current frame.")
        if len(kpts_new) == 0:
            return self.first_frame(frame)

        if self.last_reset >= self.reset_every:
            self.model.reset(
                width=self.width, height=self.height, query_points=self.kpts_prev
            )
            self.model.run(self.prev_frame)
            self.ids_tapnext = self.ids_prev.copy()
            self.last_reset = 0
            print("#################### RESETTING TIME ####################")

        self.last_reset += 1
        self.prev_frame = frame

        print("previous descs", self.descs_prev.shape)
        print("current descs", descs_new.shape)
        m = np.matmul(self.descs_prev, descs_new.T)
        idx = np.argmax(m, axis=1)
        conf = np.max(m, axis=1)
        mask_conf = conf > 0.05

        kpts_aliked = kpts_new[idx]
        descs_aliked = descs_new[idx]

        kpts_tapnext, visibility = self.model.run(frame)
        # take visible points only
        kpts_tapnext = kpts_tapnext[visibility]
        ids_tapnext = self.ids_tapnext[visibility]

        if self.should_reset(kpts_tapnext, visibility):
            self.last_reset = 1e7

        # find ids which are in ids_tapnext
        mask_tapnext = np.isin(self.ids_prev, ids_tapnext)

        # combined two masks so we can take tapnext or aliked
        mask = np.logical_or(mask_conf, mask_tapnext)
        kpts_matched = kpts_aliked[mask]
        descs_matched = descs_aliked[mask]
        ids_matched = self.ids_prev[mask]
        responses_matched = self.responses_prev[mask]
        track_cnt_matched = self.cnt_prev[mask] + 1

        # use tapnext points if available
        _, idx_in_tap, idx_in_matched = np.intersect1d(
            ids_tapnext, ids_matched, assume_unique=False, return_indices=True
        )
        from_tapnext = np.zeros(len(ids_matched), dtype=bool)
        from_tapnext[idx_in_matched] = True
        kpts_matched[idx_in_matched] = kpts_tapnext[idx_in_tap]
        print(
            f"#################### {len(idx_in_tap)} / {len(ids_tapnext)} ####################"
        )

        if self.outlier_elimination and len(kpts_matched) >= 8:
            kpts_matched_prev = self.kpts_prev[mask]
            mask_oe = outlier_elimination(kpts_matched_prev, kpts_matched)
            kpts_matched = kpts_matched[mask_oe]
            descs_matched = descs_matched[mask_oe]
            ids_matched = ids_matched[mask_oe]
            responses_matched = responses_matched[mask_oe]
            track_cnt_matched = track_cnt_matched[mask_oe]

        print(f"Outlier ratio: {np.sum(mask_oe)}/{len(mask_oe)}")

        candidate_kpts = np.concatenate([kpts_matched, kpts_new], axis=0)
        candidate_responses = np.concatenate(
            [
                responses_matched + 1000 * track_cnt_matched,
                responses_new,
            ],
            axis=0,
        )
        candidate_ids = np.concatenate(
            [ids_matched, -1 * np.ones(len(kpts_new), dtype=np.int32)],
            axis=0,
        )
        candidate_cnt = np.concatenate(
            [track_cnt_matched, np.zeros(len(kpts_new), dtype=np.int32)],
            axis=0,
        )
        candidate_descs = np.concatenate(
            [descs_matched, descs_new],
            axis=0,
        )

        if not self.anms:
            idx = simple_nms(candidate_kpts, candidate_responses, max_num=self.n_tracks)
        else:
            idx = square_covering_adaptive_nms(
                candidate_kpts[:, [1, 0]],
                candidate_responses,
                self.width,
                self.height,
                target_num_kpts=self.n_tracks,
                up_tol=10,
                indices_only=True,
                max_num_iter=10,
            )[: self.n_tracks]

        if len(idx) < self.n_tracks:
            diff_idx = np.setdiff1d(np.arange(len(candidate_kpts)), idx)[
                : self.n_tracks - len(idx)
            ]
            idx = np.concatenate([idx, diff_idx])

        idx_matched = idx[idx < len(kpts_matched)]
        idx_new = idx[idx >= len(kpts_matched)]

        mask_matched = np.zeros(len(candidate_kpts), dtype=np.bool_)
        mask_matched[idx_matched] = True
        # count_old = len(idx_matched)
        kpts_matched = candidate_kpts[mask_matched]
        ids_matched = candidate_ids[mask_matched]
        cnt_matched = candidate_cnt[mask_matched]
        descs_matched = candidate_descs[mask_matched]

        mask_new = np.zeros(len(candidate_kpts), dtype=np.bool_)
        mask_new[idx_new] = True
        count_new = len(idx_new)
        kpts_new = candidate_kpts[mask_new]
        ids_new = np.arange(
            global_id_counter, global_id_counter + count_new, dtype=np.int32
        )
        cnt_new = np.zeros(count_new, dtype=np.int32)
        descs_new = candidate_descs[mask_new]
        global_id_counter += count_new

        self.kpts_prev = np.concatenate([kpts_matched, kpts_new], axis=0)
        self.ids_prev = np.concatenate([ids_matched, ids_new], axis=0)
        self.cnt_prev = np.concatenate([cnt_matched, cnt_new], axis=0)
        self.descs_prev = np.concatenate([descs_matched, descs_new], axis=0)

        pts = self.kpts_prev[:, [1, 0]]
        ids = self.ids_prev.copy()
        cnt = self.cnt_prev.copy()

        pts = self.redistort_points(pts)
        x = pts[:, 0]
        y = pts[:, 1]

        return x, y, ids, cnt  # , from_tapnext

    def redistort_points(self, undistorted_pts):
        if len(undistorted_pts) == 0:
            return np.zeros((0, 2), dtype=np.float32)

        undistorted_pts = undistorted_pts.astype(np.float32).reshape(-1, 1, 2)
        norm_pts = cv2.undistortPoints(undistorted_pts, self.new_K, None)

        # Convert (N,1,2) -> (N,3) with z=1
        norm_pts_3d = np.concatenate(
            [
                norm_pts.reshape(-1, 2),
                np.ones((norm_pts.shape[0], 1), dtype=norm_pts.dtype),
            ],
            axis=1,
        )

        distorted = cv2.projectPoints(
            norm_pts_3d, np.zeros(3), np.zeros(3), self.K, self.D
        )[0].reshape(-1, 2)

        return distorted

    def detect_keypoints(self, image, top=None):
        with torch.no_grad():
            image = self.image_to_tensor(image)
            result = self.aliked.forward({"image": image})
            kpts = result["keypoints"][0]
            kpts = kpts[:, [1, 0]]
            responses = result["keypoint_scores"][0]
            descs = result["descriptors"][0]

        print("Detected keypoints:", kpts.shape[0])
        print("Keypoint responses:", responses.shape)

        if top is not None and len(kpts) > top:
            idx = torch.topk(responses, top)
            kpts = kpts[idx.indices].cpu().numpy()
            responses = responses[idx.indices].cpu().numpy()
            descs = descs[idx.indices].cpu().numpy()
        else:
            kpts = kpts.cpu().numpy()
            responses = responses.cpu().numpy()
            descs = descs.cpu().numpy()

        return kpts, responses, descs

    def image_to_tensor(self, frame):
        if isinstance(frame, torch.Tensor):
            return frame
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
        frame = torch.from_numpy(frame).float().permute(2, 0, 1)
        frame = frame.cuda().unsqueeze(0) / 255.0
        return frame


class TrackerRaw:
    def __init__(self, *args, **kwargs):
        mode = os.environ.get("TRACKER_MODE", "tapnext_klt_512").lower()
        if mode == "klt":
            self.tracker1 = None
            self.tracker2 = None
            self.tracker3 = TrackerKLT()
        elif mode == "tapnext_double":
            self.tracker1 = TrackerTAPNext(
                outlier_elimination=False, other_reset=False, reset_every=25
            )
            self.tracker2 = TrackerTAPNext(
                outlier_elimination=False, other_reset=False, reset_every=25
            )
            self.tracker3 = None
        elif mode == "tapnext_klt_o_512":
            self.tracker1 = TrackerTAPNext(
                onnx_path="/datasets/tapnext_512.onnx",
                engine_path="/datasets/tapnext_512_fp16.engine",
                n_tracks=512,
                outlier_elimination=True,
                other_reset=True,
                reset_every=10,
            )
            self.tracker2 = TrackerKLT()
            self.tracker3 = None
        elif mode == "tapnext_klt_o_256":
            self.tracker1 = TrackerTAPNext(
                onnx_path="/datasets/tapnext.onnx",
                engine_path="/datasets/tapnext_fp16.engine",
                n_tracks=256,
                outlier_elimination=True,
                other_reset=True,
                reset_every=10,
            )
            self.tracker2 = TrackerKLT()
            self.tracker3 = None
        elif mode == "tapnext_klt_o_128":
            self.tracker1 = TrackerTAPNext(
                onnx_path="/datasets/tapnext_128.onnx",
                engine_path="/datasets/tapnext_128_fp16.engine",
                n_tracks=128,
                outlier_elimination=True,
                other_reset=True,
                reset_every=100,
            )
            self.tracker2 = TrackerKLT()
            self.tracker3 = None
        elif mode == "tapnext_klt_512":
            self.tracker1 = TrackerTAPNext(
                onnx_path="/datasets/tapnext_512.onnx",
                engine_path="/datasets/tapnext_512_fp16.engine",
                n_tracks=512,
                outlier_elimination=False,
                other_reset=True,
                reset_every=100,
            )
            self.tracker2 = TrackerKLT()
            self.tracker3 = None
        elif mode == "tapnext_klt_256":
            self.tracker1 = TrackerTAPNext(
                onnx_path="/datasets/tapnext.onnx",
                engine_path="/datasets/tapnext_fp16.engine",
                n_tracks=256,
                outlier_elimination=False,
                other_reset=True,
                reset_every=100,
            )
            self.tracker2 = TrackerKLT()
            self.tracker3 = None
        elif mode == "tapnext_klt_128":
            self.tracker1 = TrackerTAPNext(
                onnx_path="/datasets/tapnext_128.onnx",
                engine_path="/datasets/tapnext_128_fp16.engine",
                n_tracks=128,
                outlier_elimination=False,
                other_reset=True,
                reset_every=100,
            )
            self.tracker2 = TrackerKLT()
            self.tracker3 = None
        elif mode == "tapnext":
            self.tracker1 = TrackerTAPNext(
                outlier_elimination=False, other_reset=True, reset_every=100
            )
            self.tracker2 = None
            self.tracker3 = None
        elif mode == "aliked_klt_256":
            self.tracker1 = TrackerALIKED(
                aliked_type="aliked-n16rot",
                n_tracks=256,
                anms=False,
                outlier_elimination=True,
                other_reset=True,
                onnx_path="/datasets/tapnext.onnx",
                engine_path="/datasets/tapnext_fp16.engine",
            )
            self.tracker2 = TrackerKLT()
            self.tracker3 = None
        elif mode == "aliked_256":
            self.tracker1 = TrackerALIKED(
                aliked_type="aliked-n16rot",
                n_tracks=256,
                anms=False,
                outlier_elimination=True,
                other_reset=True,
                onnx_path="/datasets/tapnext.onnx",
                engine_path="/datasets/tapnext_fp16.engine",
            )
            self.tracker2 = None
            self.tracker3 = None
        elif mode == "aliked_klt_nms_256":
            self.tracker1 = TrackerALIKED(
                aliked_type="aliked-n16rot",
                n_tracks=256,
                anms=True,
                outlier_elimination=True,
                other_reset=True,
                onnx_path="/datasets/tapnext.onnx",
                engine_path="/datasets/tapnext_fp16.engine",
            )
            self.tracker2 = TrackerKLT()
            self.tracker3 = None
        elif mode == "aliked_nms_256":
            self.tracker1 = TrackerALIKED(
                aliked_type="aliked-n16rot",
                n_tracks=256,
                anms=True,
                outlier_elimination=True,
                other_reset=True,
                onnx_path="/datasets/tapnext.onnx",
                engine_path="/datasets/tapnext_fp16.engine",
            )
            self.tracker2 = None
            self.tracker3 = None
        else:
            raise ValueError(f"Unknown tracker mode: {mode}")

        self.mode = mode
        self.counter = 0

    def dummy(self):
        return (
            np.zeros((0), dtype=np.float32),
            np.zeros((0), dtype=np.float32),
            np.zeros((0,), dtype=np.int32),
            np.zeros((0,), dtype=np.int32),
        )

    def track_image(self, frame_dist, max_cnt=None):
        if self.tracker1 is None:
            x1, y1, ids1, cnt1 = self.dummy()
        else:
            x1, y1, ids1, cnt1 = self.tracker1.track_image(frame_dist)
        if self.tracker2 is None:
            x2, y2, ids2, cnt2 = self.dummy()
        else:
            x2, y2, ids2, cnt2 = self.tracker2.track_image(frame_dist)
        if self.tracker3 is None:
            x3, y3, ids3, cnt3 = self.dummy()
        else:
            x3, y3, ids3, cnt3 = self.tracker3.track_image(frame_dist)

        # Combine results
        x = np.concatenate([x1, x2, x3])
        y = np.concatenate([y1, y2, y3])
        ids = np.concatenate([ids1, ids2, ids3])
        cnt = np.concatenate([cnt1, cnt2, cnt3])
        self.counter += 1

        # check ids are unique
        unique_ids, unique_indices = np.unique(ids, return_index=True)
        assert len(unique_ids) == len(ids), "IDs are not unique!"

        return x, y, ids, cnt

    def set_outliers(self, ids):
        print("set_outliers called", ids)
        if self.mode == "klt":
            self.tracker3.set_outliers(ids)
        elif self.mode == "tapnext_double":
            self.tracker1.set_outliers(ids)
            self.tracker2.set_outliers(ids)
        elif self.mode.startswith("tapnext_klt"):
            self.tracker1.set_outliers(ids)
            self.tracker2.set_outliers(ids)
        elif self.mode == "tapnext":
            self.tracker1.set_outliers(ids)
        else:
            raise ValueError(f"Unknown tracker mode: {self.mode}")


class TrackerCached:
    def __init__(self):
        self.tracker = TrackerRaw()
        self.path = os.environ.get("TRACKER_CACHE_PATH", "tracker_cache.shelve")
        if os.path.exists(self.path):
            self.saving_cache = False
        else:
            self.saving_cache = True

    def track_image(self, frame_dist, max_cnt=None):
        with shelve.open(self.path, writeback=True) as cache:
            key = frame_dist.tobytes()
            key = hashlib.blake2b(key).hexdigest()
            if key not in cache:
                print(f"Cache miss for key: {key}")
                x, y, ids, cnt = self.tracker.track_image(frame_dist, max_cnt=max_cnt)
                cache[key] = (x, y, ids, cnt)
            else:
                print(f"Cache hit for key: {key}")
                x, y, ids, cnt = cache[key]
            return x, y, ids, cnt


def outlier_elimination(kpts_1, kpts_2):
    # Remove outliers based on fundamental matrix estimation
    assert len(kpts_1) == len(kpts_2)

    if len(kpts_1) < 8:
        return np.zeros(len(kpts_1), dtype=np.bool_)
    kpts_1 = kpts_1.astype(np.float32)
    kpts_2 = kpts_2.astype(np.float32)
    _, mask = cv2.findFundamentalMat(kpts_1, kpts_2, cv2.FM_RANSAC, 2.0, 0.99)
    if mask is None:
        return np.ones(len(kpts_1), dtype=np.bool_)
    return mask.flatten().astype(np.bool_)


@nb.njit(parallel=True)
def refine_tracks(
    pts_prior, descs_prior, pts, descs, pts_refined, descs_refined, valid, radius=30
):
    radius2 = radius * radius
    rejection2 = (radius / 2) * (radius / 2)
    for i in nb.prange(len(pts_prior)):
        best_sim = -1
        best_idx = -1
        best_dist2 = -1
        for j in range(len(pts)):
            dist2 = np.dot(pts[j] - pts_prior[i], pts[j] - pts_prior[i])
            if not (dist2 < radius2):
                continue

            sim = np.dot(descs[j], descs_prior[i])
            if sim > best_sim:
                best_sim = sim
                best_idx = j
                best_dist2 = dist2

        if best_sim > 0.0 and best_dist2 < rejection2:
            pts_refined[i] = pts[best_idx]
            descs_refined[i] = descs[best_idx]
            valid[i] = True
        else:
            pts_refined[i] = pts_prior[i]
            descs_refined[i] = descs_prior[i]
            valid[i] = False


@nb.njit
def simple_nms(keypoints, responses, min_dist=10.0, max_num=-1):
    N = keypoints.shape[0]
    order = np.argsort(-responses)
    selected_idx = np.empty(N, dtype=np.int32)
    num_selected = 0
    for i in range(N):
        idx = order[i]
        kp_x = keypoints[idx, 0]
        kp_y = keypoints[idx, 1]
        keep = True
        # if not force_keep[idx]:
        for j in range(num_selected):
            sel_idx = selected_idx[j]
            sel_x = keypoints[sel_idx, 0]
            sel_y = keypoints[sel_idx, 1]
            dx = kp_x - sel_x
            dy = kp_y - sel_y
            if dx * dx + dy * dy < min_dist * min_dist:
                keep = False
                break
        if keep:
            selected_idx[num_selected] = idx
            num_selected += 1
            if max_num > 0 and num_selected >= max_num:
                break
    return selected_idx[:num_selected]


def draw_tracks(
    img,
    x,
    y,
    ids,
    cnt,
    from_tapnext=None,
    x_refined=None,
    y_refined=None,
    prev_pts_map=None,
    label=False,
):
    image_track = img.copy()
    if len(image_track.shape) == 2:
        image_track = cv2.cvtColor(image_track, cv2.COLOR_GRAY2BGR)
    for i in range(len(x)):
        pt = (int(x[i]), int(y[i]))

        ln = min(1.0, cnt[i] / 20.0)
        color = (255 * (1 - ln), 0, 255 * ln)
        cv2.circle(image_track, pt, 2, color, 2)

        if from_tapnext is not None:
            if from_tapnext[i]:
                cv2.circle(image_track, pt, 5, (0, 255, 0), 1)
            else:
                cv2.circle(image_track, pt, 5, (0, 0, 255), 1)

        if label:
            cv2.putText(
                image_track,
                str(ids[i]),
                (int(pt[0]) + 5, int(pt[1]) + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

        if x_refined is not None and y_refined is not None:
            cv2.circle(
                image_track, (int(x_refined[i]), int(y_refined[i])), 2, (0, 255, 0), 2
            )

        if prev_pts_map and ids[i] in prev_pts_map:
            prev_pt = prev_pts_map[ids[i]]
            cv2.arrowedLine(
                image_track,
                pt,
                (int(prev_pt[0]), int(prev_pt[1])),
                (0, 255, 0),
                1,
                tipLength=0.2,
            )

    return image_track


Tracker = TrackerRaw


def main():
    import h5py
    import hdf5plugin
    import tqdm

    parser = argparse.ArgumentParser(
        description="LK optical flow tracker with optional video saving and display."
    )
    parser.add_argument(
        "--input", type=str, required=True, help="Path to input HDF5 file"
    )
    parser.add_argument(
        "--output", type=str, default=None, help="Optional path to save output video"
    )
    parser.add_argument("--fps", type=int, default=20, help="FPS for saved video")
    parser.add_argument(
        "--imshow", action="store_true", help="Show images during processing"
    )
    args = parser.parse_args()

    if args.input.endswith(".h5"):
        h5 = h5py.File(args.input, "r")
        images = h5["/ovc/left/data"]
        img_size = (images.shape[2], images.shape[1])  # width, height
    else:
        reader = cv2.VideoCapture(args.input)
        h5 = None
        img_size = (
            reader.get(cv2.CAP_PROP_FRAME_WIDTH),
            reader.get(cv2.CAP_PROP_FRAME_HEIGHT),
        )

    # tracker = TrackerTAPNext(
    #     onnx_path="/home/tdemirdal/Others/tapnext/tapnext.onnx",
    #     engine_path="/home/tdemirdal/Others/tapnext/tapnext.engine",
    #     n_tracks=256,
    #     reset_every=100,
    #     other_reset=True,
    #     outlier_elimination=False,
    # )
    tracker = TrackerALIKED(
        aliked_type="aliked-n32",
        n_tracks=512,
    )
    prev_pts_map = None

    writer = None
    if args.output:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(args.output, fourcc, args.fps, img_size)

    i = 0
    while True:
        if h5:
            img = images[i][:, :, 0]
            print(f"Processing image {i + 1}/{len(images)}")
        else:
            ret, img = reader.read()
            if not ret:
                break
        i += 1

        # if i <= 100:
        #     continue

        x, y, ids, cnt, from_tapnext = tracker.track_image(img)
        img_track = draw_tracks(
            img, x, y, ids, cnt, from_tapnext, prev_pts_map=prev_pts_map
        )
        prev_pts_map = {id_: (x_, y_) for id_, x_, y_ in zip(ids, x, y)}

        if writer:
            writer.write(img_track)

        if args.imshow:
            cv2.imshow("Image", img)
            cv2.imshow("Image Track", img_track)
            key = cv2.waitKey(1)
            if key == 27:  # ESC
                break

    if writer:
        writer.release()
        print(f"Video saved to {args.output}")
    if args.imshow:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
