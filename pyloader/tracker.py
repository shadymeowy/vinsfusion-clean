import argparse
import hashlib
import os
import shelve
import time

import cv2
import numpy as np
from adaptivenms import square_covering_adaptive_nms

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
        onnx_path="/datasets/tapnext.onnx",
        engine_path="/datasets/tapnext_fp16.engine",
        n_tracks=256,
        reset_every=100,
        other_reset=False,
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

        self.K = K
        self.D = D
        self.new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, img_size, 0)
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            K, D, None, self.new_K, img_size, cv2.CV_32FC1
        )

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

        return kpts, responses

    def reset(self, frame):
        global global_id_counter

        self.width, self.height = frame.shape[1], frame.shape[0]

        kpts, responses = self.detect_keypoints(frame)
        print(f"Detected {len(kpts)} keypoints in the first frame.")

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

        kpts = kpts[idx]

        if len(kpts) < self.n_tracks:
            # create random keypoints if not enough are detected
            kpts_rand = np.random.rand(self.n_tracks - len(kpts), 2) * np.array(
                [self.height, self.width]
            )
            kpts = np.vstack((kpts, kpts_rand))

        self.ids = np.arange(
            global_id_counter, global_id_counter + len(kpts), dtype=np.int32
        )
        global_id_counter += len(kpts)
        self.cnt = np.zeros(len(kpts), dtype=np.int32)
        self.is_valid = np.ones(len(kpts), dtype=np.bool_)

        # Generate query points from the first frame
        self.model.reset(width=self.width, height=self.height, query_points=kpts.copy())

        self.tracks_prev = kpts.copy()
        self.tracks = kpts.copy()

        self.last_reset = 0

    def track_image(self, frame_dist, max_cnt=None):
        global global_id_counter
        # assert max_cnt == self.n_tracks

        frame = cv2.remap(
            frame_dist, self.map1, self.map2, interpolation=cv2.INTER_LINEAR
        )

        if self.is_first_frame:
            self.is_first_frame = False
            self.reset(frame)
        else:
            if self.last_reset >= self.reset_every:
                print(
                    "#### Resetting model with new query points due to timeout... ####"
                )
                self.reset(frame)

            if self.other_reset:
                cond_min_x = self.tracks[:, 1].max() < self.width * 0.7
                cond_max_x = self.tracks[:, 1].min() > self.width * 0.3
                cond_min_y = self.tracks[:, 0].max() < self.height * 0.7
                cond_max_y = self.tracks[:, 0].min() > self.height * 0.3
                if (
                    self.last_reset >= 10
                    and np.sum(self.is_valid) < self.n_tracks * 0.1
                ):
                    print(
                        "#### Resetting model with new query points due to low visibility... ####"
                    )
                    self.reset(frame)

                elif np.sum(self.is_valid) < 10:
                    print(
                        "#### Resetting model with new query points due to very low visibility... ####"
                    )
                    self.reset(frame)
                elif cond_min_x or cond_max_x or cond_min_y or cond_max_y:
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
            tracks_prev_valid = self.tracks_prev[self.is_valid]
            tracks_valid = self.tracks[self.is_valid]
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
        ids_valid = self.ids[self.is_valid]
        cnt_valid = self.cnt[self.is_valid]

        # redistort points
        pts = np.stack([x_valid, y_valid], axis=-1)
        pts = self.redistort_points(pts)
        x_valid, y_valid = pts[:, 0], pts[:, 1]

        return x_valid, y_valid, ids_valid, cnt_valid


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


class TrackerRaw:
    def __init__(self, *args, **kwargs):
        mode = os.environ.get("TRACKER_MODE", "klt").lower()
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
        elif mode == "tapnext_klt" or mode == "tapnext_klt_7":
            self.tracker1 = TrackerTAPNext(
                outlier_elimination=False, other_reset=True, reset_every=100
            )
            self.tracker2 = TrackerKLT()
            self.tracker3 = None
        elif mode == "tapnext":
            self.tracker1 = TrackerTAPNext(
                outlier_elimination=False, other_reset=True, reset_every=100
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
        if self.mode == "klt":
            x1, y1, ids1, cnt1 = self.tracker3.track_image(frame_dist)
            x2, y2, ids2, cnt2 = self.dummy()
            x3, y3, ids3, cnt3 = self.dummy()
        elif self.mode == "tapnext_double":
            x1, y1, ids1, cnt1 = self.tracker1.track_image(frame_dist)
            if self.counter >= 12:
                x2, y2, ids2, cnt2 = self.tracker2.track_image(frame_dist)
            else:
                x2, y2, ids2, cnt2 = self.dummy()
            x3, y3, ids3, cnt3 = self.dummy()
        elif self.mode == "tapnext_klt" or self.mode == "tapnext_klt_7":
            x1, y1, ids1, cnt1 = self.tracker1.track_image(frame_dist)
            x2, y2, ids2, cnt2 = self.tracker2.track_image(frame_dist)
            x3, y3, ids3, cnt3 = self.dummy()
        elif self.mode == "tapnext":
            x1, y1, ids1, cnt1 = self.tracker1.track_image(frame_dist)
            x2, y2, ids2, cnt2 = self.dummy()
            x3, y3, ids3, cnt3 = self.dummy()
        else:
            raise ValueError(f"Unknown tracker mode: {self.mode}")

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
    _, mask = cv2.findFundamentalMat(kpts_1, kpts_2, cv2.FM_RANSAC, 1.0, 0.99)
    if mask is None:
        return np.ones(len(kpts_1), dtype=np.bool_)
    return mask.flatten().astype(np.bool_)


def draw_tracks(img, x, y, ids, cnt, prev_pts_map=None, label=False):
    image_track = img.copy()
    if len(image_track.shape) == 2:
        image_track = cv2.cvtColor(image_track, cv2.COLOR_GRAY2BGR)
    for i in range(len(x)):
        pt = (int(x[i]), int(y[i]))
        ln = min(1.0, cnt[i] / 20.0)
        color = (255 * (1 - ln), 0, 255 * ln)
        cv2.circle(image_track, pt, 2, color, 2)

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

    h5 = h5py.File(args.input, "r")
    images = h5["/ovc/left/data"]
    img_size = (images.shape[2], images.shape[1])  # width, height

    tracker = TrackerCached()
    prev_pts_map = None

    writer = None
    if args.output:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(args.output, fourcc, args.fps, img_size)

    for i in tqdm.tqdm(range(len(images))):
        img = images[i][:, :, 0]
        print(f"Processing image {i + 1}/{len(images)}")

        x, y, ids, cnt = tracker.track_image(img)
        img_track = draw_tracks(img, x, y, ids, cnt, prev_pts_map)
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
