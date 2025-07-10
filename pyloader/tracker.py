import numpy as np
import cv2
import torch
from lightglue import SuperPoint, LightGlue, ALIKED
from numba import njit

from adaptivenms import square_covering_adaptive_nms


# Camera parameters
K = np.array(
    [
        [1058.1744780806393, 0, 675.570437960496],
        [0, 1058.4470113647467, 334.6606098486689],
        [0, 0, 1],
    ]
)
D = np.array(
    [
        -0.393966826253237,
        0.15803099915873642,
        2.698911244537257e-06,
        0.000535534045712845,
    ]
)

img_size = (1280, 800)


@njit
def simple_nms_numba(keypoints, responses, force_keep, min_dist=30.0, max_num=-1):
    N = keypoints.shape[0]
    order = np.argsort(-responses)

    selected_idx = np.empty(N, dtype=np.int32)
    num_selected = 0

    for i in range(N):
        idx = order[i]
        kp_x = keypoints[idx, 0]
        kp_y = keypoints[idx, 1]

        keep = True
        if not force_keep[idx]:
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


class Tracker:
    def __init__(
        self,
        device="cuda" if torch.cuda.is_available() else "cpu",
        nms_type="adaptive",  # 'adaptive', 'numba', 'none'
        min_dist=30.0,
        use_fmatrix_filter=True,
        min_track_count=1,
        use_clahe=True,
        downscale=1,
        mode="aliked",
    ):
        self.device = device
        self.nms_type = nms_type
        self.min_dist = min_dist
        self.use_fmatrix_filter = use_fmatrix_filter
        self.min_track_count = min_track_count
        self.downscale = downscale

        if use_clahe:
            self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        else:
            self.clahe = None

        self.prev_feats = None
        self.ids = np.array([], dtype=np.int32)
        self.track_cnt = np.array([], dtype=np.int32)
        self.match_cnt = np.array([], dtype=np.int32)
        self.n_id = 0

        if mode == "superpoint":
            self.detector = SuperPoint(max_num_keypoints=1024)
            self.matcher = LightGlue(features="superpoint")
        elif mode == "aliked":
            self.detector = ALIKED(model_name="aliked-n16rot", max_num_keypoints=1024)
            self.matcher = LightGlue(features="aliked")
        else:
            raise ValueError(f"Unsupported mode: {mode}")

        self.detector = self.detector.to(self.device)
        self.detector.eval()
        self.matcher = self.matcher.to(self.device)
        self.matcher.eval()

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

    def _filter_fundamental_matrix(self, prev_pts, cur_pts):
        if len(prev_pts) < 8:
            return np.ones(len(prev_pts), dtype=bool)
        F, mask = cv2.findFundamentalMat(prev_pts, cur_pts, cv2.FM_RANSAC, 1.0, 0.99)
        if mask is None:
            return np.ones(len(prev_pts), dtype=bool)
        if np.sum(mask) < len(prev_pts) * 0.5:
            print("Warning: Too few inliers from fundamental matrix filtering.")
            return np.ones(len(prev_pts), dtype=bool)
        return mask.ravel().astype(bool)

    def track_image(self, cur_img, max_cnt=256):
        if cur_img is None:
            raise ValueError("Image cannot be None.")

        # Convert to grayscale if necessary
        if len(cur_img.shape) == 3 and cur_img.shape[2] == 3:
            cur_img_gray = cv2.cvtColor(cur_img, cv2.COLOR_BGR2GRAY)
        else:
            cur_img_gray = cur_img

        # Undistort
        undist_img = cv2.remap(
            cur_img_gray, self.map1, self.map2, interpolation=cv2.INTER_LINEAR
        )

        # Downscale if needed
        if self.downscale > 1:
            h, w = undist_img.shape[:2]
            new_size = (w // self.downscale, h // self.downscale)
            undist_img = cv2.resize(
                undist_img, new_size, interpolation=cv2.INTER_LINEAR
            )

        # Apply CLAHE if enabled
        if self.clahe:
            undist_img = self.clahe.apply(undist_img)

        # Convert to tensor
        img_tensor = torch.from_numpy(undist_img).float()[None, None] / 255.0
        img_tensor = img_tensor.to(self.device)

        # Detect keypoints
        with torch.no_grad():
            cur_feats_raw = self.detector({"image": img_tensor})

        cur_kpts = cur_feats_raw["keypoints"][0].cpu().numpy()
        cur_scores = cur_feats_raw["keypoint_scores"][0].cpu().numpy()

        print(f"Found {len(cur_kpts)} features in current image.")

        cur_ids = np.full(len(cur_kpts), -1, dtype=np.int32)
        cur_cnt = np.zeros(len(cur_kpts), dtype=np.int32)
        cur_mcnt = np.zeros(len(cur_kpts), dtype=np.int32)

        if self.prev_feats is not None:
            data = {
                "image0": self.prev_feats,
                "image1": {
                    "keypoints": cur_feats_raw["keypoints"],
                    "descriptors": cur_feats_raw["descriptors"],
                },
            }

            with torch.no_grad():
                match_res = self.matcher(data)

            matches0 = match_res["matches0"][0].cpu().numpy()
            matching_scores0 = match_res["matching_scores0"][0].cpu().numpy()

            valid = matches0 >= 0
            idx_prev_all = np.where(valid)[0]
            idx_cur_all = matches0[valid]

            print(f"Matched total of {len(idx_prev_all)}")

            if self.use_fmatrix_filter and len(idx_prev_all) >= 8:
                prev_pts = self.prev_feats["keypoints"][0, idx_prev_all].cpu().numpy()
                cur_pts = cur_feats_raw["keypoints"][0, idx_cur_all].cpu().numpy()
                mask_inliers = self._filter_fundamental_matrix(prev_pts, cur_pts)
                idx_prev = idx_prev_all[mask_inliers]
                idx_cur = idx_cur_all[mask_inliers]
                print(
                    f"Fundamental matrix filtering kept {len(idx_prev)} inlier matches."
                )
            else:
                idx_prev = idx_prev_all
                idx_cur = idx_cur_all
                if self.use_fmatrix_filter:
                    print(
                        "Not enough matches for fundamental matrix filtering, skipping."
                    )

            # Assign IDs and counts to matched keypoints
            cur_ids[idx_cur] = self.ids[idx_prev]
            cur_cnt[idx_cur] = self.track_cnt[idx_prev]
            cur_mcnt[idx_cur] = self.match_cnt[idx_prev] + 1

        # Compute responses
        responses = np.zeros(len(cur_kpts), dtype=np.float32)
        if np.any(cur_cnt > 0):
            responses += 1e3 * (cur_cnt / (np.max(cur_cnt) + 1e-6))
        if np.any(cur_mcnt > 0):
            responses += 1e2 * (cur_mcnt / (np.max(cur_mcnt) + 1e-6))
        if self.prev_feats is not None and len(idx_cur) > 0:
            matching_scores_filtered = matching_scores0[idx_prev]
            mscores_norm = matching_scores_filtered / (
                np.max(matching_scores_filtered) + 1e-6
            )
            responses[idx_cur] += 1e1 * mscores_norm
        responses += 1e0 * (cur_scores / (np.max(cur_scores) + 1e-6))

        if self.nms_type == "adaptive":
            target_num_kpts = min(max_cnt, cur_kpts.shape[0] - 1)
            if target_num_kpts < 1:
                return np.array([]), np.array([]), np.array([]), np.array([])
            sidx = square_covering_adaptive_nms(
                cur_kpts,
                responses,
                cur_img.shape[1],
                cur_img.shape[0],
                target_num_kpts=target_num_kpts,
                up_tol=0,
                indices_only=True,
                max_num_iter=100,
            )[:target_num_kpts]
            print(f"Adaptive NMS selected {len(sidx)} keypoints.")
        elif self.nms_type == "numba":
            force_keep = np.zeros(len(cur_kpts), dtype=bool)
            if self.prev_feats is not None:
                force_keep[idx_cur] = True
            sidx = simple_nms_numba(
                cur_kpts, responses, force_keep, min_dist=self.min_dist, max_num=max_cnt
            )
            print(f"Numba NMS selected {len(sidx)} keypoints.")
        else:
            sidx = np.argsort(-responses)[: min(max_cnt, len(responses))]
            print(f"No NMS: selected {len(sidx)} top keypoints by response.")

        # Assign IDs to NMS-selected points
        selected_ids = cur_ids[sidx]
        mask_new = selected_ids == -1
        selected_ids[mask_new] = np.arange(self.n_id, self.n_id + np.sum(mask_new))
        self.n_id += np.sum(mask_new)
        selected_cnt = cur_cnt[sidx] + 1
        selected_mcnt = cur_mcnt[sidx]
        print(f"Selected {len(sidx)} keypoints with {np.sum(mask_new)} new IDs.")

        # Build ID and count arrays for all keypoints
        # ids_all = np.full(len(cur_kpts), -1, dtype=np.int32)
        # cnt_all = np.zeros(len(cur_kpts), dtype=np.int32)
        # mcnt_all = np.zeros(len(cur_kpts), dtype=np.int32)
        ids_all = cur_ids.copy()
        cnt_all = cur_cnt.copy()
        mcnt_all = cur_mcnt.copy()
        ids_all[sidx] = selected_ids
        cnt_all[sidx] = selected_cnt
        mcnt_all[sidx] = selected_mcnt

        # Update tracker state
        self.prev_feats = {
            "keypoints": cur_feats_raw["keypoints"],
            "descriptors": cur_feats_raw["descriptors"],
        }
        self.ids = ids_all
        self.track_cnt = cnt_all
        self.match_cnt = mcnt_all

        # Output: apply min track count
        mask_output = cnt_all[sidx] >= self.min_track_count
        selected_kpts = cur_kpts[sidx, :][mask_output]
        # Scale back to original size
        selected_kpts *= self.downscale
        pts_dist = self.redistort_points(selected_kpts)
        x_out = pts_dist[:, 0]
        y_out = pts_dist[:, 1]
        ids_out = selected_ids[mask_output]
        cnt_out = selected_cnt[mask_output]

        return x_out, y_out, ids_out, cnt_out


def draw_tracks(img, x, y, ids, cnt, prev_pts_map=None):
    """
    Draw tracked points and optional motion arrows on the image.

    Args:
        img: Input image (grayscale or BGR).
        x, y: Arrays of keypoint coordinates.
        ids: Array of track IDs.
        cnt: Array of track counts.
        prev_pts_map: Optional dict mapping id -> previous cv2.Point2f (or 2-tuple).
    Returns:
        Annotated BGR image.
    """
    # Ensure BGR image
    if len(img.shape) == 2:
        im_track = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    else:
        im_track = img.copy()

    # Draw keypoints with color by age
    for i in range(len(x)):
        pt = (int(x[i]), int(y[i]))
        length = min(1.0, cnt[i] / 20.0)
        color = (255 * (1 - length), 0, 255 * length)
        cv2.circle(im_track, pt, 2, color, 2)

    # Draw motion arrows if prev_pts_map is provided
    if prev_pts_map is not None:
        for i in range(len(ids)):
            id_val = ids[i]
            if id_val in prev_pts_map:
                prev_pt = prev_pts_map[id_val]
                cur_pt = (int(x[i]), int(y[i]))
                cv2.arrowedLine(
                    im_track,
                    cur_pt,
                    (int(prev_pt[0]), int(prev_pt[1])),
                    (0, 255, 0),
                    1,
                    tipLength=0.2,
                )

    return im_track


def main():
    import argparse
    import h5py
    import hdf5plugin

    parser = argparse.ArgumentParser(
        description="Feature tracker with optional video saving and display."
    )
    parser.add_argument(
        "--input", type=str, required=True, help="Path to input HDF5 file"
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Optional path to save output video (e.g. output.mp4)",
    )
    parser.add_argument(
        "--fps", type=int, default=30, help="Frames per second for output video"
    )
    parser.add_argument(
        "--imshow", action="store_true", help="Display images during processing"
    )
    args = parser.parse_args()

    # Open dataset
    h5 = h5py.File(args.input, "r")
    images = h5["/ovc/left/data"]
    img_size = (images.shape[2], images.shape[1])  # width, height

    # Tracker
    tracker = Tracker()
    prev_pts_map = None

    # Video writer setup
    writer = None
    if args.output:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(args.output, fourcc, args.fps, img_size)

    # Main loop
    for i in range(len(images)):
        img = images[i][:, :, 0]
        print(f"Processing image {i + 1}/{len(images)}")

        x, y, ids, cnt = tracker.track_image(img)
        img_track = draw_tracks(img.copy(), x, y, ids, cnt, prev_pts_map)
        prev_pts_map = {id_: (x_, y_) for id_, x_, y_ in zip(ids, x, y)}

        if writer:
            # Ensure BGR
            if len(img_track.shape) == 2:
                img_track_bgr = cv2.cvtColor(img_track, cv2.COLOR_GRAY2BGR)
            else:
                img_track_bgr = img_track
            writer.write(img_track_bgr)

        if args.imshow:
            cv2.imshow("Image", img)
            cv2.imshow("Image Track", img_track)
            key = cv2.waitKey(1)
            if key == 27:  # ESC to exit
                break

    # Cleanup
    if writer:
        writer.release()
        print(f"Video saved to {args.output}")
    if args.imshow:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
