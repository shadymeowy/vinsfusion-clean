# roslaunch vins vins.launch bag_path:=/datasets/euroc/MH_01_easy.bag config_path:=/datasets/euroc/config/l2_cauchy3.yaml

class TrackerTAPNext:
    def __init__(
        self,
        onnx_path="/datasets/tapnext_128.onnx",
        engine_path="/datasets/tapnext_128_fp16.engine",
        n_tracks=128,
        reset_every=100,
        other_reset=False,
        anms=True,
    ):
        self.fast = cv2.FastFeatureDetector_create()
        self.fast.setNonmaxSuppression(True)
        self.fast.setThreshold(1)

        self.model = TAPNextTRT(onnx_path, engine_path, n_tracks=n_tracks)
        self.is_first_frame = True
        self.n_tracks = n_tracks
        self.reset_every = reset_every
        self.other_reset = other_reseth
        self.anms = anms

        self.K = K
        self.D = D
        self.new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, img_size, 0)
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            K, D, None, self.new_K, img_size, cv2.CV_32FC1
        )
        self.prev_frame = None
        
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
            self.first_frame(frame)

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
            if self.last_reset >= 10 and np.sum(self.is_valid) < self.n_tracks * 0.1:
                print(
                    "#### Resetting model with new query points due to low visibility... ####"
                )
                self.reset(frame)

            elif np.sum(self.is_valid) < 10:
                print(
                    "#### Resetting model with new query points due to very low visibility... ####"
                )
                self.reset(frame)
            elif  (
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
        self.is_valid = visibility # np.logical_and(self.is_valid, visibility)

        # Update validity based on bounds
        self.is_valid = np.logical_and(
            self.is_valid,
            (0 <= self.tracks[:, 0])
            & (self.tracks[:, 0] < self.height)
            & (0 <= self.tracks[:, 1])
            & (self.tracks[:, 1] < self.width),
        )

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

        # redistort points
        pts = self.redistort_points(pts)
        x_valid, y_valid = pts[:, 0], pts[:, 1]

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
        # self.model.run(frame)

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