import cv2
import numpy as np


class KLTTracker:
    def __init__(self):
        self.prev_img = None
        self.prev_pts = np.empty((0, 2), dtype=np.float32)
        self.ids = np.empty((0,), dtype=np.int32)
        self.track_cnt = np.empty((0,), dtype=np.int32)
        self.n_id = 0
        self.clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(16, 16))

    def track_image(self, cur_img, flow_back=True, max_cnt=150, min_dist=30):
        print("IAM HERE")
        if cur_img is None:
            raise ValueError("Image0 cannot be None.")

        if len(cur_img.shape) == 3 and cur_img.shape[2] == 3:
            cur_img = cv2.cvtColor(cur_img, cv2.COLOR_BGR2GRAY)
        cur_img = self.clahe.apply(cur_img)
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

                # Update status based on reverse flow and distance
                status &= reverse_status & (
                    np.linalg.norm(self.prev_pts - reverse_pts, axis=1) <= 0.5
                )

            # Update status based on border check
            width = cur_img.shape[1]
            height = cur_img.shape[0]
            cur_pts_rounded = np.round(cur_pts).astype(np.int32)
            status &= cur_pts_rounded[:, 0] >= 1
            status &= cur_pts_rounded[:, 0] < width - 1
            status &= cur_pts_rounded[:, 1] >= 1
            status &= cur_pts_rounded[:, 1] < height - 1

            print(f"Status after filtering: {np.sum(status)} points are valid.")

            # Filter out points based on status
            self.prev_pts = self.prev_pts[status == 1]
            cur_pts = cur_pts[status == 1]
            self.ids = self.ids[status == 1]
            self.track_cnt = self.track_cnt[status == 1]

        # Update track counts
        self.track_cnt += 1

        # Create a mask for goodFeaturesToTrack
        mask = np.zeros_like(cur_img, dtype=np.uint8) + 255

        # Draw mask circles at current points
        # First sort track_cnt,ids and cur_pts by track_cnt
        if len(cur_pts) > 0:
            sorted_indices = np.argsort(self.track_cnt)[::-1]
            track_cnt_sorted = self.track_cnt[sorted_indices]
            ids_sorted = self.ids[sorted_indices]
            cur_pts_sorted = cur_pts[sorted_indices]

            self.track_cnt = []
            self.ids = []
            cur_pts = []
            for cnt, pt, id_ in zip(track_cnt_sorted, cur_pts_sorted, ids_sorted):
                pti1 = np.round(pt).astype(np.int32)
                pti2 = pt.astype(np.int32)
                if mask[pti2[1], pti2[0]] != 255:
                    continue
                cv2.circle(mask, (pti1[0], pti1[1]), min_dist, 0, -1)
                self.track_cnt.append(cnt)
                self.ids.append(id_)
                cur_pts.append(pt)

            self.track_cnt = np.array(self.track_cnt, dtype=np.int32)
            self.ids = np.array(self.ids, dtype=np.int32)
            cur_pts = np.array(cur_pts, dtype=np.float32)

        # Find new points to track
        n_max_cnt = max_cnt - len(self.prev_pts)
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
                self.ids = np.append(self.ids, self.n_id)
                self.track_cnt = np.append(self.track_cnt, 1)
                self.n_id += 1

        self.prev_img = cur_img.copy()
        self.prev_pts = cur_pts.copy()

        x_out = cur_pts[:, 0]
        y_out = cur_pts[:, 1]
        ids_out = self.ids.copy()
        cnt_out = self.track_cnt.copy()

        return x_out, y_out, ids_out, cnt_out


def draw_tracks(img, x, y, cnt):
    image_track = img.copy()
    if len(image_track.shape) == 2:
        image_track = cv2.cvtColor(image_track, cv2.COLOR_GRAY2BGR)
    for i, (x, y) in enumerate(zip(x, y)):
        ln = min(1.0, 1.0 * cnt[i] / 20)
        color = (255 * (1 - ln), 0, 255 * ln)
        image_track = cv2.circle(image_track, (int(x), int(y)), 2, color, 2)
    return image_track


def main():
    import sys
    import hdf5plugin
    import h5py

    path = "/run/media/tdemirdal/tb/m3ed/spot_indoor_building_loop/spot_indoor_building_loop_data.h5"
    h5 = h5py.File(path, "r")
    images = h5["/ovc/left/data"]

    tracker = KLTTracker()

    for i in range(len(images)):
        img = images[i][:, :, 0]
        print(f"Processing image {i + 1}/{len(images)}")

        # Track the image
        x, y, ids, cnt = tracker.track_image(img)
        img = draw_tracks(img, x, y, cnt)

        img_track = img.copy()

        cv2.imshow("Image", img)
        cv2.imshow("Image Track", img_track)
        cv2.waitKey(1)


if __name__ == "__main__":
    main()
