import time

import cv2
import numpy as np
import onnxruntime as ort
import torch


class TAPNextONNX:
    def __init__(
        self,
        onnx_model_path,
        n_tracks=256,
        model_width=256,
        model_height=256,
        providers=["CUDAExecutionProvider", "CPUExecutionProvider"],
    ):
        self.session = ort.InferenceSession(
            onnx_model_path,
            providers=providers,
        )

        # model parameters
        self.n_tracks = n_tracks
        self.model_width = model_width
        self.model_height = model_height

        # determine device
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device_id = 0 if self.device == "cuda" else None

        # allocate buffers
        self.step = torch.zeros(tuple(), dtype=torch.int64, device=self.device)
        self.query_points = torch.zeros(
            (1, n_tracks, 3), dtype=torch.float32, device=self.device
        )
        self.video = torch.zeros(
            (1, 1, model_height, model_width, 3),
            dtype=torch.float32,
            device=self.device,
        )
        self.tracks = torch.zeros(
            (1, 1, n_tracks, 2), dtype=torch.float32, device=self.device
        )
        self.visible_logits = torch.zeros(
            (1, 1, n_tracks, 1), dtype=torch.float32, device=self.device
        )
        self.conv1d_states_in = torch.zeros(
            (12, 1024 + n_tracks, 3, 768), dtype=torch.float32, device=self.device
        )
        self.conv1d_states_out = torch.zeros_like(self.conv1d_states_in)
        self.rg_lru_states_in = torch.zeros(
            (12, 1024 + n_tracks, 768), dtype=torch.float32, device=self.device
        )
        self.rg_lru_states_out = torch.zeros_like(self.rg_lru_states_in)

        # bind buffers
        io = self.session.io_binding()
        io.bind_input(
            name="video",
            device_type=self.device,
            device_id=self.device_id,
            element_type=np.float32,
            shape=self.video.shape,
            buffer_ptr=self.video.data_ptr(),
        )
        io.bind_input(
            name="step_in",
            device_type=self.device,
            device_id=self.device_id,
            element_type=np.int64,
            shape=self.step.shape,
            buffer_ptr=self.step.data_ptr(),
        )
        io.bind_input(
            name="query_points_in",
            device_type=self.device,
            device_id=self.device_id,
            element_type=np.float32,
            shape=self.query_points.shape,
            buffer_ptr=self.query_points.data_ptr(),
        )
        io.bind_output(
            name="tracks",
            device_type=self.device,
            device_id=self.device_id,
            element_type=np.float32,
            shape=self.tracks.shape,
            buffer_ptr=self.tracks.data_ptr(),
        )
        io.bind_output(
            name="visible_logits",
            device_type=self.device,
            device_id=self.device_id,
            element_type=np.float32,
            shape=self.visible_logits.shape,
            buffer_ptr=self.visible_logits.data_ptr(),
        )
        io.bind_input(
            name="conv1d_states_in",
            device_type=self.device,
            device_id=self.device_id,
            element_type=np.float32,
            shape=self.conv1d_states_in.shape,
            buffer_ptr=self.conv1d_states_in.data_ptr(),
        )
        io.bind_output(
            name="conv1d_states_out",
            device_type=self.device,
            device_id=self.device_id,
            element_type=np.float32,
            shape=self.conv1d_states_out.shape,
            buffer_ptr=self.conv1d_states_out.data_ptr(),
        )
        io.bind_input(
            name="rg_lru_states_in",
            device_type=self.device,
            device_id=self.device_id,
            element_type=np.float32,
            shape=self.rg_lru_states_in.shape,
            buffer_ptr=self.rg_lru_states_in.data_ptr(),
        )
        io.bind_output(
            name="rg_lru_states_out",
            device_type=self.device,
            device_id=self.device_id,
            element_type=np.float32,
            shape=self.rg_lru_states_out.shape,
            buffer_ptr=self.rg_lru_states_out.data_ptr(),
        )
        self.io = io

    def reset(self, query_points=None, width=None, height=None):
        if query_points is None:
            grid_size = int(np.sqrt(self.n_tracks))

            space_width = 256.0 / (grid_size + 1)
            space_height = 256.0 / (grid_size + 1)
            gx, gy = np.meshgrid(
                np.linspace(space_width, self.model_width - space_width, grid_size),
                np.linspace(space_height, self.model_height - space_height, grid_size),
                indexing="ij",
            )
        else:
            query_points = query_points.copy()
            gx, gy = query_points[:, 1], query_points[:, 0]
            gx = gx.reshape(-1, 1)
            gy = gy.reshape(-1, 1)

            if width is not None and height is not None:
                # naive resizing
                sx, sy = width / self.model_width, height / self.model_height
                gx /= sx
                gy /= sy
                # with padding, keeps aspect ratio
                # scale = max(width / self.model_width, height / self.model_height)
                # sx, sy = scale, scale
                # gx /= sx
                # gy /= sy

        # set the query points
        self.step[...] = 0
        gt = np.zeros_like(gx.flatten())
        query_points = np.stack([gt, gy.flatten(), gx.flatten()], axis=-1)
        self.query_points[0, :, :].copy_(torch.from_numpy(query_points).float())

        # reset the buffers
        self.tracks.zero_()
        self.visible_logits.zero_()
        self.conv1d_states_in.zero_()
        self.conv1d_states_out.zero_()
        self.rg_lru_states_in.zero_()
        self.rg_lru_states_out.zero_()

    def run(self, frame: np.ndarray):
        # preprocess the input frame
        width, height = frame.shape[1], frame.shape[0]

        # naive resizing
        sx, sy = width / self.model_width, height / self.model_height
        rsz = cv2.resize(frame, (self.model_width, self.model_height))
        # with padding, keeps aspect ratio
        # scale = max(width / self.model_width, height / self.model_height)
        # new_width = int(width / scale)
        # new_height = int(height / scale)
        # sx, sy = scale, scale
        # rsz = cv2.resize(
        #     frame,
        #     (new_width, new_height),
        #     interpolation=cv2.INTER_LINEAR,
        # )
        # # pad to model size
        # rsz = cv2.copyMakeBorder(
        #     rsz,
        #     0,
        #     self.model_height - rsz.shape[0],
        #     0,
        #     self.model_width - rsz.shape[1],
        #     cv2.BORDER_CONSTANT,
        #     value=(0, 0, 0),
        # )

        rgb = cv2.cvtColor(rsz, cv2.COLOR_BGR2RGB).astype(np.float32)
        norm = (rgb / 255.0) * 2.0 - 1.0

        # copy the input
        self.video[0, :, :, :].copy_(torch.from_numpy(norm))

        # run the model
        self.session.run_with_iobinding(self.io)
        self.io.synchronize_outputs()

        # increment the step
        self.step[...] += 1

        # copy states
        self.conv1d_states_in[:] = self.conv1d_states_out
        self.rg_lru_states_in[:] = self.rg_lru_states_out

        # postprocess the outputs
        tracks = self.tracks[0, 0, :, :2].cpu().numpy()
        visibility = self.visible_logits[0, 0, :].cpu().numpy().flatten() > 0
        tracks[:, 0] *= sy
        tracks[:, 1] *= sx

        return tracks, visibility


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Test TensorRT video processing")
    parser.add_argument("video_path", type=str, help="Path to the input video file")
    parser.add_argument("onnx_path", type=str, help="Path to the ONNX model file")
    parser.add_argument(
        "--show", action="store_true", help="Display video output in a window"
    )
    args = parser.parse_args()

    # Open the video file
    cap = cv2.VideoCapture(args.video_path)
    if not cap.isOpened():
        raise ValueError(f"Could not open video file {args.video_path}")

    # Get video properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Video resolution: {width}x{height}")

    frames = []
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frames.append(frame)
    cap.release()

    if not frames:
        raise ValueError("No frames read from the video file.")

    # Initialize the TAPNextTRT model
    model = TAPNextONNX(args.onnx_path)

    # Generate query points from the first frame
    model.reset(width=width, height=height)

    t_prev = time.perf_counter()

    for i, frame in enumerate(frames):
        print(f"Frame {i}")
        # Run model
        tracks, visibility = model.run(frame)

        # Draw tracks on the frame
        if args.show:
            for (y, x), ok in zip(tracks, visibility):
                if ok and (0 <= y < height) and (0 <= x < width):
                    cv2.circle(frame, (int(x), int(y)), 3, (0, 255, 0), -1)

            cv2.imshow("Output", frame)
            if cv2.waitKey(0) == ord("q"):
                break

        # report FPS
        t_now = time.perf_counter()
        print(f" FPS: {1 / (t_now - t_prev):.3f}")
        t_prev = t_now

    if args.show:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
