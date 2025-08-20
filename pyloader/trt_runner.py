import os
import time

import cupy as cp
import cv2
import numpy as np
import tensorrt as trt
from cupy.cuda import runtime as cudart


def build_engine(
    onnx_model_path,
    engine_path,
):
    TRT_LOGGER = trt.Logger(trt.Logger.INFO)

    fp16_layer_names = ["vit_block", "ssm_block"]
    fp32_layer_names = ["rg_lru", "reduce", "temporal_pre_norm", "channel_pre_norm"]
    ignore_layer_names = ["Gather", "Cast"]

    builder = trt.Builder(TRT_LOGGER)
    network = builder.create_network(
        1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    )
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)

    config.set_flag(trt.BuilderFlag.FP16)  # Enable global FP16

    parser = trt.OnnxParser(network, TRT_LOGGER)
    if not os.path.exists(onnx_model_path):
        raise FileNotFoundError(f"ONNX model not found at {onnx_model_path}")

    print(f"Parsing ONNX model from {onnx_model_path}...")
    with open(onnx_model_path, "rb") as model:
        if not parser.parse(model.read()):
            print("ERROR: Failed to parse the ONNX file.")
            for error_idx in range(parser.num_errors):
                print(parser.get_error(error_idx))
            return None
    print("ONNX parsing complete.")

    def check_layer_names(layer_name, target_names):
        for name in target_names:
            if name.lower() in layer_name.lower():
                return True
        return False

    print("Applying mixed precision settings... 2")
    for i in range(network.num_layers):
        layer = network.get_layer(i)
        layer_name = layer.name

        if check_layer_names(layer_name, ignore_layer_names):
            print(f"    -> Layer '{layer_name}' ignored.")
            continue
        if (layer.precision not in [trt.DataType.FLOAT, trt.DataType.HALF]) or (
            any(
                layer.get_output_type(j) not in [trt.DataType.FLOAT, trt.DataType.HALF]
                for j in range(layer.num_outputs)
            )
        ):
            print(
                f"    -> Layer '{layer_name}' has unsupported precision: {layer.precision}. Skipping."
            )
            continue

        if check_layer_names(layer_name, fp16_layer_names) and not check_layer_names(
            layer_name, fp32_layer_names
        ):
            layer.precision = trt.DataType.HALF
            for j in range(layer.num_outputs):
                layer.set_output_type(j, trt.DataType.HALF)
            print(f"    -> Layer '{layer_name}' forced to FP16.")
        else:
            layer.precision = trt.DataType.FLOAT
            for j in range(layer.num_outputs):
                layer.set_output_type(j, trt.DataType.FLOAT)
            print(f"    -> Layer '{layer_name}' forced to FP32.")

    print("Building TensorRT engine...")
    serialized_engine = builder.build_serialized_network(network, config)
    if serialized_engine is None:
        print("ERROR: Failed to build serialized engine.")
        return None

    with open(engine_path, "wb") as f:
        f.write(serialized_engine)
    print(f"Engine saved to {engine_path}.")

    runtime = trt.Runtime(TRT_LOGGER)
    engine = runtime.deserialize_cuda_engine(serialized_engine)

    return engine


def load_engine(engine_path: str) -> trt.ICudaEngine:
    logger = trt.Logger(trt.Logger.WARNING)
    runtime = trt.Runtime(logger)
    with open(engine_path, "rb") as f:
        return runtime.deserialize_cuda_engine(f.read())


def allocate_device_buffers(engine: trt.ICudaEngine):
    """
    Allocate device buffers (as CuPy MemoryPointers) and host pinned buffers (as NumPy arrays)
    for all named I/O tensors in the engine. Returns (device_buffers, host_buffers, stream).
    """
    device_buffers = {}
    host_buffers = {}
    stream = cp.cuda.Stream()

    for i in range(engine.num_io_tensors):
        name = engine.get_tensor_name(i)
        shape = engine.get_tensor_shape(name)
        dtype = trt.nptype(engine.get_tensor_dtype(name))
        size = int(trt.volume(shape))
        nbytes = size * np.dtype(dtype).itemsize

        # allocate device memory
        dev_mem = cp.cuda.memory.alloc(nbytes)

        # allocate host-pinned memory only for our async I/O tensors
        host_buf = None
        if name in {"video", "step_in", "tracks", "visible_logits"}:
            pinned = cp.cuda.alloc_pinned_memory(nbytes)
            host_buf = np.frombuffer(pinned, dtype=dtype, count=size)

        device_buffers[name] = {
            "device": dev_mem,
            "ptr": int(dev_mem.ptr),
            "shape": shape,
            "dtype": dtype,
            "nbytes": nbytes,
            # keep pinned mem alive if any
            "pinned": pinned if host_buf is not None else None,
        }
        if host_buf is not None:
            host_buffers[name] = host_buf

    return device_buffers, host_buffers, stream


class TAPNextTRT:
    def __init__(
        self,
        onnx_model_path,
        engine_path,
        n_tracks=256,
        model_width=256,
        model_height=256,
    ):
        if not os.path.exists(engine_path):
            print(f"Building TensorRT engine from ONNX model: {onnx_model_path}")
            self.engine = build_engine(onnx_model_path, engine_path)
            if self.engine is None:
                raise RuntimeError("Failed to build TensorRT engine.")

        self.engine = load_engine(engine_path)
        self.context = self.engine.create_execution_context()
        self.device_buffers, self.host_buffers, self.stream = allocate_device_buffers(
            self.engine
        )
        self.raw_stream = int(self.stream.ptr)

        # bind all I/O addresses
        for name, buf in self.device_buffers.items():
            self.context.set_tensor_address(name, int(buf["device"].ptr))

        # model parameters
        self.n_tracks = n_tracks
        self.model_width = model_width
        self.model_height = model_height

        # zero out the static grid and state buffers
        self.reset()
        # warm up the model and capture the CUDA Graph
        self.warm_up()
        # zero out the static grid and state buffers again
        self.reset()

    def reset(self, query_points=None, width=None, height=None):
        self.step = 0

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
                sx, sy = width / self.model_width, height / self.model_height
                gx /= sx
                gy /= sy

        qp_np = (
            np.stack([np.zeros_like(gy), gy, gx], axis=-1)
            .reshape(1, -1, 3)
            .astype(np.float32)
        )
        qp_ptr = int(self.device_buffers["query_points_in"]["device"].ptr)
        self.stream.synchronize()
        cudart.memcpyAsync(
            qp_ptr,
            qp_np.ctypes.data,
            qp_np.nbytes,
            cudart.memcpyHostToDevice,
            self.stream.ptr,
        )
        cudart.memsetAsync(
            int(self.device_buffers["conv1d_states_in"]["device"].ptr),
            0,
            self.device_buffers["conv1d_states_in"]["nbytes"],
            self.stream.ptr,
        )
        cudart.memsetAsync(
            int(self.device_buffers["rg_lru_states_in"]["device"].ptr),
            0,
            self.device_buffers["rg_lru_states_in"]["nbytes"],
            self.stream.ptr,
        )
        self.stream.synchronize()

    def warm_up(self):
        # warm-up one frame
        rgb = (
            np.random.rand(self.model_height, self.model_width, 3).astype(np.float32)
            * 255.0
        )
        norm = (rgb / 255.0) * 2.0 - 1.0
        self.host_buffers["video"][:] = norm.ravel()
        self.host_buffers["step_in"][0] = 0
        cudart.memcpyAsync(
            int(self.device_buffers["video"]["device"].ptr),
            self.host_buffers["video"].ctypes.data,
            self.host_buffers["video"].nbytes,
            cudart.memcpyHostToDevice,
            self.stream.ptr,
        )
        cudart.memcpyAsync(
            int(self.device_buffers["step_in"]["device"].ptr),
            self.host_buffers["step_in"].ctypes.data,
            self.host_buffers["step_in"].nbytes,
            cudart.memcpyHostToDevice,
            self.stream.ptr,
        )
        self.context.execute_async_v3(self.raw_stream)
        self.stream.synchronize()

        # start implicit CUDA Graph capture
        self.stream.begin_capture()
        cudart.memcpyAsync(
            int(self.device_buffers["video"]["device"].ptr),
            self.host_buffers["video"].ctypes.data,
            self.host_buffers["video"].nbytes,
            cudart.memcpyHostToDevice,
            self.stream.ptr,
        )
        cudart.memcpyAsync(
            int(self.device_buffers["step_in"]["device"].ptr),
            self.host_buffers["step_in"].ctypes.data,
            self.host_buffers["step_in"].nbytes,
            cudart.memcpyHostToDevice,
            self.stream.ptr,
        )
        self.context.execute_async_v3(self.raw_stream)
        cudart.memcpyAsync(
            self.host_buffers["tracks"].ctypes.data,
            int(self.device_buffers["tracks"]["device"].ptr),
            self.host_buffers["tracks"].nbytes,
            cudart.memcpyDeviceToHost,
            self.stream.ptr,
        )
        cudart.memcpyAsync(
            self.host_buffers["visible_logits"].ctypes.data,
            int(self.device_buffers["visible_logits"]["device"].ptr),
            self.host_buffers["visible_logits"].nbytes,
            cudart.memcpyDeviceToHost,
            self.stream.ptr,
        )

        # finish capture
        self.graph = self.stream.end_capture()
        self.stream.synchronize()

    def run(self, frame: np.ndarray):
        # preprocess the input frame
        width, height = frame.shape[1], frame.shape[0]
        sx, sy = width / self.model_width, height / self.model_height
        rsz = cv2.resize(frame, (self.model_width, self.model_height))
        rgb = cv2.cvtColor(rsz, cv2.COLOR_BGR2RGB).astype(np.float32)
        norm = (rgb / 255.0) * 2.0 - 1.0

        # copy the preprocessed frame to the host buffer
        self.host_buffers["video"][:] = norm.ravel()
        self.host_buffers["step_in"][0] = self.step

        # replay the CUDA Graph
        self.stream.synchronize()
        self.graph.launch(stream=self.stream)
        # synchronize the stream to ensure all operations are complete
        self.stream.synchronize()

        # get the output tracks and visibility logits
        trk = self.host_buffers["tracks"].reshape(1, 1, self.n_tracks, 2)[0, 0]
        trk = trk * np.array([sy, sx], np.float32)
        vis = self.host_buffers["visible_logits"].reshape(1, 1, self.n_tracks)[0, 0] > 0

        # rotate the hidden states
        cudart.memcpyAsync(
            int(self.device_buffers["conv1d_states_in"]["device"].ptr),
            int(self.device_buffers["conv1d_states_out"]["device"].ptr),
            self.device_buffers["conv1d_states_in"]["nbytes"],
            cudart.memcpyDeviceToDevice,
            self.stream.ptr,
        )
        cudart.memcpyAsync(
            int(self.device_buffers["rg_lru_states_in"]["device"].ptr),
            int(self.device_buffers["rg_lru_states_out"]["device"].ptr),
            self.device_buffers["rg_lru_states_in"]["nbytes"],
            cudart.memcpyDeviceToDevice,
            self.stream.ptr,
        )
        self.stream.synchronize()

        # increment the step counter
        self.step += 1

        return trk, vis


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Test TensorRT video processing")
    parser.add_argument("video_path", type=str, help="Path to the input video file")
    parser.add_argument("onnx_path", type=str, help="Path to the ONNX model file")
    parser.add_argument(
        "engine_path", type=str, help="Path to the TensorRT engine file"
    )
    parser.add_argument(
        "--show", action="store_true", help="Display video output in a window"
    )
    args = parser.parse_args()

    # Initialize the TAPNextTRT model
    model = TAPNextTRT(args.onnx_path, args.engine_path)

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
            if cv2.waitKey(1) == ord("q"):
                break

        # report FPS
        t_now = time.perf_counter()
        print(f" FPS: {1 / (t_now - t_prev):.3f}")
        t_prev = t_now

    if args.show:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
