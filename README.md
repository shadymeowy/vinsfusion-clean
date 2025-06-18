# vinsfusion-clean

This repository is a cleaned and containerized variant of VINS-Fusion built around ROS Noetic.
The intended workflow is:

1. Build the Docker image from [`docker/Dockerfile`](docker/Dockerfile).
2. Enter the container with [`docker/run.sh`](docker/run.sh).
3. Build and run the catkin workspace inside that container.

The `run.sh` script is the supported development/runtime entrypoint. It mounts:

- the persistent catkin workspace at `$HOME/.ws/vinsfusion` into `/root/catkin_ws`
- this repository into `/root/catkin_ws/src/VINS-Fusion`
- the dataset directory you pass as `$1` into `/datasets`

That means your build artifacts persist across container runs, while the source tree stays live-mounted from this checkout.

## Container Setup

Build the image from the repository root:

```bash
docker build -t ros:vins-fusion -f docker/Dockerfile .
```

Then start the container and mount your dataset directory:
This repository is a cleaned and containerized variant of VINS-Fusion built around ROS Noetic.
The intended workflow is:

1. Build the Docker image from [`docker/Dockerfile`](docker/Dockerfile).
2. Enter the container with [`docker/run.sh`](docker/run.sh).
3. Build and run the catkin workspace inside that container.

The `run.sh` script is the supported development/runtime entrypoint. It mounts:

- the persistent catkin workspace at `$HOME/.ws/vinsfusion` into `/root/catkin_ws`
- this repository into `/root/catkin_ws/src/VINS-Fusion`
- the dataset directory you pass as `$1` into `/datasets`

That means your build artifacts persist across container runs, while the source tree stays live-mounted from this checkout.

## Container Setup

Build the image from the repository root:

```bash
docker build -t ros:vins-fusion -f docker/Dockerfile .
```

Then start the container and mount your dataset directory:

```bash
./run.sh /absolute/path/to/datasets
```

Notes:

- Run `./run.sh` from the repository root. The script mounts `$(pwd)` into `/root/catkin_ws/src/VINS-Fusion`, so launching it from `docker/` or any other directory mounts the wrong path.
- `run.sh` expects the dataset directory as its first argument and mounts it at `/datasets`.
- The script enables X11 forwarding and GPU access, and assumes Docker can use `--runtime nvidia --gpus all`.
- `/dev/video0` and `/dev/dri` are passed through as well.

## Build Inside The Container

After `./run.sh` drops you into a shell inside the container:
./run.sh /absolute/path/to/datasets
```

Notes:

- Run `./run.sh` from the repository root. The script mounts `$(pwd)` into `/root/catkin_ws/src/VINS-Fusion`, so launching it from `docker/` or any other directory mounts the wrong path.
- `run.sh` expects the dataset directory as its first argument and mounts it at `/datasets`.
- The script enables X11 forwarding and GPU access, and assumes Docker can use `--runtime nvidia --gpus all`.
- `/dev/video0` and `/dev/dri` are passed through as well.

## Build Inside The Container

After `./run.sh` drops you into a shell inside the container:

```bash
cd /root/catkin_ws

cd /root/catkin_ws

catkin config \
    --env-cache \
    --extend /opt/ros/noetic \
    --cmake-args \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_BUILD_TYPE=Release

catkin build
source devel/setup.bash
```

This repo builds the following ROS executables:

- `vins_node`
- `loop_fusion_node`
- `global_fusion_node`

## Running MH_01 With `vins.launch`

The main launch file is `launch/vins.launch`. It starts:

- `vins_node`
- `loop_fusion_node`
- `rviz` by default
- `rosbag play` for the bag passed via `bag_path`

Use this workflow for the EuRoC `MH_01_easy.bag` example.

1. Download the dataset from [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/).
   Pick `MH_01` from the dataset list and make sure you have `MH_01_easy.bag` on the host.

2. Put the bag in a host directory, for example:

```bash
/home/your-user/euroc/MH_01_easy.bag
```

3. From the repository root, start the container and mount that directory as `/datasets`:

catkin build
source devel/setup.bash
```

This repo builds the following ROS executables:

- `vins_node`
- `loop_fusion_node`
- `global_fusion_node`

## Running MH_01 With `vins.launch`

The main launch file is `launch/vins.launch`. It starts:

- `vins_node`
- `loop_fusion_node`
- `rviz` by default
- `rosbag play` for the bag passed via `bag_path`

Use this workflow for the EuRoC `MH_01_easy.bag` example.

1. Download the dataset from [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/).
   Pick `MH_01` from the dataset list and make sure you have `MH_01_easy.bag` on the host.

2. Put the bag in a host directory, for example:

```bash
/home/your-user/euroc/MH_01_easy.bag
```

3. From the repository root, start the container and mount that directory as `/datasets`:

```bash
./run.sh /home/your-user/euroc
./run.sh /home/your-user/euroc
```

4. Inside the container, build once if needed:
4. Inside the container, build once if needed:

```bash
cd /root/catkin_ws

catkin config \
    --env-cache \
    --extend /opt/ros/noetic \
    --cmake-args \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_BUILD_TYPE=Release

catkin build
cd /root/catkin_ws

catkin config \
    --env-cache \
    --extend /opt/ros/noetic \
    --cmake-args \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_BUILD_TYPE=Release

catkin build
source devel/setup.bash
```

5. Create the output directories inside the container.

The EuRoC configs in this repository write trajectories into the mounted repository at `/root/catkin_ws/src/VINS-Fusion/output` and use `/root/catkin_ws/src/VINS-Fusion/output/pose_graph/` for loop-fusion pose-graph data.

Create the directories once:

```bash
cd /root/catkin_ws/src/VINS-Fusion
mkdir -p output/pose_graph
cd /root/catkin_ws/src/VINS-Fusion
mkdir -p output/pose_graph
```

6. Run `MH_01_easy.bag` with the mono+IMU EuRoC config:

6. Run `MH_01_easy.bag` with the mono+IMU EuRoC config:

```bash
cd /root/catkin_ws
source devel/setup.bash

roslaunch vins vins.launch \
    bag_path:=/datasets/MH_01_easy.bag \
    config_path:=/root/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml
cd /root/catkin_ws
source devel/setup.bash

roslaunch vins vins.launch \
    bag_path:=/datasets/MH_01_easy.bag \
    config_path:=/root/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml
```

7. After the run, trajectory files are written into the repository:

- `/root/catkin_ws/src/VINS-Fusion/output/vio.csv`
- `/root/catkin_ws/src/VINS-Fusion/output/vio_loop.csv`

`vio.csv` is written by `vins_node`. `vio_loop.csv` is written by `loop_fusion_node`.

These files are plain text trajectories with one pose per line:

```text
timestamp tx ty tz qx qy qz qw
```

That is TUM quaternion order.

You can evaluate them with [evo](https://github.com/MichaelGrupp/evo), for example:

```bash
evo_ape tum \
    /root/catkin_ws/src/VINS-Fusion/output/vio.csv \
    /root/catkin_ws/src/VINS-Fusion/output/vio_loop.csv -p
```

`evo` is not included in this Docker image, so install it separately on the host or in another environment if you want to analyze trajectories with it.

8. Optional launch arguments:

- `rviz:=false` to skip RViz
- `nowait:=true` to make the `rosbag play` node required

Example:

```bash
roslaunch vins vins.launch \
    bag_path:=/datasets/MH_01_easy.bag \
    config_path:=/root/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml \
    rviz:=false
```

To test other EuRoC modes, keep `bag_path` the same and change only `config_path`, for example:

```bash
/root/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml
/root/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_config.yaml
```

## Legacy Upstream README Starts Here

The remainder of this file is preserved from the original upstream-style README for reference. It contains older build/runtime guidance and does not reflect the container-first workflow above.

=======
>>>>>>> 1dd8534 (add uv and python description to readme)
# VINS-Fusion
## An optimization-based multi-sensor state estimator

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/vins_logo.png" width = 55% height = 55% div align=left />
<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/kitti.png" width = 34% height = 34% div align=center />

VINS-Fusion is an optimization-based multi-sensor state estimator, which achieves accurate self-localization for autonomous applications (drones, cars, and AR/VR). VINS-Fusion is an extension of [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono), which supports multiple visual-inertial sensor types (mono camera + IMU, stereo cameras + IMU, even stereo cameras only). We also show a toy example of fusing VINS with GPS. 
**Features:**
- multiple sensors support (stereo cameras / mono camera+IMU / stereo cameras+IMU)
- online spatial calibration (transformation between camera and IMU)
- online temporal calibration (time offset between camera and IMU)
- visual loop closure

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/kitti_rank.png" width = 80% height = 80% />

We are the **top** open-sourced stereo algorithm on [KITTI Odometry Benchmark](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) (12.Jan.2019).

**Authors:** [Tong Qin](http://www.qintonguav.com), Shaozu Cao, Jie Pan, [Peiliang Li](https://peiliangli.github.io/), and [Shaojie Shen](http://www.ece.ust.hk/ece.php/profile/facultydetail/eeshaojie) from the [Aerial Robotics Group](http://uav.ust.hk/), [HKUST](https://www.ust.hk/)

**Videos:**

<a href="https://www.youtube.com/embed/1qye82aW7nI" target="_blank"><img src="http://img.youtube.com/vi/1qye82aW7nI/0.jpg" 
alt="VINS" width="320" height="240" border="10" /></a>


**Related Paper:** (paper is not exactly same with code)

* **Online Temporal Calibration for Monocular Visual-Inertial Systems**, Tong Qin, Shaojie Shen, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS, 2018), **best student paper award** [pdf](https://ieeexplore.ieee.org/abstract/document/8593603)

* **VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator**, Tong Qin, Peiliang Li, Shaojie Shen, IEEE Transactions on Robotics [pdf](https://ieeexplore.ieee.org/document/8421746/?arnumber=8421746&source=authoralert) 


*If you use VINS-Fusion for your academic research, please cite our related papers. [bib](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/paper_bib.txt)*

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).


## 2. Build VINS-Fusion
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
(if you fail in this step, try to find another computer with clean system or reinstall Ubuntu and ROS)

## 3. EuRoC Example
Download [EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) to YOUR_DATASET_FOLDER. Take MH_01 for example, you can run VINS-Fusion with three sensor types (monocular camera + IMU, stereo cameras + IMU and stereo cameras). 
Open four terminals, run vins odometry, visual loop closure(optional), rviz and play the bag file respectively. 
Green path is VIO odometry; red path is odometry under visual loop closure.

### 3.1 Monocualr camera + IMU

```
    roslaunch vins vins_rviz.launch
    rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml 
    (optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml 
    rosbag play YOUR_DATASET_FOLDER/MH_01_easy.bag
```

### 3.2 Stereo cameras + IMU

```
    roslaunch vins vins_rviz.launch
    rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml 
    (optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml 
    rosbag play YOUR_DATASET_FOLDER/MH_01_easy.bag
```

### 3.3 Stereo cameras

```
    roslaunch vins vins_rviz.launch
    rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_config.yaml 
    (optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_config.yaml 
    rosbag play YOUR_DATASET_FOLDER/MH_01_easy.bag
```

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/euroc.gif" width = 430 height = 240 />


## 4. KITTI Example
### 4.1 KITTI Odometry (Stereo)
Download [KITTI Odometry dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) to YOUR_DATASET_FOLDER. Take sequences 00 for example,
Open two terminals, run vins and rviz respectively. 
(We evaluated odometry on KITTI benchmark without loop closure funtion)
```
    roslaunch vins vins_rviz.launch
    (optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml
    rosrun vins kitti_odom_test ~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml YOUR_DATASET_FOLDER/sequences/00/ 
```
### 4.2 KITTI GPS Fusion (Stereo + GPS)
Download [KITTI raw dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php) to YOUR_DATASET_FOLDER. Take [2011_10_03_drive_0027_synced](https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_drive_0027/2011_10_03_drive_0027_sync.zip) for example.
Open three terminals, run vins, global fusion and rviz respectively. 
Green path is VIO odometry; blue path is odometry under GPS global fusion.
```
    roslaunch vins vins_rviz.launch
    rosrun vins kitti_gps_test ~/catkin_ws/src/VINS-Fusion/config/kitti_raw/kitti_10_03_config.yaml YOUR_DATASET_FOLDER/2011_10_03_drive_0027_sync/ 
    rosrun global_fusion global_fusion_node
```

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/kitti.gif" width = 430 height = 240 />

## 5. VINS-Fusion on car demonstration
Download [car bag](https://drive.google.com/open?id=10t9H1u8pMGDOI6Q2w2uezEq5Ib-Z8tLz) to YOUR_DATASET_FOLDER.
Open four terminals, run vins odometry, visual loop closure(optional), rviz and play the bag file respectively. 
Green path is VIO odometry; red path is odometry under visual loop closure.
```
    roslaunch vins vins_rviz.launch
    rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/vi_car/vi_car.yaml 
    (optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/vi_car/vi_car.yaml 
    rosbag play YOUR_DATASET_FOLDER/car.bag
```

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/car_gif.gif" width = 430 height = 240  />


## 6. Run with your devices 
VIO is not only a software algorithm, it heavily relies on hardware quality. For beginners, we recommend you to run VIO with professional equipment, which contains global shutter cameras and hardware synchronization.

### 6.1 Configuration file
Write a config file for your device. You can take config files of EuRoC and KITTI as the example. 

### 6.2 Camera calibration
VINS-Fusion support several camera models (pinhole, mei, equidistant). You can use [camera model](https://github.com/hengli/camodocal) to calibrate your cameras. We put some example data under /camera_models/calibrationdata to tell you how to calibrate.
```
cd ~/catkin_ws/src/VINS-Fusion/camera_models/camera_calib_example/
rosrun camera_models Calibrations -w 12 -h 8 -s 80 -i calibrationdata --camera-model pinhole
```

## 7. Docker Support
To further facilitate the building process, we add docker in our code. Docker environment is like a sandbox, thus makes our code environment-independent. To run with docker, first make sure [ros](http://wiki.ros.org/ROS/Installation) and [docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/) are installed on your machine. Then add your account to `docker` group by `sudo usermod -aG docker $YOUR_USER_NAME`. **Relaunch the terminal or logout and re-login if you get `Permission denied` error**, type:
```
cd ~/catkin_ws/src/VINS-Fusion/docker
make build
```
Note that the docker building process may take a while depends on your network and machine. After VINS-Fusion successfully built, you can run vins estimator with script `run.sh`.
Script `run.sh` can take several flags and arguments. Flag `-k` means KITTI, `-l` represents loop fusion, and `-g` stands for global fusion. You can get the usage details by `./run.sh -h`. Here are some examples with this script:
```
# Euroc Monocualr camera + IMU
./run.sh ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml

# Euroc Stereo cameras + IMU with loop fusion
./run.sh -l ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml

# KITTI Odometry (Stereo)
./run.sh -k ~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml YOUR_DATASET_FOLDER/sequences/00/

# KITTI Odometry (Stereo) with loop fusion
./run.sh -kl ~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml YOUR_DATASET_FOLDER/sequences/00/

#  KITTI GPS Fusion (Stereo + GPS)
./run.sh -kg ~/catkin_ws/src/VINS-Fusion/config/kitti_raw/kitti_10_03_config.yaml YOUR_DATASET_FOLDER/2011_10_03_drive_0027_sync/

```
In Euroc cases, you need open another terminal and play your bag file. If you need modify the code, simply re-run `./run.sh` with proper auguments after your changes.


## 8. Acknowledgements
We use [ceres solver](http://ceres-solver.org/) for non-linear optimization and [DBoW2](https://github.com/dorian3d/DBoW2) for loop detection, a generic [camera model](https://github.com/hengli/camodocal) and [GeographicLib](https://geographiclib.sourceforge.io/).

## 9. License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues, please contact Tong Qin <qintonguavATgmail.com>.

For commercial inquiries, please contact Shaojie Shen <eeshaojieATust.hk>.
