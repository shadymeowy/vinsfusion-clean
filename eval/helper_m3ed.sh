rviz=true
bags=/datasets/m3ed-bags/rosbag/
config=$1
outputs=/datasets/m3ed-bags/output/
results=$2
mkdir -p "$outputs"
mkdir -p "$results"

echo "Using config: $config"
ls "$config"
echo "Using results directory: $results"
echo "Starting VINS-Fusion evaluation..."
sleep 5

seq=spot_indoor_building_loop
export VINS_TERMINATE_TIME=101
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config/$seq/config.yaml" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"
sleep 5

seq=spot_forest_hard
export VINS_TERMINATE_TIME=103
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config/$seq/config.yaml" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"
sleep 5

seq=spot_forest_road_1
export VINS_TERMINATE_TIME=177
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config/$seq/config.yaml" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"
sleep 5

seq=spot_indoor_stairs
export VINS_TERMINATE_TIME=99
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config/$seq/config.yaml" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"
sleep 5

seq=spot_indoor_stairwell
export VINS_TERMINATE_TIME=97
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config/$seq/config.yaml" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"
sleep 5

seq=spot_outdoor_day_penno_short_loop
export VINS_TERMINATE_TIME=116
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config/$seq/config.yaml" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"
sleep 5

seq=spot_outdoor_day_skatepark_1
export VINS_TERMINATE_TIME=91
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config/$seq/config.yaml" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"
sleep 5

seq=spot_outdoor_night_penno_short_loop
export VINS_TERMINATE_TIME=131
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config/$seq/config.yaml" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"
sleep 5
