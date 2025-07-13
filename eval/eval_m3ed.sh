rviz=false
bags=/datasets/m3ed-bags/rosbag/
config=/datasets/m3ed-bags/config/m3ed_mono_imu_config.yaml
outputs=/datasets/m3ed-bags/output/
results=/datasets/m3ed-bags/results/
mkdir -p "$outputs"
mkdir -p "$results"

seq=spot_indoor_building_loop
export VINS_TERMINATE_TIME=101
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

# seq=spot_forest_easy_1
# export VINS_TERMINATE_TIME=74
# roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
# mkdir -p "$results/$seq"
# cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
# cp "$outputs/vio.csv" "$results/$seq/vio.csv"
# cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

# seq=spot_forest_easy_2
# export VINS_TERMINATE_TIME=116
# roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
# mkdir -p "$results/$seq"
# cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
# cp "$outputs/vio.csv" "$results/$seq/vio.csv"
# cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

seq=spot_forest_hard
export VINS_TERMINATE_TIME=104
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

seq=spot_forest_road_1
export VINS_TERMINATE_TIME=178
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

# seq=spot_forest_road_3
# export VINS_TERMINATE_TIME=199
# roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
# mkdir -p "$results/$seq"
# cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
# cp "$outputs/vio.csv" "$results/$seq/vio.csv"
# cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

# seq=spot_indoor_obstacles
# export VINS_TERMINATE_TIME=84
# roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
# mkdir -p "$results/$seq"
# cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
# cp "$outputs/vio.csv" "$results/$seq/vio.csv"
# cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

seq=spot_indoor_stairs
export VINS_TERMINATE_TIME=100
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

seq=spot_indoor_stairwell
export VINS_TERMINATE_TIME=98
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

# seq=spot_outdoor_day_art_plaza_loop
# export VINS_TERMINATE_TIME=144
# roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
# mkdir -p "$results/$seq"
# cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
# cp "$outputs/vio.csv" "$results/$seq/vio.csv"
# cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

seq=spot_outdoor_day_penno_short_loop
export VINS_TERMINATE_TIME=117
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

# seq=spot_outdoor_day_rocky_steps
# export VINS_TERMINATE_TIME=153
# roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
# mkdir -p "$results/$seq"
# cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
# cp "$outputs/vio.csv" "$results/$seq/vio.csv"
# cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

seq=spot_outdoor_day_skatepark_1
export VINS_TERMINATE_TIME=92
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

# seq=spot_outdoor_day_skatepark_2
# export VINS_TERMINATE_TIME=65
# roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
# mkdir -p "$results/$seq"
# cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
# cp "$outputs/vio.csv" "$results/$seq/vio.csv"
# cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

# seq=spot_outdoor_day_srt_green_loop
# export VINS_TERMINATE_TIME=63
# roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
# mkdir -p "$results/$seq"
# cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
# cp "$outputs/vio.csv" "$results/$seq/vio.csv"
# cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

# seq=spot_outdoor_day_srt_under_bridge_1
# export VINS_TERMINATE_TIME=203
# roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
# mkdir -p "$results/$seq"
# cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
# cp "$outputs/vio.csv" "$results/$seq/vio.csv"
# cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

# seq=spot_outdoor_day_srt_under_bridge_2
# export VINS_TERMINATE_TIME=181
# roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
# mkdir -p "$results/$seq"
# cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
# cp "$outputs/vio.csv" "$results/$seq/vio.csv"
# cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

# seq=spot_outdoor_night_penno_plaza_lights
# export VINS_TERMINATE_TIME=75
# roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
# mkdir -p "$results/$seq"
# cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
# cp "$outputs/vio.csv" "$results/$seq/vio.csv"
# cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

seq=spot_outdoor_night_penno_short_loop
export VINS_TERMINATE_TIME=132
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"