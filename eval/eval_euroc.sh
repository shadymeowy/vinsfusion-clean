rviz=true
bags=/datasets/euroc/
config=/root/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml
outputs=/datasets/euroc/output/
results=/datasets/euroc/results/
mkdir -p "$outputs"
mkdir -p "$results"

# MH_01_easy.txt*
# MH_02_easy.txt*
# MH_03_medium.txt*
# MH_04_difficult.txt*
# MH_05_difficult.txt*
# V1_01_easy.txt*
# V1_02_medium.txt*
# V1_03_difficult.txt*
# V2_01_easy.txt*
# V2_02_medium.txt*
# V2_03_difficult.txt*

seq=MH_01_easy
# export VINS_TERMINATE_TIME=1403636762713555456
export VINS_TERMINATE_TIME=1403636762
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

# seq=MH_02_easy
# export VINS_TERMINATE_TIME=1403637009451666432
export VINS_TERMINATE_TIME=1403637009
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

seq=MH_03_medium
# export VINS_TERMINATE_TIME=1403637264388318976
export VINS_TERMINATE_TIME=1403637264
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

seq=MH_04_difficult
# export VINS_TERMINATE_TIME=1403638227695097088
export VINS_TERMINATE_TIME=1403638227
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

seq=MH_05_difficult
# export VINS_TERMINATE_TIME=1403638630527829504
export VINS_TERMINATE_TIME=1403638629
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

seq=V1_01_easy
# export VINS_TERMINATE_TIME=1403715417812143104
export VINS_TERMINATE_TIME=1403715417
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

seq=V1_02_medium
# export VINS_TERMINATE_TIME=1403715608412143104
export VINS_TERMINATE_TIME=1403715608
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"
    
seq=V1_03_difficult
# export VINS_TERMINATE_TIME=1403715993034057984
export VINS_TERMINATE_TIME=1403715992
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

seq=V2_01_easy
# export VINS_TERMINATE_TIME=1413393325455760384
export VINS_TERMINATE_TIME=1413393325
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

seq=V2_02_medium
# export VINS_TERMINATE_TIME=1413394002655760384
export VINS_TERMINATE_TIME=1413394002
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"

seq=V2_03_difficult
# export VINS_TERMINATE_TIME=1413394997605760512
export VINS_TERMINATE_TIME=1413394997
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"