rviz=true
bags=/datasets/euroc/
config=$1
outputs=/datasets/euroc/output
results=$2
mkdir -p "$outputs"
mkdir -p "$results"

echo "Using config: $config"
ls "$config"
echo "Using results directory: $results"
echo "Starting VINS-Fusion evaluation..."
sleep 5

seq=MH_01_easy
# export VINS_TERMINATE_TIME=1403636762713555456
export VINS_TERMINATE_TIME=1403636761
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"
cp "$outputs/loop_debug.csv" "$results/$seq/loop_debug.csv"
sleep 5

seq=MH_02_easy
# export VINS_TERMINATE_TIME=1403637009451666432
export VINS_TERMINATE_TIME=1403637008
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"
cp "$outputs/loop_debug.csv" "$results/$seq/loop_debug.csv"
sleep 5

seq=MH_03_medium
# export VINS_TERMINATE_TIME=1403637264388318976
export VINS_TERMINATE_TIME=1403637262
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"
cp "$outputs/loop_debug.csv" "$results/$seq/loop_debug.csv"
sleep 5

seq=MH_04_difficult
# export VINS_TERMINATE_TIME=1403638227695097088
export VINS_TERMINATE_TIME=1403638226
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"
cp "$outputs/loop_debug.csv" "$results/$seq/loop_debug.csv"
sleep 5

seq=MH_05_difficult
# export VINS_TERMINATE_TIME=1403638630527829504
export VINS_TERMINATE_TIME=1403638628
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"
cp "$outputs/loop_debug.csv" "$results/$seq/loop_debug.csv"
sleep 5

seq=V1_01_easy
# export VINS_TERMINATE_TIME=1403715417812143104
export VINS_TERMINATE_TIME=1403715415
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"
cp "$outputs/loop_debug.csv" "$results/$seq/loop_debug.csv"
sleep 5

seq=V1_02_medium
# export VINS_TERMINATE_TIME=1403715608412143104
export VINS_TERMINATE_TIME=1403715607
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"
cp "$outputs/loop_debug.csv" "$results/$seq/loop_debug.csv"
sleep 5
    
seq=V1_03_difficult
# export VINS_TERMINATE_TIME=1403715993034057984
export VINS_TERMINATE_TIME=1403715991
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"
cp "$outputs/loop_debug.csv" "$results/$seq/loop_debug.csv"
sleep 5

seq=V2_01_easy
# export VINS_TERMINATE_TIME=1413393325455760384
export VINS_TERMINATE_TIME=1413393324
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"
cp "$outputs/loop_debug.csv" "$results/$seq/loop_debug.csv"
sleep 5

seq=V2_02_medium
# export VINS_TERMINATE_TIME=1413394002655760384
export VINS_TERMINATE_TIME=1413394001
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"
cp "$outputs/loop_debug.csv" "$results/$seq/loop_debug.csv"
sleep 5

seq=V2_03_difficult
# export VINS_TERMINATE_TIME=1413394997605760512
export VINS_TERMINATE_TIME=1413394996
roslaunch vins vins.launch bag_path:="$bags/$seq.bag" config_path:="$config" rviz:="$rviz"
mkdir -p "$results/$seq"
cp "$outputs/feature_debug.csv" "$results/$seq/feature_debug.csv"
cp "$outputs/vio.csv" "$results/$seq/vio.csv"
cp "$outputs/vio_loop.csv" "$results/$seq/vio_loop.csv"
cp "$outputs/loop_debug.csv" "$results/$seq/loop_debug.csv"
sleep 5
