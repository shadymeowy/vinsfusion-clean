# run helper script for VINS-Fusion evaluation
# This script sets up the environment and runs the evaluation sequences

# helper.sh path relative to this script
script_dir="$(dirname "$(readlink -f "$0")")"
helper_script="$script_dir/helper_euroc.sh"

echo "Running VINS-Fusion evaluation with helper script: $helper_script"

export TRACKER_MODE="tapnext_klt"
# export TRACKER_CACHE_PATH="/datasets/m3ed-bags/tapnext-klt_cache.shelve"

# config_path="/datasets/euroc/config/euroc_mono_imu_config.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_base_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_base_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_base_3/

# config_path="/datasets/euroc/config/cauchy5_cauchy5.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy5-cauchy5_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy5-cauchy5_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy5-cauchy5_3/

# config_path="/datasets/euroc/config/huber5_huber5.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_huber5-huber5_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_huber5-huber5_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_huber5-huber5_3/

# config_path="/datasets/euroc/config/l2_cauchy5.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy5_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy5_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy5_3/

# config_path="/datasets/euroc/config/l2_huber5.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-huber5_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-huber5_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-huber5_3/

# config_path="/datasets/euroc/config/l2_cauchy.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy_3/

# config_path="/datasets/euroc/config/cauchy_cauchy.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy-cauchy_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy-cauchy_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy-cauchy_3/

export TRACKER_MODE="klt"
# export TRACKER_CACHE_PATH="/datasets/m3ed-bags/tapnext-klt_cache.shelve"

# config_path="/datasets/euroc/config/euroc_mono_imu_config.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_base_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_base_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_base_3/

# config_path="/datasets/euroc/config/cauchy5_cauchy5.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy5-cauchy5_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy5-cauchy5_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy5-cauchy5_3/

# config_path="/datasets/euroc/config/huber5_huber5.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_huber5-huber5_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_huber5-huber5_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_huber5-huber5_3/

# config_path="/datasets/euroc/config/l2_cauchy5.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy5_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy5_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy5_3/

# config_path="/datasets/euroc/config/l2_huber5.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-huber5_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-huber5_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-huber5_3/

# config_path="/datasets/euroc/config/l2_cauchy.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy_3/

# config_path="/datasets/euroc/config/cauchy_cauchy.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy-cauchy_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy-cauchy_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy-cauchy_3/

export TRACKER_MODE="tapnext_double"
# export TRACKER_CACHE_PATH="/datasets/m3ed-bags/tapnext-klt_cache.shelve"

# config_path="/datasets/euroc/config/euroc_mono_imu_config.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_base_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_base_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_base_3/

# config_path="/datasets/euroc/config/cauchy5_cauchy5.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy5-cauchy5_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy5-cauchy5_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy5-cauchy5_3/

# config_path="/datasets/euroc/config/huber5_huber5.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_huber5-huber5_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_huber5-huber5_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_huber5-huber5_3/

# config_path="/datasets/euroc/config/l2_cauchy5.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy5_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy5_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy5_3/

# config_path="/datasets/euroc/config/l2_huber5.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-huber5_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-huber5_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-huber5_3/

# config_path="/datasets/euroc/config/l2_cauchy.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy_3/

# config_path="/datasets/euroc/config/cauchy_cauchy.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy-cauchy_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy-cauchy_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy-cauchy_3/

export TRACKER_MODE="tapnext_klt_2"
# export TRACKER_CACHE_PATH="/datasets/m3ed-bags/tapnext-klt_cache.shelve"

# config_path="/datasets/euroc/config/euroc_mono_imu_config.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_base_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_base_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_base_3/

# config_path="/datasets/euroc/config/cauchy5_cauchy5.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy5-cauchy5_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy5-cauchy5_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy5-cauchy5_3/

# config_path="/datasets/euroc/config/huber5_huber5.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_huber5-huber5_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_huber5-huber5_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_huber5-huber5_3/

# config_path="/datasets/euroc/config/l2_cauchy5.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy5_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy5_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy5_3/

# config_path="/datasets/euroc/config/l2_huber5.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-huber5_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-huber5_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-huber5_3/

# config_path="/datasets/euroc/config/l2_cauchy.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_l2-cauchy_3/

# config_path="/datasets/euroc/config/cauchy_cauchy.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy-cauchy_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy-cauchy_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_cauchy-cauchy_3/

# export TRACKER_CACHE_PATH="/datasets/m3ed-bags/tapnext-klt_cache.shelve"

# config_path="/datasets/euroc/config/l2_cauchy.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy_3/

# export TRACKER_MODE="tapnext_klt_512"
# export num=14
# config_path="/datasets/euroc/config/l2_cauchy4.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy4_1/

export TRACKER_MODE="tapnext_klt_512"
export num=15
config_path="/datasets/euroc/config/l2_cauchy4.yaml"
bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy4_1/

export TRACKER_MODE="tapnext_klt_o_512"
export num=15
config_path="/datasets/euroc/config/l2_cauchy4.yaml"
bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy4_1/

# export TRACKER_MODE="tapnext_klt_256"
# export num=14
# config_path="/datasets/euroc/config/l2_cauchy4.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy4_1/

# export TRACKER_MODE="tapnext_klt_256"
# export num=12
# config_path="/datasets/euroc/config/l2_cauchy4.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy4_1/

# export TRACKER_MODE="tapnext_klt_o_128"
# export num=10
# config_path="/datasets/euroc/config/l2_cauchy4.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy4_1/

# export TRACKER_MODE="tapnext_klt_128"
# export num=10
# config_path="/datasets/euroc/config/l2_cauchy4.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy4_1/


export TRACKER_MODE="tapnext_klt_o_512"
export num=15
config_path="/datasets/euroc/config/l2_cauchy3.yaml"
bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy3_1/

export TRACKER_MODE="tapnext_klt_512"
export num=15
config_path="/datasets/euroc/config/l2_cauchy3.yaml"
bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy3_1/

# export TRACKER_MODE="tapnext_klt_o_256"
# export num=12
# config_path="/datasets/euroc/config/l2_cauchy3.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy3_1/

# export TRACKER_MODE="tapnext_klt_256"
# export num=12
# config_path="/datasets/euroc/config/l2_cauchy3.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy3_1/

# export TRACKER_MODE="tapnext_klt_o_128"
# export num=10
# config_path="/datasets/euroc/config/l2_cauchy3.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy3_1/

# export TRACKER_MODE="tapnext_klt_128"
# export num=10
# config_path="/datasets/euroc/config/l2_cauchy3.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy3_1/

# export TRACKER_MODE="aliked_256"
# export num=1
# config_path="/datasets/euroc/config/l2_cauchy4.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy4_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy4_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy4_3/

# export TRACKER_MODE="aliked_256"
# export num=1
# config_path="/datasets/euroc/config/l2_cauchy3.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy3_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy3_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy3_3/

# export TRACKER_MODE="aliked_klt_256"
# export num=1
# config_path="/datasets/euroc/config/l2_cauchy4.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy4_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy4_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy4_3/

# export TRACKER_MODE="aliked_klt_256"
# export num=1
# config_path="/datasets/euroc/config/l2_cauchy3.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy3_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy3_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy3_3/

# export TRACKER_MODE="aliked_nms_256"
# export num=1
# config_path="/datasets/euroc/config/l2_cauchy4.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy4_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy4_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy4_3/

# export TRACKER_MODE="aliked_nms_256"
# export num=1
# config_path="/datasets/euroc/config/l2_cauchy3.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy3_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy3_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy3_3/

# export TRACKER_MODE="aliked_klt_nms_256"
# export num=1
# config_path="/datasets/euroc/config/l2_cauchy4.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy4_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy4_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy4_3/

# export TRACKER_MODE="aliked_klt_nms_256"
# export num=1
# config_path="/datasets/euroc/config/l2_cauchy3.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy3_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy3_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_${TRACKER_MODE}_${num}_l2-cauchy3_3/