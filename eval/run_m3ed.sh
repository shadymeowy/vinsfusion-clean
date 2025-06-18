# run helper script for VINS-Fusion evaluation
# This script sets up the environment and runs the evaluation sequences

# helper.sh path relative to this script
script_dir="$(dirname "$(readlink -f "$0")")"
helper_script="$script_dir/helper_m3ed.sh"

echo "Running VINS-Fusion evaluation with helper script: $helper_script"

# export TRACKER_MODE="tapnext_klt"
# export TRACKER_CACHE_PATH="/datasets/m3ed-bags/tapnext-klt_cache.shelve"

bash "$helper_script" /datasets/m3ed-bags/config/ /datasets/m3ed-bags/results_depthanything_1/
bash "$helper_script" /datasets/m3ed-bags/config/ /datasets/m3ed-bags/results_depthanything_2/
bash "$helper_script" /datasets/m3ed-bags/config/ /datasets/m3ed-bags/results_depthanything_3/

