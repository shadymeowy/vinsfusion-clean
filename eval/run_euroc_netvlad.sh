# run helper script for VINS-Fusion evaluation
# This script sets up the environment and runs the evaluation sequences

# helper.sh path relative to this script
script_dir="$(dirname "$(readlink -f "$0")")"
helper_script="$script_dir/helper_euroc.sh"

echo "Running VINS-Fusion evaluation with helper script: $helper_script"
# config_path="/datasets/euroc/config/euroc_mono_imu_config_dbow2.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_dbow2_base_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_dbow2_base_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_dbow2_base_3/

# config_path="/datasets/euroc/config/euroc_mono_imu_config_netvlad.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_netvlad_base_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_netvlad_base_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_netvlad_base_3/

# config_path="/datasets/euroc/config/euroc_mono_imu_config_netvlad_2.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_netvlad_2_base_1/
# bash "$helper_script" "$config_path" /datasets/euroc/results_netvlad_2_base_2/
# bash "$helper_script" "$config_path" /datasets/euroc/results_netvlad_2_base_3/

# loop_netvlad-th_lightglue_50.yaml
# loop_dbow_lightglue_50.yaml
# loop_dbow_brief_50.yaml
# loop_netvlad_brief_50.yaml
# loop_netvlad_lightglue_50.yaml
# loop_netvlad-th_brief_50.yaml

# loop_netvlad-th_lightglue_75.yaml
# loop_dbow_lightglue_75.yaml
# loop_dbow_brief_75.yaml
# loop_netvlad_brief_75.yaml
# loop_netvlad_lightglue_75.yaml
# loop_netvlad-th_brief_75.yaml

# loop_netvlad-th_lightglue_25.yaml
# loop_dbow_lightglue_25.yaml
# loop_dbow_brief_25.yaml
# loop_netvlad_brief_25.yaml
# loop_netvlad_lightglue_25.yaml
# loop_netvlad-th_brief_25.yaml

# test_name="netvlad-th_lightglue_50"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

# test_name="netvlad-th_lightglue_50"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

# test_name="dbow_lightglue_50"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

# test_name="dbow_brief_50"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

# test_name="netvlad_brief_50"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

# test_name="netvlad_lightglue_50"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

# test_name="netvlad-th_brief_50"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

# test_name="netvlad-th2_brief_50"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

# test_name="netvlad-th_lightglue_75"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

# test_name="netvlad-th2_lightglue_75"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

# test_name="dbow_lightglue_75"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

# test_name="dbow_brief_75"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

# test_name="netvlad_brief_75"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

# test_name="netvlad_lightglue_75"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

# test_name="netvlad-th_brief_75"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1

# test_name="netvlad-th2_brief_75"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1

# test_name="netvlad-th_lightglue_25"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

# test_name="netvlad-th2_lightglue_25"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

# test_name="dbow_lightglue_25"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

# test_name="dbow_brief_25"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

# test_name="netvlad_brief_25"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

# test_name="netvlad_lightglue_25"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

# test_name="netvlad-th_brief_25"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

# test_name="netvlad-th2_brief_25"
# config_path="/datasets/euroc/config/loop_${test_name}.yaml"
# bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_1/

test_name="netvlad-th_lightglue_25"
config_path="/datasets/euroc/config/loop_${test_name}.yaml"
bash "$helper_script" "$config_path" /datasets/euroc/results_${test_name}_check_1/