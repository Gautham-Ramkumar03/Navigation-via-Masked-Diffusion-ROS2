#!/usr/bin/env bash

# Function to prompt for y/n and set the variable accordingly
prompt_for_flag() {
    local prompt_message=$1
    local var_name=$2
    while true; do
        read -p "$prompt_message (y/n): " yn
        case $yn in
            [Yy]* ) eval $var_name=true; break;;
            [Nn]* ) eval $var_name=false; break;;
            * ) echo "Please answer y or n.";;
        esac
    done
}

# Get the directory of the script
setup_paths() {
    SCRIPT_DIR=$(dirname "$0")
    cd "$SCRIPT_DIR"
    current_path=$(pwd)
}

# Create configuration files
create_config_files() {
    local output_path="$current_path/nomad/train/config/path.yaml"
    local dataset_content="datasets:
  training_data:
    data_folder: $current_path/nomad/preprocessing/training_data/
    train: $current_path/nomad/preprocessing/data_splits/training_data/train/
    test: $current_path/nomad/preprocessing/data_splits/training_data/test/"

    mkdir -p "$(dirname "$output_path")"
    echo "$dataset_content" > "$output_path"
    echo "Path file created at $output_path"

    local navigate_output_path="$current_path/nomad/deploy/config/navigate.yaml"
    local navigate_content="nomad_navigator:
  ros__parameters:
    model_name: \"nomad\"
    model_weights_path: \"$current_path/nomad/deploy/model_weights/nomad.pth\"
    model_config_path: \"$current_path/nomad/train/config/model.yaml\"
    topomap_images_dir: \"$current_path/nomad/preprocessing/topomap\"
    topomap_dir: \"$current_path/nomad/preprocessing/topomap/bag_name\"
    waypoint: 2
    goal_node: -20
    close_threshold: 3
    radius: 15
    num_samples: 8
    v_max: 0.2
    w_max: 0.2
    hz: 4.0
    graph_hz: 0.33"

    mkdir -p "$(dirname "$navigate_output_path")"
    echo "$navigate_content" > "$navigate_output_path"
    echo "Navigate file created at $navigate_output_path"
}

# Run colcon build if needed
run_colcon_build() {
    tmux new-session -d -s colcon_build -c "$current_path/.." "
        source ~/miniconda3/etc/profile.d/conda.sh
        conda deactivate
        colcon build --symlink-install
        echo 'Colcon build completed.'
        sleep 3
        tmux kill-session -t colcon_build
    "
    tmux attach -t colcon_build
}

# Download nomad.pth if needed
download_nomad_pth() {
    FILE_ID="1YJhkkMJAYOiKNyCaelbS_alpUpAJsOUb"
    FILE_NAME="nomad.pth"
    DOWNLOAD_DIR="$current_path/nomad/deploy/model_weights"

    mkdir -p "${DOWNLOAD_DIR}"
    pip install gdown
    gdown ${FILE_ID} -O "${DOWNLOAD_DIR}/${FILE_NAME}"
    echo "File downloaded as ${DOWNLOAD_DIR}/${FILE_NAME}"
    pip uninstall gdown -y
}

# Clone/update the diffusion_policy submodule if needed
clone_or_update_submodule() {
    if [ -f "$current_path/.gitmodules" ] && grep -q "diffusion_policy" "$current_path/.gitmodules"; then
        git submodule update --init --recursive
        echo 'Submodule updated'
    else
        [ -d "$current_path/nomad/diffusion_policy" ] && rm -rf "$current_path/nomad/diffusion_policy"
        git clone https://github.com/real-stanford/diffusion_policy.git "$current_path/nomad/diffusion_policy"
        echo 'Cloned from github'
    fi
}

# Create Conda environment if needed
create_conda_environment() {
    local env=$1
    local yaml_file=$2

    if conda env list | grep -q "^$env\s"; then
        prompt_for_flag "$env already exists. Do you want to replace it?" REPLACE_ENV
        if [ "$REPLACE_ENV" = true ]; then
            conda remove --name "$env" --all
            conda env create -f "$yaml_file"
            echo "Conda environment $env created."
        else
            echo "Skipping creation of $env environment."
        fi
    else
        conda env create -f "$yaml_file"
        echo "Conda environment $env created."
    fi
}

# Install packages in specified environments
install_packages_in_envs() {
    SESSION_NAME="install_pkg"

    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        echo "tmux session $SESSION_NAME already exists"
    else
        tmux new-session -d -s "$SESSION_NAME"
        env_list=("train_nomad" "deploy_nomad")

        for env in "${env_list[@]}"; do
            if conda env list | grep -q "^$env\s"; then
                tmux send-keys -t "$SESSION_NAME" "conda activate $env" C-m
                tmux send-keys -t "$SESSION_NAME" "pip install -e $current_path/nomad/diffusion_policy" C-m
                tmux send-keys -t "$SESSION_NAME" "pip install -e $current_path/nomad/train" C-m
                tmux send-keys -t "$SESSION_NAME" "conda deactivate" C-m
            else
                echo "Conda environment $env does not exist. Skipping package installation for $env."
            fi
        done

        tmux send-keys -t "$SESSION_NAME" "sleep 3; tmux kill-session -t $SESSION_NAME" C-m
        tmux attach -t "$SESSION_NAME"
    fi
}

# Main script execution for installation
installation() {
    local create_deploy_env=$1
    local create_train_env=$2

    setup_paths
    create_config_files

    RUN_COLCON_BUILD=true
    run_colcon_build

    DOWNLOAD_NOMAD_PTH=true
    download_nomad_pth

    CLONE_UPDATE_SUBMODULE=true
    clone_or_update_submodule

    if [ "$create_deploy_env" = true ]; then
        create_conda_environment "deploy_nomad" "$current_path/nomad/deploy/deploy_nomad.yaml"
    fi

    if [ "$create_train_env" = true ]; then
        create_conda_environment "train_nomad" "$current_path/nomad/train/train_nomad.yaml"
    fi

    install_packages_in_envs
}

# Main script execution for selective installation
selective_installation() {
    setup_paths
    create_config_files

    prompt_for_flag "Do you want to run colcon build?" RUN_COLCON_BUILD
    if [ "$RUN_COLCON_BUILD" = true ]; then
        run_colcon_build
    fi

    prompt_for_flag "Do you want to download nomad.pth?" DOWNLOAD_NOMAD_PTH
    if [ "$DOWNLOAD_NOMAD_PTH" = true ]; then
        download_nomad_pth
    fi

    prompt_for_flag "Do you want to clone/update the diffusion policy submodule?" CLONE_UPDATE_SUBMODULE
    if [ "$CLONE_UPDATE_SUBMODULE" = true ]; then
        clone_or_update_submodule
    fi

    prompt_for_flag "Do you want to create the deployment environment?" CREATE_DEPLOY_ENV
    if [ "$CREATE_DEPLOY_ENV" = true ]; then
        create_conda_environment "deploy_nomad" "$current_path/nomad/deploy/deploy_nomad.yaml"
    fi

    prompt_for_flag "Do you want to create the training environment?" CREATE_TRAIN_ENV
    if [ "$CREATE_TRAIN_ENV" = true ]; then
        create_conda_environment "train_nomad" "$current_path/nomad/train/train_nomad.yaml"
    fi

    prompt_for_flag "Do you want to install packages in environments?" INSTALL_PACKAGES_IN_ENVS
    if [ "$INSTALL_PACKAGES_IN_ENVS" = true ]; then
        install_packages_in_envs
    fi

}

# Main menu function
main_menu() {
    while true; do
    	clear
        echo "Choose an option:"
        echo "1. Full installation"
        echo "2. Installation for deployment"
        echo "3. Installation for training"
        echo "4. Installation without creating conda env"
        echo "5. Select what to install"
        echo "0. Exit"

        read -p "Enter your choice (0-5): " choice
        echo    # move to a new line

        case $choice in
            1)
                installation true true
                break
                ;;
            2)
                installation true false
                break
                ;;
            3)
                installation false true
                break
                ;;
            4)
                installation false false
                break
                ;;
            5)
                selective_installation
                break
                ;;
            0)
                echo "Exiting."
                sleep 1
                clear
                exit 0
                ;;
            *)
                echo "Invalid choice. Please select a valid option."
                sleep 2
                ;;
        esac
    done
}

# Start the main menu
main_menu

echo "Installation completed."
