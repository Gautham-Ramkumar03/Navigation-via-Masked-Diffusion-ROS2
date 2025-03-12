## Overview

This readme explain about the scripts to manage the setup, execution, training, and cleanup processes for the Nomad project. Below, you will find detailed instructions on how to use each script.

These scripts are located inside the `src` folder, which is inside the workspace directory. All scripts should be executed from the workspace directory, except for the `train.sh` script, which should be run from the `train` folder.

---

## `setup.sh`

### Purpose

The `setup.sh` script sets up the environment and necessary configurations for the Nomad project. It performs tasks such as creating configuration files, running colcon build, downloading model weights, cloning/updating submodules, and creating Conda environments.

### Usage

1. **Navigate to the workspace directory:**
   ```bash
   cd /path/to/workspace
   ```

2. **Run the setup script:**
   ```bash
   ./src/setup.sh
   ```

### Script Breakdown

- **Setup Paths:**
  Determines the directory of the script and sets the current working path.

- **Create Configuration Files:**
  Generates `path.yaml` and `navigate.yaml` files with project-specific configurations.

- **Run Colcon Build:**
  Compile the workspace.

- **Download Model Weights:**
  Downloads the `nomad.pth` from Google drive.

- **Clone/Update Submodule:**
  Clones or updates the `diffusion_policy` submodule as needed.

- **Create Conda Environment:**
  Creates Conda environments (`deploy_nomad` and `train_nomad`) based on YAML configuration files.

- **Install Packages in Environments:**
  Installs required packages in the created Conda environments.

---

## `run.sh`

### Purpose

The `run.sh` script is designed to collect new trajectories, create data from recorded bags, generate topomaps, and navigate with the trajectory.

### Usage

1. **Navigate to the workspace directory:**
   ```bash
   cd /path/to/workspace
   ```

2. **Run the script:**
   ```bash
   ./src/run.sh
   ```

### Script Breakdown

- **Bag Name Input:**
  Prompts the user to enter a name for the bag file.

- **Collect New Trajectory:**
  Prompts the user to start recording a new bag file and handle data collection.

- **Create Data:**
  Uses the recorded bag to create training data and generate topomaps.

- **Navigate:**
  Initiates the navigation process using the collected trajectory and created topomap.

### Key Commands and Parameters

- **Collect New Trajectory:**
  Records image and odometry data into a ROS bag file.
  ```bash
  ros2 bag record /cam0/image_raw /odom/local -o src/nomad/preprocessing/rosbags/$bag_name
  ```
  - `-o`: Output path for the bag file.
  - `/cam0/image_raw`: Topic for image data.
  - `/odom/local`: Topic for odometry data.
  - **Explanation:** This command records the data from the specified topics (`/cam0/image_raw` and `/odom/local`) and saves it in a ROS bag file named `$bag_name` in the specified output directory.

- **Create Data:**
  Processes the recorded bag to generate training data.
  ```bash
  python3 src/nomad/preprocessing/process_bag_diff.py -i src/nomad/preprocessing/rosbags/$bag_name -o src/nomad/preprocessing/training_data -n -1 -s 4.0 -c /cam0/image_raw -d /odom/local
  ```
  - `-i`: Input path to the recorded bag file.
  - `-o`: Output path for the training data.
  - `-n`: Number of data points to generate.
  - `-s`: Step size for sampling data.
  - `-c`: Image topic.
  - `-d`: Odom topic.
  - **Explanation:** This command processes the recorded bag file to generate training data, using the specified parameters such as the number of data points, step size, image topic, and odom topic.

  ```bash
  python3 src/nomad/preprocessing/pickle_data.py -f src/nomad/preprocessing/training_data/${bag_name}_0/traj_data.pkl -g
  ```
  - `-f`: Path to the trajectory data pickle file.
  - `-g`: Flag to plot the graph of positions (x, y) and yaw.
  - **Explanation:** This command plots the graph of positions and yaw of the processed data, ensuring it is collected without any error.

- **Create Topomap:**
  Generates a topological map of the environment from the images recorded in the bag file.
  ```bash
  ros2 run nomad create_topomap.py -b src/nomad/preprocessing/rosbags/$bag_name -T src/nomad/preprocessing/topomap -d $bag_name -i /cam0/image_raw -t 1.0 -w 1
  ```
  - `-b`: Path to the bag file.
  - `-T`: Path to the topomap directory.
  - `-d`: Directory name for the topomap images.
  - `-i`: Image topic.
  - `-t`: Time interval between images.
  - `-w`: Number of worker threads.
  - **Explanation:** This command creates a topological map from the images recorded in the specified bag file. The images are saved in the specified topomap directory, with a specified time interval between them. The number of worker threads used for processing is also specified.

- **Navigate:**
  Runs the PD controller and the navigation script to start navigating based on the created topomap.
  ```bash
  ros2 run nomad pd_controller.py
  ```
  - **Explanation:** This command starts the Proportional-Derivative (PD) controller which is responsible for driving the robot based on the waypoints generated.

  ```bash
  ros2 run nomad navigate.py --ros-args --params-file src/nomad/deploy/config/navigate.yaml --remap /img:=/cam0/image_raw
  ```
  - `--ros-args`: Passes ROS arguments.
  - `--params-file`: Path to the parameter file.
  - `--remap /img:=/cam0/image_raw`: Remaps the image topic.
  - **Explanation:** This command runs the navigation script, which uses the PD controller and the generated topomap to navigate through the environment. The image topic is remapped to ensure the correct image data is used during navigation.

---

## `train.sh`

### Purpose

The `train.sh` script is used to train the Nomad model. It allows for training from scratch or continuing training from a previous checkpoint.

### Usage

1. **Navigate to the train directory:**
   ```bash
   cd /path/to/workspace/src/nomad/train
   ```

2. **Run the train script:**
   ```bash
   ./train.sh
   ```

### Script Breakdown

- **Training Mode Prompt:**
  Prompts the user to choose whether to train from scratch or continue training from a previous checkpoint.

- **Configuration Adjustments:**
  Comments or uncomments the `load_run` line in the `model.yaml` configuration file based on the user's choice.

- **Activate Conda Environment:**
  Activates the `train_nomad` Conda environment.

- **Run Data Split and Training:**
  Executes the `data_split.py` and `train.py` scripts to split the data and train the model.

### Commands

- **Data Split:**
  ```bash
  python3 data_split.py
  ```

- **Training:**
  ```bash
  python3 train.py
  ```

### Parameters

- **train_from_scratch:** Determines whether to train from scratch or continue from a previous checkpoint.
- **model_weights_path:** Path to the model weights file.
- **model_config_path:** Path to the model configuration file.

---

## `cleanup.sh`

### Purpose

The `cleanup.sh` script is used to clean up the environment by removing collected data, training logs, and uninstallation of environments and dependencies. This will reset the repository to its initial state.

### Usage

1. **Navigate to the workspace directory:**
   ```bash
   cd /path/to/workspace
   ```

2. **Run the script:**
   ```bash
   ./src/cleanup.sh
   ```

### Options

1. **Delete Collected Data:**
   - Removes all collected data including rosbags, topomaps, and training data.

2. **Clear Training Logs:**
   - Clears all training logs, including wandb cache and data splits.

3. **Select What to Remove:**
   - Allows the user to select specific items to remove, such as pycache, deploy config files, diffusion policy, etc.

4. **Cleanup:**
   - Executes all predefined cleanup tasks.

5. **Uninstall:**
   - Removes everything related to the project from the system, including Conda environments and installed packages, ensuring the system is in a state similar to a freshly cloned repository.