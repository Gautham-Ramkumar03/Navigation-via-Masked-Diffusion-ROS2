# NoMaD: Navigation via Masked Diffusion

![ROS2](https://img.shields.io/badge/ROS2-Humble-brightgreen)
![Python](https://img.shields.io/badge/Python-3.8+-blue)
![License](https://img.shields.io/badge/License-Proprietary-red)

## Overview

NoMaD (Navigation via Masked Diffusion) is an end-to-end visual navigation framework that leverages masked diffusion policies for robust navigation and exploration. Building upon the General Navigation Models (GNM) family, NoMaD enables robots to navigate to goals and explore environments using only visual inputs.

This repository contains a complete pipeline for data collection, preprocessing, training, and deployment of NoMaD-powered navigation systems within a ROS2 environment. The system supports:

- Recording topological maps from demonstration trajectories
- Processing ROS bag files into training data
- Training the NoMaD model using collected data
- Deploying the trained model for autonomous navigation

## System Requirements

- **OS**: Ubuntu 20.04+ 
- **ROS2**: Humble
- **Python**: 3.8+
- **GPU**: CUDA-compatible (for training)
- **Conda**: For environment management
- **Storage**: Minimum 10GB free space

## Project Structure

```
.
├── gnm_ws/                     # Main NoMaD workspace
│   ├── src/
│   │   ├── nomad/
│   │   │   ├── deploy/         # Deployment scripts and configs
│   │   │   ├── diffusion_policy/ # Diffusion policy implementation
│   │   │   ├── preprocessing/  # Data processing tools
│   │   │   └── train/          # Training code
│   │   ├── setup.sh            # Setup script
│   │   ├── run.sh              # Main execution script
│   │   └── README.md           # Documentation
├── cam_ws/                     # Camera visualization workspace
│   ├── src/
│   │   └── camera/
│   │       ├── cam_launch.py   # Camera launch file
│   │       └── config/
│   │           └── cam0.yaml   # Camera configuration
```

## Installation & Setup

### 1. Clone the Repository

```bash
# Create workspace directory
mkdir -p <workspace_directory>
cd <workspace_directory>

# Clone repository
git clone https://github.com/Gautham-Ramkumar03/Navigation-via-Masked-Diffusion-ROS2.git 
cd gnm_ws
```

### 2. Run the Setup Script

The setup script will create configuration files, download model weights, clone necessary submodules, and set up conda environments:

```bash
# Run setup from workspace directory
./src/setup.sh
```

Available options in the setup script:

1. **Full installation**: Sets up everything
2. **Installation for deployment**: Creates only the deployment environment
3. **Installation for training**: Creates only the training environment
4. **Installation without creating conda env**: Sets up configurations and downloads without creating environments
5. **Select what to install**: Customizable installation options

## Camera Workspace

The camera workspace (`cam_ws`) provides functionality for camera visualization and integration with the NoMaD system. It allows you to visualize the camera feed using RViz while the navigation is running.



### Setup Camera Workspace

```bash
# Build the camera workspace
cd <workspace_directory>/cam_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### Camera Configuration

The camera is configured with the following parameters (modifiable in `cam_ws/src/camera/config/cam0.yaml`):
- Resolution: 288x160
- Framerate: 10.0 FPS
- Pixel format: mjpeg2rgb
- Video device: /dev/video2 (adjust to match your camera)

### Launch Camera Node

```bash
# Launch the camera node
ros2 launch camera cam_launch.py
```

### Visualization with RViz

To visualize the camera feed while NoMaD is running:

```bash
# In a new terminal
source <workspace_directory>/cam_ws/install/setup.bash
ros2 run rviz2 rviz2

# In RViz:
# 1. Add an Image display
# 2. Set the Image Topic to /cam0/image_raw
# 3. Adjust view settings as needed
```

You can also create a custom RViz configuration file for easy visualization:

```bash
# Save your RViz configuration after setting it up
# Then load it with:
ros2 run rviz2 rviz2 -d <path_to_config_file>
```

## Usage Guide

### Data Collection & Preprocessing

1. **Record a trajectory**:
   ```bash
   # From workspace directory
   ./src/run.sh
   # Select option 1: Collect trajectory
   # Enter the bag name when prompted
   ```

2. **Create training data**:
   ```bash
   # From the run.sh menu
   # Select option 2: Create training data
   ```

3. **Create topological map**:
   ```bash
   # From the run.sh menu
   # Select option 3: Create topomap
   ```

### Training

1. **Train the model**:
   ```bash
   # Navigate to training directory
   cd src/nomad/train
   python train.py -c config/path.yaml
   ```

   The training script will:
   - Load the dataset defined in the configuration
   - Initialize the NoMaD model with ViNT encoder and diffusion policy
   - Train the model for the specified number of epochs
   - Save checkpoints to the `logs` directory

### Navigation

1. **Navigate using a trained model**:
   ```bash
   # From workspace directory
   ./src/run.sh
   # Select option 4: Navigate
   ```

2. **Visualize navigation with RViz**:
   ```bash
   # In a separate terminal
   source <workspace_directory>/cam_ws/install/setup.bash
   ros2 run rviz2 rviz2
   ```

## Configuration

### Model Configuration

The model configuration is defined in `src/nomad/train/config/model.yaml`. Key parameters include:

- `encoding_size`: Size of the visual embedding
- `context_size`: Number of context frames
- `down_dims`: Architecture of the diffusion UNet
- `num_diffusion_iters`: Number of diffusion steps
- `goal_mask_prob`: Probability of masking the goal during training

### Navigation Configuration

Navigation parameters are defined in `src/nomad/deploy/config/navigate.yaml`:

- `model_weights_path`: Path to the trained model weights
- `topomap_dir`: Directory containing the topological map
- `waypoint`: Current waypoint to navigate to
- `goal_node`: Target node in the topological map
- `v_max` and `w_max`: Maximum linear and angular velocities

### Camera Configuration

Camera parameters are defined in `cam_ws/src/camera/config/cam0.yaml`:
- `camera_name`: Name identifier for the camera
- `image_width` and `image_height`: Resolution settings
- `framerate`: Camera capture rate
- `video_device`: Path to the camera device
- `pixel_format`: Image encoding format

## Training Pipeline

The NoMaD training pipeline consists of:

1. **Data collection**: Record trajectories using the robot
2. **Data processing**: Convert raw bag files to structured training data
3. **Data splitting**: Split data into training and validation sets
4. **Model training**: Train the NoMaD model on the processed data
5. **Evaluation**: Evaluate the model's navigation performance

The training uses a combination of:
- **ViNT encoder**: Vision encoder for processing image sequences
- **Diffusion policy**: For generating action trajectories
- **Masked goal conditioning**: For robust navigation with varying goals

## Working with New Robots

To adapt this system to a new robot platform:

1. Modify the camera topics in `run.sh` to match your robot's camera
2. Adjust the odometry topic to match your robot's odometry source
3. Update maximum velocities in the navigation configuration file
4. Configure the PD controller parameters if needed for your robot's dynamics
5. Update the camera configuration in `cam_ws/src/camera/config/cam0.yaml`

## Troubleshooting

### Common Issues

1. **Missing model weights**:
   ```
   FileNotFoundError: [Errno 2] No such file or directory: '.../nomad.pth'
   ```
   Solution: Run the setup script again and select the download model option.

2. **CUDA out of memory**:
   ```
   RuntimeError: CUDA out of memory
   ```
   Solution: Reduce batch size in the training configuration.

3. **ROS topics not found**:
   ```
   ERROR: topic [/cam0/image_raw] does not exist
   ```
   Solution: Verify the camera is properly connected, publishing to the correct topic, and that the camera workspace is properly sourced.

4. **Camera not working**:
   ```
   Failed to open camera device
   ```
   Solution: Check the `video_device` parameter in the camera configuration matches your actual device.

## References

This work builds upon the following research papers:

1. **GNM**: A General Navigation Model to Drive Any Robot (ICRA 2023)
2. **ViNT**: A Foundation Model for Visual Navigation (CoRL 2023)
3. **NoMaD**: Goal Masking Diffusion Policies for Navigation and Exploration (2023)

For more information, please refer to the original publications.


## Acknowledgments

This project integrates work from:
- [Stanford REAL Lab](https://github.com/real-stanford/diffusion_policy)
- [Berkeley Robot Learning Lab](https://github.com/gkahn13/vint)
