#!/bin/bash

# Paths to the YAML files
NAVIGATE_YAML="src/nomad/deploy/config/navigate.yaml"
MODEL_YAML="src/nomad/train/config/model.yaml"
CONTROLLER_YAML="src/nomad/deploy/config/controller.yaml"

# Function to modify a variable in the YAML file
update() {
    local file=$1
    local var_name=$2
    local new_value=$3
    sed -i "s/^\(\s*$var_name:\).*/\1 $new_value/" "$file"
}

# Function to read the current value of a variable from the YAML file
show() {
    local file=$1
    local var_name=$2
    grep "^\s*$var_name:" "$file" | sed "s/^\s*$var_name:[[:space:]]*//"
}

# Function to handle input and variable modification
tune() {
    local var_name=$1
    local file=$2
    local current_value=$(show "$file" "$var_name")
    read -p "Enter new value for $var_name (current: $current_value): " new_value
    update "$file" "$var_name" "$new_value"
}

# Main menu function
main_menu() {
    while true; do
        clear
        echo "Choose a variable to tune:"
        echo "A. v_max (current: $(show "$NAVIGATE_YAML" "v_max"))"
        echo "B. w_max (current: $(show "$NAVIGATE_YAML" "w_max"))"
        echo "C. hz (current: $(show "$NAVIGATE_YAML" "hz"))"
        echo "D. graph_hz (current: $(show "$NAVIGATE_YAML" "graph_hz"))"
        echo "E. num_samples (current: $(show "$NAVIGATE_YAML" "num_samples"))"
        echo "F. radius (current: $(show "$NAVIGATE_YAML" "radius"))"
        echo "G. goal_node (current: $(show "$NAVIGATE_YAML" "goal_node"))"
        echo "H. num_diffusion_iters (current: $(show "$MODEL_YAML" "num_diffusion_iters"))"
        echo "I. len_traj_pred (current: $(show "$MODEL_YAML" "len_traj_pred"))"
        echo "J. context_size (current: $(show "$MODEL_YAML" "context_size"))"
        echo "K. v_max (pd_controller) (current: $(show "$CONTROLLER_YAML" "v_max"))"
        echo "L. w_max (pd_controller) (current: $(show "$CONTROLLER_YAML" "w_max"))"
        echo "M. frame_rate (pd_controller) (current: $(show "$CONTROLLER_YAML" "frame_rate"))"
        echo "N. waypoint_timeout (pd_controller) (current: $(show "$CONTROLLER_YAML" "waypoint_timeout"))"
        echo "O. waypoint (current: $(show "$NAVIGATE_YAML" "waypoint"))"
        echo "P. close_threshold (current: $(show "$NAVIGATE_YAML" "close_threshold"))"
        echo "X. Exit"
        read -p "Enter your choice: " choice
        echo    # move to a new line

        case ${choice^^} in
            A) tune "v_max" "$NAVIGATE_YAML" ;;
            B) tune "w_max" "$NAVIGATE_YAML" ;;
            C) tune "hz" "$NAVIGATE_YAML" ;;
            D) tune "graph_hz" "$NAVIGATE_YAML" ;;
            E) tune "num_samples" "$NAVIGATE_YAML" ;;
            F) tune "radius" "$NAVIGATE_YAML" ;;
            G) tune "goal_node" "$NAVIGATE_YAML" ;;
            H) tune "num_diffusion_iters" "$MODEL_YAML" ;;
            I) tune "len_traj_pred" "$MODEL_YAML" ;;
            J) tune "context_size" "$MODEL_YAML" ;;
            K) tune "v_max" "$CONTROLLER_YAML" ;;
            L) tune "w_max" "$CONTROLLER_YAML" ;;
            M) tune "frame_rate" "$CONTROLLER_YAML" ;;
            N) tune "waypoint_timeout" "$CONTROLLER_YAML" ;;
            O) tune "waypoint" "$NAVIGATE_YAML" ;;
            P) tune "close_threshold" "$NAVIGATE_YAML" ;;
            X)
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
echo "Tuned"

