#!/bin/bash
available_scenes=("cobot_simple" "cobot_conveyor")

MOUNT_POINT="/mnt/ssd_nvidia"
MOUNT_SOURCE="/dev/sda1"

# Check if the mount point folder exists
if [ ! -d "$MOUNT_POINT" ]; then
    echo "Mount point $MOUNT_POINT does not exist. Existing script."
    exit 1
fi

# If check if the mount point is already mounted and contain the NVIDIA local asset route
# If not, mount the drive
if [ ! -d "$MOUNT_POINT/NVIDIA" ]; then
    echo "$MOUNT_POINT has not been mounted. Mounting device /dev/sda1."
    sudo mount $MOUNT_SOURCE $MOUNT_POINT
    echo "successfylly mount $MOUNT_SOURCE to $MOUNT_POINT assets"
fi

PARENT_DIR="$(dirname "$(realpath "$0")")"

# If no argument, run the default scene

if [ -z "$1" ]; then
$ISAACSIM_DIR/isaac-sim.sh --ext-folder $PARENT_DIR --enable oc.utils.cobot --enable oc.scene.cobot_simple
exit 0
fi


# check if the argument is a valid scene, if not, print the available scenes and exit
is_valid_scene=false

for item in "${available_scenes[@]}"; do
    echo $item
    if [ "$1" == "$item" ]; then
        is_valid_scene=true
        break
    fi
done

if [ "$is_valid_scene" = false ]; then
    echo "Invalid scene name. Available scenes are: ${available_scenes[@]}"
    exit 1
fi

# Run the scene with the argument
$ISAACSIM_DIR/isaac-sim.sh --ext-folder $PARENT_DIR --enable oc.utils.cobot --enable oc.scene.$1