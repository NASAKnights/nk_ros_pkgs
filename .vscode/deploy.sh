#!/bin/bash

REMOTE_USER=$1
REMOTE_IP=$2    
WORKSPACE_LOCAL=$3
WORKSPACE_REMOTE=$4
CONTAINER_NAME="vision-container"

# Stop and remove the existing container if it's running
echo "Stopping existing container..."
ssh -t "$REMOTE_USER@$REMOTE_IP" "docker stop $CONTAINER_NAME"


# Sync code to Jetson
echo "Syncing code to Jetson..."
rsync -avz --delete --exclude 'logs/' --exclude 'build/' --exclude 'install/' "$WORKSPACE_LOCAL/" "$REMOTE_USER@$REMOTE_IP:$WORKSPACE_REMOTE"

echo "Starting new container..."
ssh -t "$REMOTE_USER@$REMOTE_IP" "docker start $CONTAINER_NAME &&
    docker exec vision-container /bin/bash -c 'source /opt/ros/humble/setup.bash && cd /workspaces/isaac_ros-dev && colcon build --symlink-install'"

echo "Code Rebuilt, Running Robot..."

ssh -t "$REMOTE_USER@$REMOTE_IP" "docker exec -d vision-container /bin/bash -c 'cd /workspaces/isaac_ros-dev && source install/setup.bash && .run_robot.sh'"

echo "Deployment complete!"
