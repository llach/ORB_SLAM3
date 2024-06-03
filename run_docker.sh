#!/bin/bash
REPOSITORY_NAME="orb3_slam"

DOCKER_VOLUMES="
--volume=".:/home/slam/ORB3_SLAM" \
--env DISPLAY=docker.for.mac.host.internal:0 \
"
DOCKER_ARGS=${DOCKER_VOLUMES}


# container neither running nor stopped? → create
if [[ -z "$(docker ps -a -q -f name=${REPOSITORY_NAME})" ]];
then
  echo "creating container"
  docker create -i -t --name ${REPOSITORY_NAME} \
                ${DOCKER_ARGS} \
                ${REPOSITORY_NAME} bash
fi

# container not running? → start
if [[ -z  "$(docker ps -q -f name=${REPOSITORY_NAME})" ]];
then
  echo "starting container"
  docker container start -a -i $REPOSITORY_NAME
else # else just start a new shell session in the container
  docker container exec -it $REPOSITORY_NAME bash
fi
