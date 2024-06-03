#!/bin/bash
REPOSITORY_NAME="orb3_slam"

DOCKER_USER=slam
DOCKER_GID=$(id -g $(whoami))
DOCKER_UID=1001

echo "======================="
echo " Building docker image "
echo " IMAGE_TAG:   ${REPOSITORY_NAME}"
echo " DOCKER_USER: ${DOCKER_USER}"
echo " DOCKER_GID:  ${DOCKER_GID}"
echo " DOCKER_UID:  ${DOCKER_UID}"

docker rm -f ${REPOSITORY_NAME}
docker build ${DOCKER_FILE_PATH} --build-arg USERNAME="${DOCKER_USER}"\
                                 --build-arg GID=${DOCKER_GID}\
                                 --build-arg UID=${DOCKER_UID}\
                                 --progress=plain \
                                 -f ./docker/Dockerfile \
                                 -t "${REPOSITORY_NAME}" .
