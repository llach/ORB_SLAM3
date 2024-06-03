FROM ubuntu:22.04

WORKDIR /home/slam

# Dev container arguments
ARG USERNAME=slam
ARG UID=${UID}
ARG GID=${GID} 

RUN apt update 
RUN apt install -y sudo cmake build-essential git wget

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
RUN apt update --fix-missing
RUN apt install -y libopencv-dev libeigen3-dev libboost-dev libboost-serialization-dev libssl-dev 

# Create new user and home directory
# hadolint ignore=DL3004
RUN groupadd -f --gid $GID $USERNAME \
 && useradd --uid ${UID} --gid ${GID} --create-home ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && chown -R ${UID}:${GID} /home/${USERNAME}

USER ${USERNAME}
RUN git clone https://github.com/stevenlovegrove/Pangolin 
WORKDIR Pangolin
RUN yes Y | ./scripts/install_prerequisites.sh recommended
RUN cmake -B build && cmake --build build

RUN mkdir /home/${USERNAME}/ORB3_SLAM
WORKDIR /home/${USERNAME}/ORB3_SLAM

ENTRYPOINT  ["/bin/bash", "-c"]
