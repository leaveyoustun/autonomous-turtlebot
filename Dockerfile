# App image: build on top of existing environment
FROM leaveyou/ros1-noetic:latest

# Path to the catkin workspace that already exists in base image
ENV CATKIN_WS=/ros1_ws

# Copy package into the workspace
WORKDIR ${CATKIN_WS}/src

# Remove old version of the package in the base image
RUN rm -rf projet

# Copy this repo (GitHub checkout) into the workspace src
COPY . projet

# Build the workspace with the new code
WORKDIR ${CATKIN_WS}
RUN bash -lc "source /opt/ros/noetic/setup.bash && catkin_make"

