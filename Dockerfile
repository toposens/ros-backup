FROM ros:melodic-ros-core

ENV CXXFLAGS ${CXXFLAGS} -fdiagnostics-color
ENV PATH /usr/lib/ccache:$PATH
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y apt-utils ccache gcc \
    g++ tree python-pip python-catkin-tools \
    python-catkin-lint python-rosdep

COPY . /tmp
RUN rosdep update 2>&1 && rosdep install -y \
    --from-paths /tmp --ignore-src \
    --rosdistro ${ROS_DISTRO} --as-root apt:false
RUN rm -rf /tmp/*