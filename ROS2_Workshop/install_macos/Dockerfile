FROM ros:foxy

SHELL ["/bin/bash", "-c"]

# dependencies
RUN apt-get update --fix-missing && \
    apt-get install -y ros-foxy-rviz2 \
		       vim \
		       wget \
		       curl \
           ros-foxy-teleop-twist-keyboard 

RUN curl -sSL http://get.gazebosim.org | sh

RUN apt-get -y dist-upgrade

RUN mkdir -p /jackal_files/jackal_ws/src

RUN cd /jackal_files/jackal_ws/src && \
    git clone -b foxy-devel https://github.com/jackal/jackal.git && \
    git clone -b foxy-devel https://github.com/jackal/jackal_desktop.git && \
    git clone -b foxy-devel https://github.com/jackal/jackal_simulator.git && \
    sed -i '59s/true/false/g' ./jackal/jackal_control/config/control.yaml && \
    sed -i '62s/4.0/25.0/g' ./jackal/jackal_control/config/control.yaml && \
    sed -i '63s/4.0/25.0/g' ./jackal/jackal_control/config/control.yaml

RUN cd /jackal_files/jackal_ws && \
    source /opt/ros/foxy/setup.bash && \
    apt-get update --fix-missing && \
    rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y && \
    colcon build

RUN echo "export JACKAL_LASER=1" >> ~/.bashrc

WORKDIR '/jackal_files'

ENTRYPOINT ["/bin/bash"]
