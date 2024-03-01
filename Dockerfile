# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

FROM ros:foxy

SHELL ["/bin/bash", "-c"]

# dependencies
RUN apt-get update --fix-missing && \
    apt-get install -y git \
                       nano \
                       vim \
                       python3-pip \
                       libeigen3-dev \
                       tmux \
                       ros-foxy-rviz2 \
                       python3-venv 
RUN apt-get -y dist-upgrade

# python dependencies
RUN pip3 install --upgrade pip
RUN pip3 install transforms3d \ 
                 cython

# f1tenth gym
RUN git clone https://github.com/f1tenth/f1tenth_gym
RUN cd f1tenth_gym && \
    pip3 install -e .

# Raceline Optimization
RUN git clone https://github.com/ian-chuang/Raceline-Optimization
# Create a virtual environment and install dependencies
RUN cd Raceline-Optimization && \ 
    python3 -m venv venv && \
    source venv/bin/activate && \
    pip3 install -r requirements.txt

# particle filter dependencies
RUN git clone https://github.com/f1tenth/range_libc
RUN cd range_libc/pywrapper && \
    ./compile.sh

# ros2 gym bridge
RUN mkdir -p sim_ws/src/f1tenth_gym_ros
COPY ./src /sim_ws/src
RUN source /opt/ros/foxy/setup.bash && \
    cd sim_ws/ && \
    apt-get update --fix-missing && \
    rosdep install -i --from-path src --rosdistro foxy -y && \
    colcon build

# source in bashrc
RUN echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc
RUN echo 'source /sim_ws/install/local_setup.bash' >> ~/.bashrc

WORKDIR '/sim_ws'
ENTRYPOINT ["/bin/bash"]
