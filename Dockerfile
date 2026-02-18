# Please note that this dockerfile is for building the .deb files only

FROM ros:jazzy AS builder

ARG APP_VERSION=2.1.0

WORKDIR /src
COPY . .

# Fix ARM64 build network error
RUN sed -i 's|http://ports.ubuntu.com/ubuntu-ports|https://mirrors.ocf.berkeley.edu/ubuntu-ports/|g' /etc/apt/sources.list.d/ubuntu.sources
RUN rosdep update
RUN apt-get update \
    && apt-get install -y \
    # Install debian packages build dependencies
    python3-bloom python3-rosdep fakeroot debhelper dh-python \ 
    dpkg wget \
    # Install dependencies
    && rosdep install --from-paths lgdxrobot2_agent --ignore-src -y \
    && rosdep install --from-paths lgdxrobot2sim_webots --ignore-src -y \
    && rm -rf /var/lib/apt/lists/* \
    # Install LGDXRobotics Source
    && wget -q http://packages.bristolgram.uk/lgdxrobotics-apt-source.deb \
    && dpkg -i lgdxrobotics-apt-source.deb \
    && rm lgdxrobotics-apt-source.deb

# Install LGDXRobot Cloud msgs
RUN apt update \
  && apt install -y --no-install-recommends ros-${ROS_DISTRO}-lgdxrobot-cloud-msgs \
  && rm -rf /var/lib/apt/lists/* 

# Complie the packages
WORKDIR /src/lgdxrobot2_msgs
RUN bloom-generate rosdebian
RUN fakeroot debian/rules binary

## Install the agent package for webots
WORKDIR /src
RUN dpkg -i *.deb

WORKDIR /src/lgdxrobot2_agent
RUN bloom-generate rosdebian
RUN fakeroot debian/rules binary

WORKDIR /src/lgdxrobot2_bringup
RUN bloom-generate rosdebian
RUN fakeroot debian/rules binary

WORKDIR /src/lgdxrobot2_description
RUN bloom-generate rosdebian
RUN fakeroot debian/rules binary

WORKDIR /src/lgdxrobot2_navigation
RUN bloom-generate rosdebian
RUN fakeroot debian/rules binary

WORKDIR /src/lgdxrobot2sim_webots
RUN bloom-generate rosdebian
RUN fakeroot debian/rules binary

# Organise the output
## Debian packages
WORKDIR /src
RUN mkdir -p /debs
RUN mv *.deb /debs
#RUN mv *.ddeb /debs

# Final stage outputs /src
FROM scratch AS export
COPY --from=builder /debs /debs