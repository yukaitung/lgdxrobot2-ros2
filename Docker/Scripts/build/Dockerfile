FROM ros:jazzy AS builder

ARG APP_VERSION=2.1.0

WORKDIR /src
COPY . .

RUN rosdep update
RUN apt-get update \
    && apt-get install -y \
    # Install debian packages build dependencies
    python3-bloom python3-rosdep fakeroot debhelper dh-python \ 
    dpkg tar devscripts \
    # Install dependencies
    && rosdep install --from-paths lgdxrobot2_agent --ignore-src -y \
    && rosdep install --from-paths lgdxrobot2_webots --ignore-src -y \
    && rm -rf /var/lib/apt/lists/*

# Zip the source code
RUN tar -czvf ros-${ROS_DISTRO}-lgdxrobot2-agent_${APP_VERSION}.orig.tar.gz lgdxrobot2_agent
RUN tar -czvf ros-${ROS_DISTRO}-lgdxrobot2-bringup_${APP_VERSION}.orig.tar.gz lgdxrobot2_bringup
RUN tar -czvf ros-${ROS_DISTRO}-lgdxrobot2-description_${APP_VERSION}.orig.tar.gz lgdxrobot2_description
RUN tar -czvf ros-${ROS_DISTRO}-lgdxrobot2-navigation_${APP_VERSION}.orig.tar.gz lgdxrobot2_navigation
RUN tar -czvf ros-${ROS_DISTRO}-lgdxrobot2-webots_${APP_VERSION}.orig.tar.gz lgdxrobot2_webots

# Complie the packages
WORKDIR /src/lgdxrobot2_agent
RUN bloom-generate rosdebian
RUN debuild -S -us -uc
RUN fakeroot debian/rules binary

## Install the agent package for webots
WORKDIR /src
RUN dpkg -i *.deb

WORKDIR /src/lgdxrobot2_bringup
RUN bloom-generate rosdebian
RUN debuild -S -us -uc
RUN fakeroot debian/rules binary

WORKDIR /src/lgdxrobot2_description
RUN bloom-generate rosdebian
RUN debuild -S -us -uc
RUN fakeroot debian/rules binary

WORKDIR /src/lgdxrobot2_navigation
RUN bloom-generate rosdebian
RUN debuild -S -us -uc
RUN fakeroot debian/rules binary

WORKDIR /src/lgdxrobot2_webots
RUN bloom-generate rosdebian
RUN debuild -S -us -uc
RUN fakeroot debian/rules binary

# Organise the output
## Debian packages
WORKDIR /src
RUN mkdir -p /debs
RUN mv *.deb /debs
RUN mv *.ddeb /debs
## Debian build files
RUN mkdir -p /debuild
RUN mv *.orig.tar.gz /debuild
RUN mv *.debian.tar.xz /debuild
RUN mv *.dsc /debuild
RUN mv *.build /debuild
RUN mv *.buildinfo /debuild
RUN mv *.changes /debuild

# Final stage outputs /src
FROM scratch AS export
COPY --from=builder /debuild /debuild
COPY --from=builder /debs /debs