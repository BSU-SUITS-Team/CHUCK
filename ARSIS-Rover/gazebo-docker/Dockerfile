# This is an auto generated Dockerfile for gazebo:gzserver11
# generated from docker_images/create_gzserver_image.Dockerfile.em
FROM ubuntu:jammy

# update and install basic stuff
RUN apt-get update -y
RUN apt-get install -y lsb-release wget gnupg git

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update -y
RUN DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get -y install tzdata
RUN apt-get install -y ignition-fortress gz-fortress

# CMD ign gazebo -v 4 -s --headless-rendering sensors_demo.sdf
RUN apt-get install -y mesa-utils libglu1-mesa-dev freeglut3-dev mesa-common-dev
CMD ign gazebo -v 4 -s sensors_demo.sdf
# CMD ["bash"]
