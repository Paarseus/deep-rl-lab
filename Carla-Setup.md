## Building Carla using Docker
Documentation: https://carla-ue5.readthedocs.io/en/latest/build_docker/
(I used docker to build Carla since i'm on Ubuntu 24.04 and Carla hasnt supported this version yet!)

### Step 1: Install Docker
Here's the tutorial for installing Docker on Ubuntu: <br>
https://docs.docker.com/engine/install/ubuntu/

### Step 2: Post-installation steps for Docker Engine
The post-installation gives root access for using Docker: <br>
https://docs.docker.com/engine/install/linux-postinstall/

### Step 3: Installing the NVIDIA Container Toolkit
As we need NVIDIA Drivers to run the Unreal Engine with Carla:
https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

### Step 4: Pull the CARLA Docker image
Pull the image with the following command:
```Bash
docker pull carlasim/carla:0.10.0
```

### Step 5: Run the image
Running the Docker image without display:
```Bash
docker run \
    --runtime=nvidia \
    --net=host \
    --env=NVIDIA_VISIBLE_DEVICES=all \
    --env=NVIDIA_DRIVER_CAPABILITIES=all \
    carlasim/carla:0.10.0 bash CarlaUnreal.sh -RenderOffScreen -nosound
```

Running the Docker image with a display:
To run the Docker image with a display, you will need the x11 display protocol:
```Bash
docker run \
    --runtime=nvidia \
    --net=host \
    --user=$(id -u):$(id -g) \
    --env=DISPLAY=$DISPLAY \
    --env=NVIDIA_VISIBLE_DEVICES=all \
    --env=NVIDIA_DRIVER_CAPABILITIES=all \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    carlasim/carla:0.10.0 bash CarlaUnreal.sh -nosound
```
