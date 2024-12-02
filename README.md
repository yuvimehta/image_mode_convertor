# Image_Conversion_service

## 1. Project Overview
`image_mode_service` is a ROS 2 package that processes images from a topic and allows switching between **grayscale** and **color** modes via a service. It subscribes to an input image topic, processes the image, and publishes it to an output topic.

## 2. Clone and Build Instructions

### Clone the Repository
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clonehttps://github.com/yuvimehta/image_mode_convertor.git 
```
### Build the package and Setup Workspace

``` bash
cd ~/ros2_ws
colcon build --packages-select image_mode_service
source install/setup.bash
``` 
## 3. Launch instructions

### Launch file

``` bash
ros2 launch image_mode_service image_mode_service.launch.py input_image_topic:=/image_raw output_image_topic:=/processed_image
```
### Arguments

**input_image_topic**: Topic to subscribe for raw images (default: /image_raw).

**output_image_topic**: Topic to publish processed images (default: /processed_image)

### ROS2 Service to Change Image Mode

**Service Call Commands**:
```bash
# Grayscale mode set the bool to change to true
ros2 service call /set_mode std_srvs/srv/SetBool "{data: true}"

# For normal mode set the bool to false
ros2 service call /set_mode std_srvs/srv/SetBool "{data: false}"

```

### 4. Testing 
Run the launch file with the image topic
``` bash
ros2 launch image_mode_service image_mode_service.launch.py input_image_topic:=/image_raw output_image_topic:=/processed_image

```

Run the Rqt image viewer and select the topic the one you gave while launch the file with output_topic args

**Note:: by Default the output topic is "/processed_image" Select that.**

``` bash
ros2 run rqt_image_view rqt_image_view

```