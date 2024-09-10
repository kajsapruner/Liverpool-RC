# Packages that are essential for the Logitech camera

## Jetson Multimedia API
Often already preinstalled on most Jetson boards.

## v4l2 (Video for Linux 2)
Since the logitech camera uses a normal USB port we will need v4l2 so that the Jetson can interface this.
- Package: v4l-utils
- Command: sudo apt-get install v4l-utils
- You can use v4l2-ctl to check if the Jetson detects the camera.

## OpenCV
Will allow us to process the camera´s video feed, can capture frames from USB cameras (like Logitech) and has a number of computer 
vision libraries to help with object detection and image filtering for example.
- Commands:
-     sudo apt-get update
-     sudo apt-get install python3-opencv

## GStreamer
Often used on Jetson boards to support the use of hardware-accelerated video encoding/decoding and is compitable
with OpenCV (might not be neccesary but good to have?????)
- Commands:
-     sudo apt-get install gstreamer1.0-tools
-     sudo apt-get install gstreamer1.0-plugins-base gstreamer1.0-plugins-good

## PyTorch or TensorFlow
This is optional. If we are incorporating deep learning-based navigations (object recognition, reinforcement learning) we
will want either of these installed. Jetson has support for both libraries optimized for its GPUs.
- Follow these instructions to download this: https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html



# Other resources that I found that can be helpful when downloading packages and installing camera:
## NVIDIA Jetson Forum
Good for troubleshooting and tips
https://forums.developer.nvidia.com/

## NVIDIA Developer Documentation
Includes guides on multimedia, computer vision, and deep learning.
https://docs.nvidia.com/

## GitHub
We can look for Jetson and camera-related repositories here, and take help from other similar projects.

## OpenCV Tutorials
Find tutorials here and documentation specific to video input from cameras. Could help in the installation.
https://opencv.org/
