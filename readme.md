# ros_dvs_bridge

Work done at CNRS-AIST JRL. This package enables the direct visual servoings of https://github.com/jrl-umi3218/DirectVisualServoing to run within a ROS wrapping. 

Authors: Guillaume Caron, Belinda Naamani ...

Dates: from April 2025 to ...

# Pre-requisities

Tested under `Ubuntu 22.04` and `ROS Humble`

## ros packages:
- `spinnaker`: to use Flir camera (Flir Spinnaker SDK, version 3.0.0.118 tested: `https://flir.app.boxcn.net/v/SpinnakerSDK?pn=Spinnaker+SDK&vn=Spinnaker_SDK`)

- `flir_camera_driver`: to use Flir camera with ROS2 (`https://github.com/ros-drivers/flir_camera_driver`)

- `visp_bridge`: to convert ROS messages to ViSP data formats, mainly grayscale images (`https://github.com/lagadic/vision_visp`)

-`The imaging source camera`: To use The Imaging source camera, use : `https://github.com/meriemBelinda7/the_imaging_source_image_publisher`



### Example with UR10 equipped with The Imagig source camera:

After creating a directory `ros2_dvs_bridge` in your directory `$HOME/.ros`

1- To capture the desired image run :

`ros2 launch captureAndSaveDsiredImage.py`

2- To start the defocus based visual servoing run :

`ros2 launch defocusBasedVisualServoing.py`

