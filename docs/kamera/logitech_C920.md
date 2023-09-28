# Aruco with Logitech C922

Logitech C922 

- 1080p @ 30 fps
- 720p @ 60 fps
- 3 Mpx
- diagonal field of view 78°
- autofocus

## usb_cam driver
First, a camera driver needs to be installed - [wiki](https://github.com/NVlabs/Deep_Object_Pose/blob/master/doc/camera_tutorial.md)

```bash
sudo apt-get install ros-melodic-usb-cam
```

Check for camera device (`ls /dev`, look for new `videoX`) and run the driver (`/dev/video3` replace with the correct device):
```bash
rosrun usb_cam usb_cam_node _camera_name:='usb_cam' _camera_frame_id:='usb_cam' _video_device:='/dev/video3' _image_width:=1920 _image_height:=1080
```

You can check the video stream in RViz (Add > By topic > /usb_cam/image_raw/image).


## Calibration 

With this procedure, the camera's intrinsic parameters are obtained.

Install  `camera_calibration` package - [wiki](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) 

```bash
sudo apt install ros-melodic-camera-calibration
```
Prepare the calibration checkerboard - example can be found here: [checkerboard, 12x16 fields, square side 15 mm](calib_12x16_15mm.pdf)

Run calibration node
```bash
rosrun camera_calibration cameracalibrator.py --size 11x15 --square 0.015 image:=/usb_cam/image_raw camera:=/usb_cam
```
Move the calibration checkerboard on the plane you want to calibrate until "Calibrate" button is active. After calibration procedure is finished commit calibration with the buttons "Save" and  "Commit".





## Aruco markers

Aruco markers are used to determine the pose of an object equipped with proper markers.

For this [fiducials](http://wiki.ros.org/fiducials) and [aruco_detect](http://wiki.ros.org/aruco_detect) packages are used.

Install packages:
```bash
sudo apt-get install ros-melodic-fiducials
sudo apt-get install ros-melodic-aruco-detect
```

Generate markers (14 cm side; for anything else, tune the .py file):
```bash
rosrun aruco_detect create_markers.py 100 110 fiducials.pdf
```

Use the `aruco_detect` package to detect aruco markers. Set the correct parameters for the camera.
```bash
roslaunch aruco_detect aruco_detect.launch camera:=/usb_cam image:=image_raw fiducial_len:=0.14 
```
When using Intel RealSense D435i run
```bash
roslaunch aruco_detect aruco_detect.launch camera:=/camera/color fiducial_len:=0.14 
```
Change `fiducial_len` to the size of markers used.


Pose of the marker is returned in the `/fiducial_transforms` topic. Markers can also be seen with
```bash
roslaunch fiducial_slam fiducial_rviz.launch
```


