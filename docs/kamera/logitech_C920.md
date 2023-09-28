# Aruco with Logitech C922

Manual - [wiki](http://wiki.ros.org/sick_safetyscanners)

- 1651 measurements
- angle resolution: 0.002909 rad
- scan angle: 275°

## usb_cam driver

## Calibration

Use of MonocularCalibration package: http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration



## Aruco markers

http://wiki.ros.org/fiducials

set correct parameters for camera:
roslaunch aruco_detect aruco_detect.launch camera:=/camera/color image:=image_raw

