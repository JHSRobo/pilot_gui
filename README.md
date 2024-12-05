![Image](./img/logo.png)

12/5/24 Version 1.0:


**Contributors:** Jack Frings '26

**Editors:** Jack Frings '26 

**Approved by:** Pending

---
This repository contains the software to receive camera streams from the ROV and display said camera data to the pilot. 

## Finding Cameras
While runing the ROV, every camera constantly attempts to connect with the TCU laptop over TCP ([camera_stream](https://github.com/jhsrobo/camera_stream)). The TCU, however, chooses to only listen to one of these cameras at a time. All of these cameras are managed from a cam_config.json file. When running topside, if there is no cam_config.json file present, find_cameras.py will look for any available cameras accesible over ethernet. The IPs of these cameras will then be stored in cam_config.json. The file can then be manually edited to change the titles of the cameras and their assigned grippers.

## Displaying the Camera Feed 
The camera feed from the currently active camera is then displayed to the TCU monitor with a gui overlay according to [camera_overlay.py](https://github.com/jhsrobo/corews/blob/main/src/core/core_lib/camera_overlay.py) in core. 
## Ops Integration
The camera feed of the currently active camera is also split over the ros topic camera_feed to the ops laptop.
