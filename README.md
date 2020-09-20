# Project: Turtle-Sketch



## Overview

- In this project, the image on the left is taken as input and the image on the right is obtained as output by using turtlesim

![Output](./sketch/docs/Output.png)



- The image on the left can be selected using it's path as well as can be directly taken from the camera

- The below videos show the project in action

  <iframe width="1188" height="622" src="https://www.youtube.com/embed/ulUPPNfbd6A" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

  <iframe width="1195" height="622" src="https://www.youtube.com/embed/_Yt93x_M1ck" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

  



---



## Concepts Used

Following are the concepts used for this project:

- **Dynamic reconfigure**
  - To import  the image using path
  - To import the image using camera
  - To set thresholds for Canny edge detection 
- **ROS parameters** 
  - To set the values of threshold parameters
  - To fetch the  values of threshold parameters
- **ROS services** 
  - To spawn the turtles
  - To teleport the turtle
  - To set the status of a pen as - ON/OFF
  - To remove the turtles after sketch is completed
- **OpenCV**
  - To import image
  - To find the edges using Canny edge detection
  - To find the contours
- **Multi-processing** 
  - To spawn an army of turtles to draw the sketch



---



## Directory Structure

- The directory contains 3 packages:
  - `sketch`
  - `dynamic reconfigure`
  - `image_thresholding`

- Sketch directory structure

  ```
  ├── CMakeLists.txt
  ├── docs                                 # Supported files for documentation
  │   ├── Contours.png
  │   ├── done.png
  │   ├── dynamic reconfigure.png
  │   ├── edges.png
  │   ├── Output.png
  │   ├── rosgraph.png
  │   ├── test.png
  │   └── turtles.png
  ├── include
  │   └── sketch
  ├── launch                                # Launch Files
  │   └── sketcher.launch
  ├── package.xml
  ├── README.md
  ├── nodes                                 # ROS Nodes
  │   ├── ironman.jpeg
  │   ├── sketcher.py
  │   └── Turtle.py
  └── TODO.md                               # TO DO for nect version
  ```

- Image_thresholding directory structure

  ```
  .
  ├── cfg                                    # Configuration file for GUI
  │   ├── import.cfg                         # GUI params related to image import
  │   └── thresholds.cfg                     # GUI params related to image thresholding
  ├── CMakeLists.txt
  ├── include
  │   └── dynamic_parameters
  ├── launch                                 # Launch files 
  │   └── canny_thresholding.launch          # GUI testing
  ├── nodes                                  # ROS Nodes
  │   ├── get_values.py
  │   ├── import_server.py                   # Import Image import parameters in python node
  │   └── server.py                          # Import thresholding parameters in python node
  └── package.xml
  ```



---



## Coding Style Guide - PEP8



---



## Dependencies

- `dynamic reconfigure` package
- OpenCV



---



## Setup and Run

To run the project on your local system, follow the procedure:

- Download the packages - `sketch`, `image_thresholding`, and `dynamic-reconfigure-noetic-devel`
- Copy these packages to your ROS workspace i.e. `~/ROS_ws/src/`
- Build the workspace
  - `$ cd ~/ROS_ws/` 
  - `$ catkin_make`

- Open new terminal and source the ROS workspace - `source ~/ROS_ws/devel/setup.bash`

- Run the command - `$ roslaunch sketch sketcher.launch`

- This command will open turtlesim and GUI for this project

  ![](./sketch/docs/test.png)

  ![](./sketch/docs/dynamic reconfigure.png)



- You can select image **Address (0)** or **Camera (1)** option for `CaptureType`
- If you select *Address (0)*, insert the path of the image on you system in the `img_path ` section
- Then click on the checkbox in front of `Capture`

- If you select *Camera (0)*, camera window will pop-up and you can click on the checkbox in front of `Capture` once you get the desired frame

- This  will open up a window with edges in the selected frame

  ![](./sketch/docs/edges.png)

  

- Now, use the GUI to set minimum and maximum threshold values to get the desired contours

- Click on the checkbox in front of `Start` to spawn an army of turtles which will sketch these contours for you

  ![](./sketch/docs/turtles.png)

- After the sketch is completed, the turtles will disapper

  ![](./sketch/docs/done.png)



**Note:**

- If you want to change the approach to sequential from parallel, follow the steps:
  1. Open launch file in this directory `~/ROS_ws/sketch/launch/`
  2. The `arg` tag for the sketcher node has a value of 1
  3. Change this value to `0`
- Here, 0 = sequential execution and 1 = parallel execution



---



## Article

[<img src="https://cdn.mos.cms.futurecdn.net/xJGh6cXvC69an86AdrLD98-320-80.jpg" style="zoom: 70%;">](https://medium.com/@shilpajbhalerao/ros-turtlesim-playground-cbc867924a8)



---



## Contact

[<img src="https://www.iconfinder.com/data/icons/popular-social-media-flat/48/Popular_Social_Media-22-512.png" alt="LinkedIn Logo" style="zoom: 25%;" />](https://www.linkedin.com/in/shilpaj-bhalerao/)[<img src="https://image.flaticon.com/icons/png/512/25/25231.png" style="zoom: 15%;">](https://github.com/Shilpaj1994)      [<img src="data:image/png;base64,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" style="zoom: 35%;">](https://www.youtube.com/channel/UCucf49_Iau18mG5YFFCSpmw?view_as=subscriber)

