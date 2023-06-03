# Flying Object Detection and Tracking using LIDAR
Student: Ofek Weiss  

Student: Dan Komisarchick  

Supervisor: Prof. Yossi Yovel                                           

Project Carried Out at: TAU Garden for Zoological Research


## 1 Abstract
This project was conducted as part of a research which was carried out at the TAU Garden for Zoological Research. The scientific study of small flying creatures poses a significant challenge when it comes to monitoring their movement. To address this challenge, the project introduces a software system, using MATLAB, specifically designed to identify and track small flying objects.
<img width="359" alt="image" src="https://github.com/Dankomis/FinalProject2023/assets/109519884/b6beb342-b13a-4b00-b1a3-3a9890e253ea">

## 2 Implementation
In this chapter, the implementation and considerations for its choice will be described. 

<img width="367" alt="image" src="https://github.com/Dankomis/FinalProject2023/assets/109519884/bb7a0f90-db34-4c9e-9a04-5677c31e1856">

### 2.1 Hardware Description
The system focuses on SW, and get an input which has been generated   by Livox Horizon LIDAR.
One major issue that we learned from this table is how to parse the point cloud data. The point rate is 240,000 points/s (we used first or strongest return mode), frames rate is 10 frames/s and from this calculation we chose to divide the point cloud data to tables of 24,000 rows each such that every table includes the data of a specific frame. 
A description of components, tools, platforms and systems used in the project implementation, including appropriate diagrams. Our SW project could be compatible with a variety of LIDAR products which generate a csv point cloud file.

### 2.2 Software Description
As mentioned in the system’s description the main block that was implemented was the software based tracker. The LIDAR sensor outputs a raw point cloud data stream file, this stream of point cloud based frames will serve as an input to the tracker.
Before diving into the software implementation a quick discussion about the input data is vital.
#### 2.2.1 Input CSV Frame Stream
At the beginning of the system’s design and researching the LIVOX Horizon™ sensor we came to the conclusion there is a need for the system to be a generic software system due to the variety of sensors available and the different data file types each sensor outputs. Our understanding that the system cannot depend on a certain sensor brand we decided to use the raw CSV file data to represent the point cloud data stream in understanding this will allow future use of the system with different sensors and develop future versions of the system. 

The system was implemented using MATLAB™ software and will be executed on PC hardware in the TAU Garden for Zoological Research by the researchers. 
#### 2.2.2 MATLAB™ Software implementation
As mentioned the system was implemented with the MATLAB™ software. 
Before choosing this platform we were researching other options such as Python and the open source point cloud processing libraries such as open3D, PCL, Trimesh most of these options are more suitable specifically for other research field such as geology, autonomous vehicles etc. we concluded that for our specific purpose of tracking objects in a 3D point cloud and combining point cloud processing algorithms with basic tracking tools in the time frame of creating the first version of the system using MATLAB™  will offer an overall straight forward platform with variety of documentation and could help us a generic tracking system for our purpose.

#### 2.2.3 Software implementation blocks
In this section the system’s submodules implementation and functionality will be described.

<img width="412" alt="image" src="https://github.com/Dankomis/FinalProject2023/assets/109519884/d891dcbe-a16f-4ec7-b3c5-3cd17d3e4e05">

The figure above visualizes the main submodules of the system and the workflow of the software that implements it. Each module will be described:
#### 2.2.3.1 System parameter acquisition
Described as getParameters this module gets a data structure that holds the configuration information of the system as an input and outputs all the relevant system parameters for initialization and execution of the system.

The motivation for the configuration process is the main understanding of the project that the system should support different cases of tracking for multiple types of objects, environments and different LIDAR sensors. Therefore the system is completely configurable from the time of each frame, number of points in a frame and to the type of Kalman filter that will be used based on the motion model.

#### 2.2.3.2 Frame parsing
Based on the LIDAR sensor that was used this module will parse the raw data stream into frames according to the configured frame duration and number of points. The data will be assigned as n (number of frames) matrices that contain x,y,z and reflectivity columns for each point (row) and will be saved into a data structure.
This preprocess module will allow us to obtain the data in a rather simple structure and will save complexity and space.

#### 2.2.3.3 User interface
This module will output the initial location of the object that will be tracked and researched. By implementing the function “initPos()” the user will be prompted to mark with MATLAB data tip the centroid of the object, this mark will be used as the initial location to initialize the tracking system.

The idea of using this user interface came after a discussion with the zoological research team, as the users of the system they have mentioned the variety of subject’s they are studying and complexity of modeling the objects for detection and on the other hand the need in a system that as a research purpose of learning the motion and movement of the objects. Therefore the key assumption that the researchers can identify the objects and are focused on researching the movement of different flying subjects this module was implemented.

#### 2.2.3.3 Main frame loop
This is the main module of the system as mentioned in sections 2.3 -2.5 of the theoretical background. The initial frame will help to initialize the frame loop and this module will iterate each frame to estimate the object’s state vector.
The first submodule “getMeasurement” includes two main key features:

Object segmentation - using the estimated location from the previous frame a “gate” will be created, this is a region of interest (ROI) configured according to the object’s estimated speed. In the ROI all objects will be segmented into clusters by using a cluster algorithm.Different algorithms were compared and the results concluded that pcsegdist will be the choice.

Note: this module and segmentation algorithm is configurable as well.

Data association - using the concept of minimum Euclidean distance the centroid that represents the object will be chosen. 
This submodule outputs two arguments, the measured location and a flag which indicates if the object was detected.
The functionality of the submodule is a measurement function that outputs an observation that will be used to correct the estimation of the Kalman filter. 

As this module is completely configurable it can be used in future versions for sensor fusion and other features.
The second module includes the Kalman filter that was discussed in section 2.5 extensively, it uses two steps:

Prediction - in this step using the configured motion model and noise parameters the current state is estimated based on the previous frame state.

Correct - this is the correction step, the Kalman gain is computed based on the current and previous frame parameters and the state estimation will be corrected by weighing between the prediction and the measurement computed from the first submodule.

<img width="179" alt="image" src="https://github.com/Dankomis/FinalProject2023/assets/109519884/3641930d-09fb-4257-9a86-e57db737c357">

Note: If the first submodule as failed to produce a measurement the correction step won’t be used.

#### 2.2.3.4 system’s output
The system can output a variety of data. The system is described as a closed function that can be configured by an outer program and will simulate the system with the desired data files and configurations.

Using a data structure “verif” that will be filled during runtime with tracking information per frame, the program that simulates the system gets from the system variety of insights: video stream with tracking bounding box, location and velocity statistics, module success rates etc.

## Project Documentation
### System documentation access guide
As the system was mainly implemented by software this section will include a simple instruction guide to access the files and documentations.
The project is located under the github™ repository:

The repository includes 5 main parts:
#### LidarTrackerV1 
this folder includes the system implementation of the system and all of the documented code sections, it will be described in the next section.
#### LivoxHorizon/Docs 
this folder includes all the hardware user manuals and documentation that the system uses.
#### README.md (current file)
file that contains an explanation about the system’s functionality,implementation and  overview description.
#### User manual 
this file will contain an extensive description of the system implementation and a hand on explanation how to use it.
#### Recordings
all data and recordings the simulations where based upon.
### System documentation description
This section will describe the included files that implement the system and are located in the LidarTrackerV1 folder.

System modules - The system main file is LidarTRackerV1.m, this main file uses the configuration modules: getFrames.m, getParams.m. 

The user interface is implemented in initPos.m and the submodules of the system including Kalman filter and segmentation are implemented in:

MyEKF.m files and get_meas_i.m ( i version).

Auxiliary functions - The system includes the use of helper functions:

markRealLocation.m, movie_maker.m,getROI,fit_box.m

Simulation and testing functions - 

### System documentation Release notes
All files were implemented with the MATLAB™ R2021b version and includes the libraries: Computer Vision Toolbox and  Statistics and Machine Learning Toolbox.

## References
### Books:
Eli Brookner. "Tracking and Kalman Filtering Made Easy"1998 John Wiley & Sons, Inc.
### Papers:
Rongqing Su, Jun Tang, Jiangnan Yuan, Yuewen Bi. "Nearest Neighbor Data Association Algorithm Based on Robust Kalman Filtering." IEEE 2021.

Bima Sahbani, Widyawardana Adiprawita. "Kalman Filter and Iterative-Hungarian Algorithm Implementation for Low Complexity Point Tracking as Part of FastMultiple  Object Tracking System" 2016 IEEE 6th International Conference on System Engineering and Technology (ICSET) 

P. Morton, B. Douillard, J. Underwood. "An evaluation of dynamic object tracking with 3D LIDAR." Proceedings of Australasian Conference on Robotics and Automation, 7-9 Dec 2011, Monash University, Melbourne Australia.

### User's Guide:
"Livox Horizon User Manual", V1.0 2019.10
### Links:
Kenshi Saho "Kalman Filter for Moving Object Tracking: Performance Analysis and Filter Design", https://www.intechopen.com/chapters/57673
MATLAB™ – Tracking and Motion Estimation guide,  https://www.mathworks.com/help/vision/ug/using-kalman-filter-for-object-tracking.html?searchHighlight=use%20kalmanfilter&s_tid=srchtitle_use%2520kalmanfilter_2  
Alex Becker, ""Kalman Filter from the Ground Up", online tutorial https://www.kalmanfilter.net/default.aspx



