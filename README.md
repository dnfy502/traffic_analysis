# traffic_analysis
Our mongodb collection is called bingus bongus. God help us all.

This is a cool ass Traffic Analysis and Road Model generation tool.
You can upload:
i) a single image
ii) dashcam footage
iii) traffic camera footage.
We do traffic analysis using a fine-tuned YOLOv11n model to detect cars, pedestrians, etc. and use the generated data to recreate the scene in Gazebo (3D modelling). 
This is useful not only for lawmakers and roadmakers to understand the ground reality of their roads, but also to get useful 3D simulated data for training FSD models.

How it works:
We use YOLOv11 + depth anything v2. When the footage is uploaded, we split the video into 3fps (while we can run at decent fps, we found for analysis and 3D model creation that 3fps is more than good enough.
These frames are then run through YOLO. For accurate depth perception, we first find the top 2 most confident (usually closest) car predictions and triangulate their real distance using their height and pixel focal length. 
We further use a depth map created using Depth Anything V2 (the most accurate and performant model we found). By linearly mapping the depth across the depth map using the real distances from the most confident cars, we can get a very good distance prediction for the rest of the cars.
Real x and real y are found using homogenisation formulae.

All this data is then stored in a live MongoDB server. This data is accessed by two different tools: traffic analysis and 3D model creation in Gazebo.
The traffic analysis uses xyz coordinates as well as number of cars per second to calculate different important metrics including average no. of cars, average distance between cars, that lawmakers can put to good use.
The 3D modelling software uses the real x,y and distance estimates to recreate the exact scene in Gazebo. This simulation can prove very useful to train real Self Driving models, so that they can be trained using real scenarios in virtual simulations.

Our models used have the following metrics along with the given fine tunes:
1) Our complex depth triangulation and calculation using depth anything is better than any monocular metric depth perception model we found online, along with being far more performant.
2) Our instance of YOLO11 has been finetuned on a dataset of 10000 images from dashcam and traffic light data, to further improve car and pedestrian detection with tight bounding boxes.


Sincerely ,
boga
