# Traffic Analysis & Road Model Generation Tool

This project provides a cutting-edge solution for traffic analysis and 3D road model generation. It leverages state-of-the-art deep learning models to detect vehicles, pedestrians, and more from various inputs, and then uses the generated data to recreate real-world scenes in the Gazebo 3D simulation environment. This dual-purpose approach not only helps lawmakers and urban planners better understand road conditions but also produces high-quality simulated data for training self-driving (FSD) models.

Demo video: https://youtu.be/yPgG6S0mtiw
---
3D Model video: https://youtu.be/2RmpJdB2nf4
---
## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [How It Works](#how-it-works)
- [Data Management](#data-management)
- [Use Cases](#use-cases)
- [Models and Performance](#models-and-performance)
- [Getting Started](#getting-started)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

The tool is designed to analyze traffic footage and generate accurate 3D representations of road scenes. By processing input from:

- Single images
- Dashcam footage
- Traffic camera feeds

The system extracts crucial metrics and scene details using a fine-tuned version of YOLOv11 and the latest depth estimation techniques. The outcome is a live, dynamic MongoDB database (`bingus bongus`) that stores detailed scene data for both traffic analytics and 3D simulation.

---

## Features

- **Multiple Input Formats:**  
  Supports single images, dashcam videos, and traffic camera footage.

- **Advanced Object Detection:**  
  Uses a fine-tuned YOLOv11n model, optimized on a dataset of 10,000+ images from dashcams and traffic lights, to detect vehicles, pedestrians, and more with precise bounding boxes.

- **Accurate Depth Estimation:**  
  Integrates Depth Anything v2 to generate high-fidelity depth maps. Utilizes a robust triangulation method as mentioned below.

- **3D Scene Recreation:**  
  Recreates real-world traffic scenes in the Gazebo simulation environment, enabling realistic virtual scenarios.

- **Traffic Analytics:**  
  Computes important metrics like:
  - **Unique Vehicle Count:** Count distinct vehicles across the entire video using their unique track ID.
  - **Filtering by Distance:** Filter detections to include only vehicles within a specified distance (e.g., z < 20), focusing on near-field traffic.
  - **Number of Cars per Frame:** Group detections by frame (timestamp) and count the number of cars in each frame. Apply a moving average to the raw car counts per frame to generate a smoothed trend line.
  - **Average Pairwise Distance Between Cars per Frame:** Compute the average distance between all pairs of vehicles within each frame to assess vehicle clustering or dispersion.
  - **Heatmap of Vehicle Positions:** Create a 2D histogram (heatmap) of the (x, y) positions of vehicles to visualize areas of high and low vehicle density.

- **Real-Time Data Storage:**  
  All processed data is stored on a live MongoDB server, allowing seamless access for both traffic analysis and 3D simulation modules.

---

## How It Works

1. **Input Processing:**  
   - **Image/Video Upload:** Users can upload a single image or video footage.
   - **Frame Extraction:** Videos are split into frames at 3 frames per second (3fps), which provides sufficient data density for analysis without overloading the system.

2. **Object Detection with YOLOv11:**  
   - Each frame is processed using a fine-tuned YOLOv11n model with a confidence threshold of 0.5.
  
3. **Depth Estimation:**  
   - **Triangulation:** The top two most confident vehicle detections (typically the closest) are used to calculate their real distances using their bounding box heights and the pixel focal length.
   - **Depth Mapping:** A depth map is generated using Depth Anything v2. This map is then linearly scaled using the triangulated distances, providing accurate depth predictions for all detected objects.
   - **Coordinate Calculation:** Real-world x and y coordinates are computed using homogenization formulas based on the depth data.

4. **Data Storage and Utilization:**  
   - All extracted data (3D coordinates, object counts, etc.) are stored in the MongoDB collection `bingus bongus` on Atlas.
   - **Traffic Analysis Module:** Utilizes stored data to calculate metrics like average vehicle counts and inter-vehicle distances.
   - **3D Simulation Module:** Uses real-world coordinates to accurately recreate the scene in Gazebo, aiding in realistic virtual simulations for self-driving model training.

---

## Use Cases

- **Urban Planning & Lawmaking:**  
  Provides critical traffic metrics and road usage data to help shape better road policies and infrastructure planning.

- **Self-Driving Model Training:**  
  Generates realistic 3D simulations of road scenes for training autonomous driving models, bridging the gap between simulated and real-world driving conditions.

---

## Models and Performance

- **YOLOv11n Fine-Tuning:**  
  Trained on a dedicated dataset of 10,000 images from dashcams and traffic light footage, enhancing the model’s accuracy in detecting cars and pedestrians with tight bounding boxes.

- **Depth Anything v2 Integration:**  
  Our depth estimation process leverages advanced triangulation techniques, providing superior performance compared to traditional monocular depth perception models. This ensures both high accuracy and efficient processing speeds.

- **Performance Metrics:**  
  Based on a benchmark study, YOLOv11 achieved:
  - **mAP@0.5:** 76.8%
  - **mAP@0.75:** 68.1%
  - **mAP@[0.5:0.95]:** 48.5%
  - **Inference Speed:** 290 FPS
  - **F1 Score:** 0.71 at confidence 0.61
  - **Precision-Recall Balance:** Demonstrates high accuracy for vehicles, with minor misclassifications in occluded scenarios.
  - The remaining metrics can be accessed here: https://arxiv.org/html/2410.22898v1
---

## Getting Started

### Prerequisites

- Python 3.10
- Above Jupyter Notebooks
- MongoDB server (ensure the collection `bingus bongus` is configured)
- Gazebo for 3D simulation

### Installation

1. **Clone the repository:**

   ```bash
   git clone https://github.com/dnfy502/traffic-analysis.git
   ```

3. **Configure MongoDB:**

   Ensure your MongoDB instance is running and accessible. Update the configuration file with the correct connection string if necessary.

4. **Run the main CoordsCalc Jupyter Notebook (preferably in Google Colab):**

   ```bash
   CoordsCalc(5).ipynb
   ```
5. **Upload your choice of video in Google Colab, edit the videofile variable and run the code.**
6. **Run the Data Analysis Jupyter Notebook (preferably in Google Colab):**

   ```bash
   data-analysis.ipynb
   ```
7. **Install ROS packages**
  Make a workspace and create a directory 'src' where modell package will be stored, copy this repo's modell to get the packages and then build the catkin workspace.
  
  cd ~/kh/src/
  git clone https://github.com/dnfy502/traffic_analysis.git
  cd ~/kh && catkin build
  
  Source your workspace in .bashrc file by running the following command so that you don't have to source it in every terminal
  
  echo "source ~/robo_arm/devel/setup.bash" >> ~/.bashrc
  

8. **To start the Gazebo simulation:**
  
  ```bash
  roslaunch gazebo_ros empty_world.launch
  ```

9. **To spawn the robots**
  
  ```bash
  rosrun modell multi_box.py
  ```
