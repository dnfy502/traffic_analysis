# Traffic Analysis & Road Model Generation Tool

This project provides a cutting-edge solution for traffic analysis and 3D road model generation. It leverages state-of-the-art deep learning models to detect vehicles, pedestrians, and more from various inputs, and then uses the generated data to recreate real-world scenes in the Gazebo 3D simulation environment. This dual-purpose approach not only helps lawmakers and urban planners better understand road conditions but also produces high-quality simulated data for training self-driving (FSD) models.

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

the system extracts crucial metrics and scene details using a fine-tuned version of YOLOv11 and the latest depth estimation techniques. The outcome is a live, dynamic MongoDB database (humorously named `bingus bongus`) that stores detailed scene data for both traffic analytics and 3D simulation.

---

## Features

- **Multiple Input Formats:**  
  Supports single images, dashcam videos, and traffic camera footage.

- **Advanced Object Detection:**  
  Uses a fine-tuned YOLOv11n model, optimized on a dataset of 10,000 images from dashcams and traffic lights, to detect vehicles, pedestrians, and more with precise bounding boxes.

- **Accurate Depth Estimation:**  
  Integrates Depth Anything v2 to generate high-fidelity depth maps. Utilizes a robust triangulation method:
  - Extracts the top two most confident (usually the closest) vehicle predictions.
  - Uses their heights and pixel focal length to triangulate real-world distances.
  - Linearly maps depth across the entire depth map for reliable predictions of all objects.

- **3D Scene Recreation:**  
  Recreates real-world traffic scenes in the Gazebo simulation environment, enabling realistic virtual scenarios.

- **Traffic Analytics:**  
  Computes important metrics like:
  - Average number of cars per second
  - Average distance between vehicles  
  These metrics can assist lawmakers and road engineers in making data-driven decisions.

- **Real-Time Data Storage:**  
  All processed data is stored on a live MongoDB server, allowing seamless access for both traffic analysis and 3D simulation modules.

---

## How It Works

1. **Input Processing:**  
   - **Image/Video Upload:** Users can upload a single image or video footage.
   - **Frame Extraction:** Videos are split into frames at 3 frames per second (3fps), which provides sufficient data density for analysis without overloading the system.

2. **Object Detection with YOLOv11:**  
   - Each frame is processed using a fine-tuned YOLOv11n model that has been optimized on a specialized dataset for high precision in detecting vehicles and pedestrians.
  
3. **Depth Estimation:**  
   - **Triangulation:** The top two most confident vehicle detections (typically the closest) are used to calculate their real distances using their bounding box heights and the pixel focal length.
   - **Depth Mapping:** A depth map is generated using Depth Anything v2. This map is then linearly scaled using the triangulated distances, providing accurate depth predictions for all detected objects.
   - **Coordinate Calculation:** Real-world x and y coordinates are computed using homogenization formulas based on the depth data.

4. **Data Storage and Utilization:**  
   - All extracted data (3D coordinates, object counts, etc.) are stored in the MongoDB collection `bingus bongus`.
   - **Traffic Analysis Module:** Utilizes stored data to calculate metrics like average vehicle counts and inter-vehicle distances.
   - **3D Simulation Module:** Uses real-world coordinates to accurately recreate the scene in Gazebo, aiding in realistic virtual simulations for self-driving model training.

---

## Data Management

The project uses a live MongoDB server to store all processed data, ensuring efficient and real-time access for both analytical and simulation purposes. The MongoDB collection is humorously named **`bingus bongus`**, reflecting the project’s playful yet innovative spirit.

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

---

## Getting Started

### Prerequisites

- Python 3.7+
- MongoDB server (ensure the collection `bingus bongus` is configured)
- Gazebo for 3D simulation
- Dependencies listed in `requirements.txt` (e.g., PyTorch, OpenCV, YOLOv11 dependencies)

### Installation

1. **Clone the repository:**

   ```bash
   git clone https://github.com/yourusername/traffic-analysis-tool.git
   cd traffic-analysis-tool
   ```

2. **Install dependencies:**

   ```bash
   pip install -r requirements.txt
   ```

3. **Configure MongoDB:**

   Ensure your MongoDB instance is running and accessible. Update the configuration file with the correct connection string if necessary.

4. **Run the application:**

   ```bash
   python main.py
   ```

### Uploading Data

- **Single Image:**  
  Use the provided interface to upload an image.
- **Video Footage:**  
  Upload your dashcam or traffic camera video. The system automatically extracts frames at 3fps for processing.

---

## Contributing

Contributions are welcome! Please follow these guidelines:
- Fork the repository and create your branch from `main`.
- Write clear and concise commit messages.
- Ensure your code adheres to the project's coding standards.
- Submit a pull request with a detailed description of your changes.

For any issues or feature requests, please open an issue in the GitHub repository.

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

*Developed with passion by boga*

Feel free to explore, contribute, and help improve traffic safety and simulation fidelity!
