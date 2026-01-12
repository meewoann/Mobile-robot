# ROS 2 Humble – Lane Following & Ball Grasping Mobile Robot

This project implements a **mobile robot** that can **follow a lane** and **detect / grasp a ball** using **ROS 2 Humble**. The system follows a classic robotics pipeline: perception → decision → actuation, combining **deep learning** and **traditional computer vision**, keeping things reliable and easy to debug.

---

## 🚀 Features

* **Lane Following (Deep Learning)**

  * Lane segmentation using a deep learning model
  * Extracts lane center and computes lateral error
  * Designed for real-time navigation

* **Ball Detection (Classical CV)**

  * Color-based detection in **HSV color space**
  * Color thresholding + contour extraction
  * Ball selected as the **largest contour by area**

* **ROS 2 Humble Compatible**

  * Modular node-based architecture
  * Camera image shared via a common topic

---

## 🧠 System Architecture

```
Camera Reader
   └── publishes /image/image_raw
        ├── lane_detect_node
        │     └── Deep Learning Lane Segmentation
        │
        └── object_detection_node
              └── HSV Color Threshold + Contour Detection
```

All perception nodes subscribe to the same camera topic for synchronized processing.

---

## 📦 Nodes Overview

### 1️⃣ Lane Detection Node

* **Purpose**: Detect and segment the lane for robot navigation
* **Method**: Deep Learning–based semantic segmentation
* **Input Topic**:

  * `/image/image_raw`
* **Output**:

  * Lane mask
  * Lane center position
  * Lateral error for control

**Run command:**

```
ros2 run lane_detect lane_detect_node.py
```

---

### 2️⃣ Ball Detection Node

* **Purpose**: Detect a ball for grasping

* **Method**:

  1. Convert RGB image → HSV
  2. Apply color thresholding
  3. Find contours
  4. Select the contour with the **largest area** as the ball

* **Input Topic**:

  * `/image/image_raw`

* **Output**:

  * Ball position in image frame
  * Visualized detection (circle / center point)

**Run command:**

```
ros2 run object_detection object_detection_node.py
```

---

### 3️⃣ Bringup Node

* **Purpose**: Launch the entire robot system at once
* **Includes**:

  * Camera reader
  * Lane detection
  * Ball detection
  * Other required system nodes

**Run command:**

```
ros2 launch bringup system.launch.py
```

---

## 📷 Camera Interface

* **Topic**: `/image/image_raw`
* **Message Type**: `sensor_msgs/Image`
* All perception nodes subscribe to this topic

---

## 🛠 Dependencies

* ROS 2 Humble
* OpenCV
* NumPy
* Deep Learning framework (PyTorch / TensorFlow – depending on lane model)
* cv_bridge

---

## 🧪 Visualization

* Lane segmentation mask with center point
* Ball detection with bounding circle and centroid
* Useful for debugging and tuning thresholds

---

## 📌 Design Philosophy

* **Lane detection** uses deep learning for robustness against lighting and noise
* **Ball detection** uses classical CV for speed, simplicity, and explainability
* Old-school vision where it works, AI where it matters 😎

---

## 📄 License

This project is for **research and educational purposes**.

---

## ✨ Author

Developed by an AI & Robotics Engineering student using ROS 2 Humble.

If it moves, sees, and grabs — it’s robotics the traditional way 🤖
