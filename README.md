
---

# ORB-SLAM3 for ROS 2
## this repo is part of my summer internship repositories that can be found [here ](https://github.com/Abinpramudya/BlueROV2025)

This is a slightly modified version of the code from this [repository](https://github.com/doeswork/EASY-ORB-SLAM3/tree/master/src/orbslam3_ros2). They provide an excellent demonstration in this [video](https://www.youtube.com/watch?v=CRg8zmCNOAU).

## My Setup

* ROS 2 Humble
* Ubuntu 22.04
* OpenCV 4.2.0

This repository aims to simplify the integration of ORB-SLAM3 with the **BlueROV camera**, with a revised README for easier setup.

---

## First-Time Installation

This section is intended for users who have never installed ORB-SLAM on their machine before. Several dependencies must be installed before building ORB-SLAM3.

### 1. OpenCV

The original ORB-SLAM requires at least OpenCV 3.3.0, but this repository specifically depends on **OpenCV 4.2.0**.

**Installation via APT (simpler):**

```bash
sudo apt update
sudo apt install libopencv-dev python3-opencv
```

**Alternatively, build OpenCV 4.2.0 from source:**

```bash
sudo apt install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 4.2.0
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

---

### 2. Pangolin

Pangolin is the GUI used by ORB-SLAM3 to visualize SLAM results.

```bash
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
cmake --build .
sudo make install
```

---

### 3. Eigen

Eigen is a linear algebra library used internally by ORB-SLAM3.

**Option 1: Install via APT**

```bash
sudo apt install libeigen3-dev
```

**Option 2: Install manually**

```bash
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz
tar -xvzf eigen-3.3.7.tar.gz
cd eigen-3.3.7
mkdir build && cd build
cmake ..
sudo make install
```

---

## Repository Installation

Once the dependencies are ready, install the **modified ORB-SLAM3** repository. You do **not** need the original UZ-SLAM version.

### Steps:

1. Clone the stereo-fixed ORB-SLAM3 repository:
   → [Stereo-Fixed ORB-SLAM3 Repository](https://github.com/zang09/ORB-SLAM3-STEREO-FIXED)

```bash
git clone https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git ORB_SLAM3
```

It’s recommended to place this near your home directory for easier access (e.g., `/home/abin/Documents/mir/SLAM/`).

2. Build the repository:

```bash
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

3. Install the **Sophus** library:

```bash
cd ORB_SLAM3/Thirdparty/Sophus/build
sudo make install
```

⚠️ **Important:** Make sure everything is built with **OpenCV 4.2.0**.

---

## Final Setup Steps

Now that ORB-SLAM3 is built, clone this repository and modify a few files:

### 1. Update `PYTHONPATH`

Edit the `CMakeLists.txt` of this repository:

```cmake
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/")
```

Replace this with your actual Python site-packages path. You can check with:

```bash
python3 -m site
```

---

### 2. Set ORB-SLAM3 Root Directory

In `CMakeModules/FINDORB_SLAM3.cmake`, update the following:

```cmake
set(ORB_SLAM3_ROOT_DIR "/home/abin/Documents/mir/SLAM/ORB-SLAM3-STEREO-FIXED")
```

Replace the path with your local path to the stereo-fixed ORB-SLAM3 repository.

---

### 3. Build Inside Your ROS 2 Workspace

After placing this repository inside your ROS 2 workspace (e.g., `slam_ws/src/this_repo`):

```bash
cd ~/slam_ws
colcon build
source install/setup.bash
```

### 4. Verify Installation

Check that the package is recognized:

```bash
ros2 pkg list | grep orb
```

Expected output:

```bash
orbslam3
```

---

## Topic Configuration

Before running the system, you must ensure that the image topics used by your ROS camera node match those expected by ORB-SLAM3.

### Files to Modify

#### 1. **Monocular Mode**

File: `src/monocular/monocular-slam-node.cpp`

```cpp
m_image_subscriber = this->create_subscription<ImageMsg>(
    "camera/left",  // <- Change this to match your actual image topic
```

#### 2. **Stereo Mode**

File: `src/stereo/stereo-slam-node.cpp`

```cpp
left_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(shared_ptr<rclcpp::Node>(this), "camera/left");
right_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(shared_ptr<rclcpp::Node>(this), "camera/right");
```

Ensure these topic names match those published by your camera drivers.

---

## Running ORB-SLAM3

Once the topics are aligned, you can launch ORB-SLAM3 in one of the supported modes:

> ⚠️ Supported modes: `MONO`, `STEREO`, `RGBD`, `STEREO-INERTIAL`

You will need:

* A **vocabulary file** (e.g., `orbslam3_ros2/vocabulary/ORBvoc.txt` extract it from the zip file)
* A **config file** (e.g., `orbslam3_ros2/config/monocular/TUM1.yaml`)

> 📌 Make sure your config file is properly calibrated for your camera.
> Refer to my [camera calibration package](https://github.com/Abinpramudya/BlueROV2025/tree/main/cam_cal) for help!

---

### Launch Commands

* **Monocular (MONO):**

```bash
ros2 run orbslam3 mono PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
```

* **Stereo:**

```bash
ros2 run orbslam3 stereo PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY
```

* **RGB-D:**

```bash
ros2 run orbslam3 rgbd PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
```

* **Stereo-Inertial:**

```bash
ros2 run orbslam3 stereo-inertial PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY [BOOL_EQUALIZE]
```

> Replace each `PATH_TO_*` with the actual path on your system.
> Boolean flags like `BOOL_RECTIFY` and `BOOL_EQUALIZE` should be set to `true` or `false`.

---

## Acknowledgements

This project is built on the shoulders of several incredible open-source contributions. Special thanks to:

1. **UZ-SLAM Lab** – Original ORB-SLAM3
   → [ORB-SLAM3 GitHub Repository](https://github.com/UZ-SLAMLab/ORB_SLAM3)

2. **Haebeom Jung** – Stereo-fixed ORB-SLAM3
   → [Stereo-Fixed ORB-SLAM3 Repository](https://github.com/zang09/ORB-SLAM3-STEREO-FIXED)

3. **doeswork** – ROS 2 wrapper and integration
   → [EASY-ORB-SLAM3 (ROS 2) Repository](https://github.com/doeswork/EASY-ORB-SLAM3/tree/master/src/orbslam3_ros2)

---

### Special Thanks

To my fellow **MIR colleagues** for their support and collaboration:

* **Mahmoud Aboelrayat**
  → [GitHub Profile](https://github.com/MahmoudAboelrayat)

* **Anastasiia Frolova**
  → [GitHub Profile](https://github.com/anastasiiafrolova211)

---
