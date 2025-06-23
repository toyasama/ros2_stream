# Real-Time Video Processing Pipeline (ROS 2 + Docker)

This project implements a real-time video processing pipeline using ROS 2 and Docker. It captures video from a camera, processes it with OpenCV (filters and overlays), and streams the result via RTSP.

##  Easiest Way to Run (Using Docker Compose)

This is the recommended and quickest way to get everything running.

### ✅ Prerequisites

- Docker
- Docker Compose v1 or v2
- Access to a webcam device (`/dev/video0`)
- X11 support on host (for GUI apps, e.g. image viewers)

> 🛑 Make sure your user is part of the `docker` group or use `sudo` when needed.

---

### 1. Clone the Repository

```bash
git clone https://github.com/toyasama/ros2_stream.git
cd ros2_stream/compose
```


### 2. Launch the Pipeline
Simply run the provided script to build and launch everything:

```bash
./compose_run.sh
```

This script will:

- Build the Docker image using the Dockerfile

- Start two ROS 2 containers:

    - ros2_stream_launcher: starts the full ROS 2 node graph (video capture, processing, and RTSP server)

    - ros2_stream_viewer: waits for the video stream topic and then launches a viewer node

### Trouble shouting

- Error :```listen tcp :8554: bind: address already in use```
  - This means the RTSP port (8554) is already occupied, likely by another instance of MediaMTX or another RTSP server.

  - To resolve:
    - Stop any running service using port 8554:
      ```
      sudo lsof -i :8554
      sudo kill <PID>
      ```
    - Then relaunch the ROS 2 pipeline.

Stop any running service using port 8554:



## 🛠️ Manual Installation (Hard Way)

> **Note:** This method is for advanced users who want to build and run the project natively, without Docker.  
> You are responsible for managing all dependencies and system setup.

---

### ✅ Prerequisites

Ensure the following tools and libraries are installed and configured:

- ROS 2 Humble (or compatible)
- `colcon` (build tool for ROS 2)
- `rosdep` (dependency manager)
- `FFmpeg` (for RTSP streaming)
- `OpenCV` (with V4L2 and X11 support)
- `cyclonedds` (or compatible DDS implementation)
- `MediaMTX` (formerly `rtsp-simple-server`)
- Access to a physical camera at `/dev/video0`

Make sure your environment is sourced:

```bash
source /opt/ros/humble/setup.bash
```

### 1. Clone the Repository

```bash
git clone https://github.com/toyasama/ros2_stream.git
cd ros2_stream
```

### 2: Set Up the ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ln -s /absolute/path/to/ros2_stream .
```


```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 3: Launch the ROS 2 Nodes

Open two terminals, and in each:

Terminal 1: Launch the processing pipeline
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros_stream ros_stream_launch.py
```

wait for the server to be launch and in
Terminal 2: Launch the stream viewer

```
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros_stream show_launch.py
```

### Project Structure (High-Level)

```graphql
.
├── compose/                # Docker setup (compose, Dockerfile, launch script)
├── src/
│   ├── video_capture/      # Captures webcam input
│   ├── video_processing/   # Applies filters and overlays (heatmap, text)
│   └── ros_stream/         # ROS 2 launcher, includes RTSP server config
├── cyclonedds.xml          # ROS 2 DDS configuration
├── setup.bash              # Environment setup script
└── read_me.md              # You're here!
```

## Technical Overview

### Architecture: Modular ROS 2 Nodes

- **video_capture**  
  Captures frames from the webcam and publishes them to a ROS 2 topic.

- **video_processing**  
  Applies OpenCV-based transformations such as:
  - Heat map filter  
  - Text overlay (static or dynamic)

- **ros_stream**  
  Orchestrates the node launch using a `ComposableNodeContainer`, and streams the processed output via MediaMTX (RTSP).

### Latency Target

- **Estimated Total Latency**: ~32–79 ms  
  Suitable for near real-time video visualization.

### Streaming Protocol

- **RTSP**  
  Chosen for its:
  - Low latency
  - Broad compatibility (works with VLC, ffplay, etc.)

---

## 📄 License

This project is licensed under the terms of the [LICENSE](../LICENSE) file.
