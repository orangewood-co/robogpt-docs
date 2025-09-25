# RoboGPT Perception

A practical vision stack for RoboGPT that brings up cameras, runs object detection and segmentation, provides world-context reasoning, finds ArUco marker poses, and can generate object point clouds. It also includes an AutoTrain workflow to capture data, annotate on the fly, and retrain the detector.

---

## **Quick start**

1) Source your ROS 2 workspace (skip if already done).
2) Plug in your camera (Intel RealSense or Orbbec recommended).
3) Launch the full perception stack:

```bash
ros2 launch robogpt_perception vision_bringup.launch.py robot_name:=ec612
```

**What this starts for you:**
- Vision startup (handles cameras and object detection/context nodes)
- Pose calculator service (image to base-frame conversion)
- Segmentation service (Used within pose calculator for accurate rotation)
- AutoTrain action server (Train a new object/model on the go)
- ArUco pose service (Machine calibration)
- Web feed publisher (Publishes the camera feed to the webapp)

> Tip: You can visualize image topics with rqt_image_view or RViz.

---

## **Camera setup (recommended before launching)**

Use the guided setup to register cameras and their roles (detection/context) and to populate the configuration file used by the stack.

- Run the setup:
  ```bash
  ros2 run robogpt_perception camera_setup.py
  ```

**What it does**
- Creates/updates `vision_config/camera_params.yaml` with:
  - number_of_cams, number_of_detection_cameras, number_of_context_cameras
  - Per-camera entries: `camera_N`, `serial_no_N`, `type_N` (realsense|orbbec|rgb)
  - Purpose mappings: `detection_cam_N` + `detection_type_N`, `context_cam_N` + `context_type_N`
- Internally uses helpers to discover and configure different camera types (RealSense, Orbbec, RGB webcams).

> You can re-run this anytime; it will start from a clean file each run.

---

## **Launch files (how and when to use them)**

**Use these when you want to run just part of the stack or customize inputs.**

- **vision_startup.py**
	- Orchestrates camera bringup and launches detection/context nodes based on `vision_config/camera_params.yaml`.
	- Reads camera types and names, starts RealSense/Orbbec/RGB pipelines, and kicks off detection/context nodes.
	- **Example** (typically started via the bringup, but can be run directly):
		```bash
		ros2 run robogpt_perception vision_startup.py
		```

- **custom_realsense.launch.py**
	- Brings up a RealSense camera with depth aligned and pointcloud enabled.
	- Topics end up under /<camera_name>/...
	- **Args:** camera_name (default: camera), serial_number (optional)
	- **Example:**
		```bash
		ros2 launch robogpt_perception custom_realsense.launch.py \
			camera_name:=camera serial_number:="<YOUR_RS_SERIAL>"
		```

- **detection_bringup.launch.py**
	- Starts an object detection node that subscribes to your camera topics.
	- **Args:** camera_name, type, node_name, image_topic, depth_topic, index
	- **Example** (RealSense-style topics remapped to /camera_name/...):
		```bash
		ros2 launch robogpt_perception detection_bringup.launch.py \
			camera_name:=camera \
			image_topic:=/camera/color/image_raw \
			depth_topic:=/camera/depth/image_raw \
			index:=1
		```

- **world_context_bringup.launch.py**
	- Starts the World Context service and a context image publisher.
	- **Args:** camera_name, type, node_name, image_topic, index
	- **Example:**
		```bash
		ros2 launch robogpt_perception world_context_bringup.launch.py \
			camera_name:=camera \
			image_topic:=/camera/color/image_raw \
			index:=1
		```

> **Note**:

- The full bringup (vision_bringup.launch.py) already starts the services you need. Use the others for custom flows or troubleshooting.
- RealSense topics are remapped to: /<camera_name>/color/image_raw, /<camera_name>/depth/image_raw, /<camera_name>/color/camera_info, and pointcloud at /<camera_name>/depth/color/points.

---

**Run services independently (optional)**
- Start only the segmentation service:
  ```bash
  ros2 run robogpt_perception segmentation.py
  ```
- Start only the pose calculator service:
  ```bash
  ros2 run robogpt_perception pose_calculator.py
  ```
- Start only the ArUco pose service:
  ```bash
  ros2 run robogpt_perception aruco.py
  ```
- Start only the world context service:
  ```bash
  ros2 run robogpt_perception world_context_agent.py
  ```

> Not recommended to run the services independetly in this manner because they take certain parameters as input which are defined when the services are started via the launch file.

---

## **Services (what they do and how to call them)**

Run these commands in a terminal. Ensure the relevant node/service is running (via vision_bringup or by launching/running the node mentioned above).

- **Pose calculator (image to base-frame pose)**
	- **Service name:** /pose_calculator
	- **What it does:** Converts the selected object’s image-frame coordinates into the robot base frame (x, y, z, roll, pitch, yaw) using camera intrinsics and known transforms. Uses detection results stored by the detector.
	- **Params:**
		- parent_frame: Target frame for the pose (e.g., base_link)
		- object_name: The object label to compute pose for (must exist in the latest detections)
		- camera_name: Name of the camera (e.g., camera)
	- **Example call:**
		```bash
		ros2 service call /pose_calculator robogpt_perception/srv/PoseCalculator \
			"{parent_frame: 'base_link', object_name: 'book', camera_name: 'camera'}"
		```

- **Segmentation (returns a binary mask for the current detection)**
	- **Service name:** /segment
	- **What it does:** Generates a segmentation mask for the currently detected object(s) using the local SAM model by default.
	- **Params:**
		- camera_name: Name of the camera (e.g., camera)
		- use_local_compute: Whether to use local SAM or cloud API (default: false recommended here)
	- **Example call:**
		```bash
		ros2 service call /segment robogpt_perception/srv/Segment \
			"{camera_name: 'camera', use_local_compute: false}"
		```

- **ArUco pose detection (writes latest marker poses, returns status)**
	- **Service name:** /get_aruco_pose
	- **What it does:** Detects 5x5 ArUco markers in the current frame, estimates their poses, transforms them into base_link, and saves to a JSON file.
	- **Params:**
		- camera_name: Name of the camera (e.g., camera)
	- **Example call:**
		```bash
		ros2 service call /get_aruco_pose robogpt_perception/srv/GetArucoPose \
			"{camera_name: 'camera'}"
		```

- **World context (GPT-based image analysis; set OPENAI_API_KEY before using)**
	- **Service name:** /analyze_image
	- **What it does:** Sends the current image to a VLM for answering prompts, optional OCR, and simple image comparisons.
	- **Params:**
		- camera_name: Name of the camera (e.g., camera)
		- prompt: A natural language question/instruction about the scene
		- perform_ocr: Whether to extract text in the image
		- compare_images: Compare current frame with a stored image (by name)
		- image1_name: The name/ID of the reference image to compare
	- **Example call:**
		```bash
		ros2 service call /analyze_image robogpt_perception/srv/WorldContext \
			"{camera_name: 'camera', prompt: 'Describe the scene', perform_ocr: false, compare_images: false, image1_name: ''}"
		```

---

## **AutoTrain (capture ➜ annotate ➜ train)**

AutoTrain runs on a cloud GPU by default and is started as a server by the bringup. Use the client to trigger a run.

- **Run the client**
  ```bash
  ros2 run robogpt_perception autotrain_client.py
  ```

**Client arguments (edit inside the client script if needed)**
- base_url: Cloud server URL (default in the script)
- camera_name: e.g., "camera" (images taken from /<camera_name>/color/image_raw)
- combined_folder: local folder name to organize this run’s data (default: auto_train_data/train_vTIMESTAMP)
- object_name: free-text name, used for prompts/annotations
- object_label: class label used for training (e.g., book)
- image_threshold: number of images to collect/annotate
- number_aug: augmentation count used by the cloud pipeline
- epochs: training epochs
- map_threshold, box_threshold, text_threshold: thresholds for annotation/quality gates

**What happens**
- The client captures frames, requests annotations, and triggers training in the cloud. When complete, the new weights are saved locally and picked up automatically by the detector on the next cycle.

---

>    ## **Tips and troubleshooting**

- Camera topics
    - RealSense bringup here remaps topics under /<camera_name>/. If you use your own camera launch, be sure to pass image/depth topics to detection_bringup accordingly.

- Permissions
    - On Linux, if /dev/video* devices aren’t readable, add your user to the video group and re-login.

- World context
    - Requires an OpenAI API key in the environment: export OPENAI_API_KEY=... before launching.

- Visualization
    - Check detection outputs on `/detection_feed_<index>` and context feed on `/context_feed_<index>`.

If you get stuck, share the exact command and terminal output so we can help quickly.