### Quick useful commands
- get short 3 level depth repo structure
```text
tree -a -L 3 -I '.git|__pycache__|*.pyc'
```
- Enter Docker for SVO
```bash
./scripts/enter_svo_docker.sh
```

## le 18 mai 2026
- considering how SVO needs docker, while DA3 is a python project i recommend a three-layer architecture:
```text
1. SVO Docker container
   Input: RGB frames / ROS bag
   Output: svo_traj_raw.txt

2. DA3 Python environment
   Input: same RGB frames
   Output: depth_da3/*.npy

3. Fusion Python environment
   Input: RGB + depth .npy + trajectory.txt + intrinsics.yaml
   Output: .ply point clouds + .txt trajectory
```

- cloned DA3 successfuly:
```bash
cd ~/Documents/GitHub/masters-internship-svo-da3-fusion

git clone https://github.com/ByteDance-Seed/depth-anything-3.git external/Depth-Anything-3
```
```text
DA3 needs:
requires-python = ">=3.9, <=3.13"
torch>=2
numpy<2
xformers
open3d
pycolmap
```

- moved to new machine, unless said otherwise, all future progress is done under these params:
```text
RAM: 32 GB
GPU: RTX 4070
VRAM: 12 GB
Driver: 570.211.01
CUDA reported by driver: 12.8
```
- create missing folders 
```bash
mkdir -p external env configs examples experiments/sequence_001
touch external/.gitkeep env/.gitkeep configs/.gitkeep examples/.gitkeep experiments/sequence_001/.gitkeep
```
```bash
git clone https://github.com/ByteDance-Seed/depth-anything-3.git external/Depth-Anything-3
```
- create yaml for conda venv config:
```yaml
name: da3
channels:
  - pytorch
  - nvidia
  - conda-forge
dependencies:
  - python=3.10
  - pip
  - pytorch=2.3.1
  - torchvision
  - torchaudio
  - pytorch-cuda=12.1
  - numpy<2
```
then
```bash
conda env create -f env/environment_da3.yml
conda activate da3
```
then install DA3
```bash
pip install -e external/Depth-Anything-3
```

- after many kernel problems, we setteled for:
```text
Kernel: 6.8.0-117-generic
GPU: RTX 4070 visible
Driver: 580.142
CUDA shown by driver: 13.0
Ethernet: enp5s0 connected
```

test DE3
```bash
cd ~/Documents/Diyari_M_salih_2026/masters-internship-svo-da3-fusion
conda activate da3

python - <<'PY'
import torch
print("Torch:", torch.__version__)
print("Torch CUDA build:", torch.version.cuda)
print("CUDA available:", torch.cuda.is_available())
print("Device count:", torch.cuda.device_count())
if torch.cuda.is_available():
    print("GPU:", torch.cuda.get_device_name(0))
PY
```

confirm CUDA + DE3:
```bash
python - <<'PY'
import torch
import depth_anything_3

print("Depth Anything 3 import OK")
print("CUDA available:", torch.cuda.is_available())
print("GPU:", torch.cuda.get_device_name(0))
PY
```
```bash
Depth Anything 3 import OK
CUDA available: True
GPU: NVIDIA GeForce RTX 4070
```
## Working machine setup

- Machine: lab PC `qcartech`
- Kernel used for GPU work: `6.8.0-117-generic`
- GPU: NVIDIA GeForce RTX 4070, 12 GB VRAM
- NVIDIA driver: 580.142
- Conda env: `da3`
- PyTorch: 2.3.1+cu121
- Torch CUDA build: 12.1
- CUDA available in PyTorch: yes

Important: kernel `6.8.0-111-generic` did not have a working NVIDIA module. Use `6.8.0-117-generic` for DA3/GPU work.

Testing DA3
```bash
mkdir -p data_local/test_images outputs/da3_test
```
```bash
python - <<'PY'
import cv2
import numpy as np
from pathlib import Path

Path("data_local/test_images").mkdir(parents=True, exist_ok=True)

h, w = 480, 640
img = np.zeros((h, w, 3), dtype=np.uint8)
img[:, :, 0] = np.linspace(0, 255, w, dtype=np.uint8)
img[:, :, 1] = np.linspace(0, 255, h, dtype=np.uint8)[:, None]
img[:, :, 2] = 120

cv2.imwrite("data_local/test_images/test_000.png", img)
print("Saved data_local/test_images/test_000.png")
PY
```
```bash
da3 image data_local/test_images/test_000.png \
  --export-dir outputs/da3_test \
  --export-format mini_npz \
  --device cuda \
  --process-res 504 \
  --auto-cleanup
```
## DA3 CLI local patch

During the first `da3 image ... --export-format mini_npz` test, the DA3 CLI failed because of an internal argument naming mismatch.

Observed errors:
- `NameError: name 'reference_view_strategy' is not defined`
- `TypeError: run_inference(## naming mismatch error:) got an unexpected keyword argument 'reference_view_strategy'`

Cause:
- `cli.py` defines the CLI option as `ref_view_strategy`.
- `run_inference()` also expects `ref_view_strategy`.
- Some CLI calls incorrectly passed `reference_view_strategy`.

Local fix:
- In `external/Depth-Anything-3/src/depth_anything_3/cli.py`, replaced the incorrect keyword usage with:
  `ref_view_strategy=ref_view_strategy`

Status:
- After this patch, `da3 image ... --export-format mini_npz` successfully exported:
  - `depth.npy`
  - `conf.npy`
  - `intrinsics.npy`
  - `extrinsics.npy`

Note:
- This is a local patch inside the external DA3 clone. Before reporting upstream, verify whether the bug still exists on the latest official commit.

## DA3 depth-to-PLY sanity test

Converted DA3 `mini_npz` output into a colored point cloud.

Input:
- RGB: `data_local/test_images/test_000.png`
- Depth: `outputs/da3_test/exports/mini_npz/depth.npy`
- Intrinsics: `outputs/da3_test/exports/mini_npz/intrinsics.npy`

Output:
- `outputs/da3_test/test_000_da3_cloud.ply`

Result:
- Points: 11970
- DA3 processed resolution: 504x378
- PLY size: 458K

Interpretation:
- The DA3 depth and intrinsics are usable for 3D back-projection.
- This test does not yet use SVO poses.
- Next fusion milestone is to replace identity/DA3-only camera frame with SVO camera poses.

## cloned SVO:
```bash
cd ~/Documents/Diyari_M_salih_2026/masters-internship-svo-da3-fusion

git clone https://github.com/uzh-rpg/rpg_svo_pro_open.git external/rpg_svo_pro_open
```
```bash
mkdir -p patches/svo
cp external/rpg_svo_pro_open/dependencies.yaml patches/svo/dependencies_https.yaml

python - <<'PY'
from pathlib import Path

p = Path("patches/svo/dependencies_https.yaml")
text = p.read_text()
text = text.replace("git@github.com:", "https://github.com/")
text = text.replace(".git", ".git")
p.write_text(text)

print(p.read_text())
PY
```

- create the main docker file in dockerfile.svo_noetic
```bash
sudo docker build -f docker/Dockerfile.svo_noetic -t svo-noetic-base .
```
- test
```bash
sudo docker run --rm -it svo-noetic-base bash -lc "source /opt/ros/noetic/setup.bash && rosversion -d && catkin --version"
```
- output
```bash
ROS: noetic
catkin_tools: 0.9.4
Python: 3.8.10
```

## create an SVO workspace inside the mounted repo, but do not build yet.

Run this from the host repo root:
```bash
mkdir -p svo_ws/src
cp -r external/rpg_svo_pro_open svo_ws/src/
cp patches/svo/dependencies_https.yaml svo_ws/src/dependencies_https.yaml
```
Then start the container again:
```bash
sudo docker run --rm -it \
  -v "$PWD":/workspace/project \
  svo-noetic-base \
  bash
```
Inside the container:
```bash
source /opt/ros/noetic/setup.bash
cd /workspace/project/svo_ws

catkin config --init --mkdirs --extend /opt/ros/noetic \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3

cd src
vcs-import < dependencies_https.yaml
touch minkindr/minkindr_python/CATKIN_IGNORE
```
Then check:
```bash
ls
```
```text
DA3:
  working in Conda env da3
  CUDA works on RTX 4070
  mini_npz export works
  depth → PLY sanity test works

SVO:
  Docker ROS Noetic base works
  workspace imported successfully
  all 34 SVO packages built successfully
```
```text
note: dbow2_catkin was trying to clone:
git@github.com:dorian3d/DBoW2.git

We patched it to:
https://github.com/dorian3d/DBoW2.git
```

## SVO Docker build success

SVO Pro was built successfully inside the ROS Noetic Docker environment.

Environment:
- Docker image: `svo-noetic-base`
- Base: `osrf/ros:noetic-desktop-full`
- Workspace: `svo_ws`
- Build command: `catkin build`
- Result: all 34 packages succeeded

Issues fixed:
- Added `autoconf`, `automake`, and `libtool` to the Dockerfile because `glog_catkin` needed `libtoolize`.
- Patched `dbow2_catkin` internal clone URL from SSH to HTTPS because Docker did not have GitHub SSH credentials.

Next:
- Save the DBoW2 HTTPS patch reproducibly.
- Test launching/locating SVO ROS nodes.

## SVO ROS package verification

After building SVO in the Noetic Docker container, the workspace was sourced with:

```bash
source /workspace/project/svo_ws/devel/setup.bash

rospack find svo_ros
# /workspace/project/svo_ws/src/rpg_svo_pro_open/svo_ros

rospack find svo
# /workspace/project/svo_ws/src/rpg_svo_pro_open/svo

rospack find svo_msgs
# /workspace/project/svo_ws/src/rpg_svo_pro_open/svo_msgs
```

## SVO runtime sanity check

Created and launched a minimal no-RViz SVO launch file:

```bash
roslaunch /workspace/project/configs/svo_launch/euroc_mono_no_rviz.launch
```
With roscore running in another Docker shell, the SVO node started successfully.

Verified nodes:

rosnode list
```
# /rosout
# /svo
```
Verified topics:
```
rostopic list
# /cam0/image_raw
# /imu0
# /svo/pose_cam/0
# /svo/pose_imu
# /svo/pointcloud
# /svo/keyframes
# /svo/info
# ...
```
Status:

SVO runtime environment is valid.
The node launches without RViz.
It is ready for a real input stream/bag/image publisher.

download rosbag for sanity test:
```bash
mkdir -p data_local/euroc
cd data_local/euroc

wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag
```
or official page link https://projects.asl.ethz.ch/datasets/euroc-mav/

with data set extracted, testing svo:
roscore
```bash
./scripts/enter_svo_docker.sh
source /opt/ros/noetic/setup.bash
source /workspace/project/svo_ws/devel/setup.bash
roscore
```
svo node
```bash
./scripts/enter_svo_docker.sh
source /opt/ros/noetic/setup.bash
source /workspace/project/svo_ws/devel/setup.bash

roslaunch /workspace/project/configs/svo_launch/euroc_mono_no_rviz.launch
```
bag
```bash
./scripts/enter_svo_docker.sh
source /opt/ros/noetic/setup.bash
source /workspace/project/svo_ws/devel/setup.bash

rosbag play /workspace/project/data_local/euroc/machine_hall/MH_01_easy/MH_01_easy.bag \
  --clock \
  -r 0.5
```
record the SVO pose output

While roscore, SVO, and rosbag play are running, open one more host terminal:
```bash
cd ~/Documents/Diyari_M_salih_2026/masters-internship-svo-da3-fusion
./scripts/enter_svo_docker.sh
source /opt/ros/noetic/setup.bash
source /workspace/project/svo_ws/devel/setup.bash

mkdir -p /workspace/project/outputs/svo_mh01

rosbag record -O /workspace/project/outputs/svo_mh01/svo_pose_cam.bag \
  /svo/pose_cam/0 \
  /svo/pose_imu \
  /svo/info
```
generate tum format:
```bash
cd /workspace/project
source /opt/ros/noetic/setup.bash
source /workspace/project/svo_ws/devel/setup.bash

python3 scripts/rosbag_pose_to_tum.py \
  --bag outputs/svo_mh01/svo_pose_cam.bag \
  --topic /svo/pose_cam/0 \
  --output outputs/svo_mh01/svo_pose_cam_tum.txt
```

## SVO trajectory export test

Ran SVO on EuRoC `MH_01_easy.bag` and recorded pose output.

Recorded bag:
- `outputs/svo_mh01/svo_pose_cam.bag`

Topics:
- `/svo/pose_cam/0`: 702 messages
- `/svo/pose_imu`: 702 messages
- `/svo/info`: 702 messages

Converted `/svo/pose_cam/0` to TUM trajectory format:

- `outputs/svo_mh01/svo_pose_cam_tum.txt`

Result:
- 702 poses
- 703 lines including header
- Format: `timestamp tx ty tz qx qy qz qw`

Status:
- SVO can process EuRoC MH_01_easy and export a usable trajectory file.
- Next step is to extract the matching RGB frames from the same EuRoC bag for DA3.


## le 18 mai 2026
- using extract_rosbag_images.py get 100 frames
```bash
cd /workspace/project
source /opt/ros/noetic/setup.bash
source /workspace/project/svo_ws/devel/setup.bash

python3 scripts/extract_rosbag_images.py \
  --bag data_local/euroc/machine_hall/MH_01_easy/MH_01_easy.bag \
  --topic /cam0/image_raw \
  --output_dir outputs/euroc_mh01/cam0_rgb \
  --timestamps outputs/euroc_mh01/cam0_timestamps.csv \
  --max_frames 100 \
  --every_n 5
```
- convert to 16 since 100 frames in one tensor can lead to OOM, also consistant with:

- PCL1 = 1–16 keyframes
- PCL2 = 17–32 keyframes
- PCL3 = 33–48 keyframes
```bash
mkdir -p outputs/euroc_mh01/cam0_rgb_016

cp outputs/euroc_mh01/cam0_rgb/frame_0000{00..15}.png \
   outputs/euroc_mh01/cam0_rgb_016/
```
- run DA3
```bash
da3 images outputs/euroc_mh01/cam0_rgb_016 \
  --export-dir outputs/euroc_mh01/da3_016 \
  --export-format mini_npz \
  --device cuda \
  --process-res 504 \
  --auto-cleanup
```

output:
```text
PCL1 = 16 frames
DA3 input tensor = [16, 3, 322, 504]
Model forward = ~3.08 seconds
Export = mini_npz successful

results.npz
  depth:      (16, 322, 504)
  conf:       (16, 322, 504)
  extrinsics: (16, 3, 4)
  intrinsics: (16, 3, 3)

```

Important observation: DA3 changed the processed image size to 504 × 322

### rerecorded SVO for the first 30s to match the first 100 frames of DA3 
```text
outputs/svo_mh01_clean/svo_pose_cam.bag
outputs/svo_mh01_clean/svo_pose_cam_tum.txt

575 poses
trajectory time range:
1403636580.063555479 → 1403636608.863555431
```

using the sync script, we try to match them in a csv:
```bash
python scripts/sync_frames_to_svo.py \
  --frames_csv outputs/euroc_mh01/cam0_timestamps.csv \
  --svo_tum outputs/svo_mh01_clean/svo_pose_cam_tum.txt \
  --output outputs/euroc_mh01/sync_016.csv \
  --max_frames 16 \
  --max_dt 0.20
```
output csv notes :
```text 
Only this row is bad:

frame 0 → dt = 0.300s

Everything from frame 2 onward is perfectly aligned, and frame 1 is acceptable:

frame 1 → dt = 0.05s
frame 2–15 → dt = 0.00s
```

Fuse DA3 depth chunk with SVO poses:
```bash
chmod +x scripts/make_pcl_chunk_from_da3_npz.py

python scripts/make_pcl_chunk_from_da3_npz.py \
  --rgb_dir outputs/euroc_mh01/cam0_rgb \
  --da3_npz outputs/euroc_mh01/da3_016/exports/mini_npz/results.npz \
  --sync_csv outputs/euroc_mh01/sync_016.csv \
  --output outputs/euroc_mh01/fusion/pcl_001_015.ply \
  --start_frame 1 \
  --end_frame 15 \
  --stride 6 \
  --max_dt 0.20
```
## current understanding
DA3 answers:
“For this image, what is the depth of each pixel?”

SVO answers:
“Where was the camera when this image was taken?”

Then fusion does:

pts_w = points placed into the global/world frame

SVO pose
→ move those local 3D points into a shared/world coordinate frame

in current pcl_001_015.ply, SVO is being used here:
```
T_w_c = pose_to_matrix(row)
pts_w = (T_w_c @ pts_c_h.T).T[:, :3]
```
Meaning:
```
pts_c = points in the camera frame from DA3 depth
T_w_c = SVO camera pose
pts_w = points placed into the global/world frame
```

DA3 creates the depth surface for each frame. SVO tells us where each frame belongs in 3D space. The fused point cloud exists because DA3 provides depth and SVO provides camera motion.

## Vicon room 1 sanity test, same setup:
folder structure 
```bash
masters-internship-svo-da3-fusion/data_local/euroc
├── machine_hall
│   ├── ._.DS_Store
│   ├── .DS_Store
│   ├── MH_01_easy
│   │   ├── ._.DS_Store
│   │   ├── .DS_Store
│   │   ├── MH_01_easy.bag
│   │   └── MH_01_easy.zip
│   ├── MH_02_easy
│   │   ├── MH_02_easy.bag
│   │   └── MH_02_easy.zip
│   ├── MH_03_medium
│   │   ├── MH_03_medium.bag
│   │   └── MH_03_medium.zip
│   ├── MH_04_difficult
│   │   ├── MH_04_difficult.bag
│   │   └── MH_04_difficult.zip
│   └── MH_05_difficult
│       ├── MH_05_difficult.bag
│       └── MH_05_difficult.zip
├── machine_hall.zip
├── vicon_room1
│   ├── ._.DS_Store
│   ├── .DS_Store
│   ├── V1_01_easy
│   │   ├── ._.DS_Store
│   │   ├── .DS_Store
│   │   ├── V1_01_easy.bag
│   │   └── V1_01_easy.zip
│   ├── V1_02_medium
│   │   ├── V1_02_medium.bag
│   │   └── V1_02_medium.zip
│   └── V1_03_difficult
│       ├── V1_03_difficult.bag
│       └── V1_03_difficult.zip
└── vicon_room1.zip
```

1. Run SVO clean recording on V1_01_easy

Using the same 4-terminal setup.

Terminal 1 — roscore
```bash
cd ~/Documents/Diyari_M_salih_2026/masters-internship-svo-da3-fusion
./scripts/enter_svo_docker.sh
source /opt/ros/noetic/setup.bash
source /workspace/project/svo_ws/devel/setup.bash
roscore
```
Terminal 2 — SVO
```
```bash
cd ~/Documents/Diyari_M_salih_2026/masters-internship-svo-da3-fusion
./scripts/enter_svo_docker.sh
source /opt/ros/noetic/setup.bash
source /workspace/project/svo_ws/devel/setup.bash
roslaunch /workspace/project/configs/svo_launch/euroc_mono_no_rviz.launch
```
Terminal 3 — record SVO poses first
```bash
cd ~/Documents/Diyari_M_salih_2026/masters-internship-svo-da3-fusion
./scripts/enter_svo_docker.sh
source /opt/ros/noetic/setup.bash
source /workspace/project/svo_ws/devel/setup.bash

mkdir -p /workspace/project/outputs/svo_v1_01_clean

rosbag record -O /workspace/project/outputs/svo_v1_01_clean/svo_pose_cam.bag \
  /svo/pose_cam/0 \
  /svo/pose_imu \
  /svo/info
```
Terminal 4 — play first 30 seconds of Vicon Room
```bash
cd ~/Documents/Diyari_M_salih_2026/masters-internship-svo-da3-fusion
./scripts/enter_svo_docker.sh
source /opt/ros/noetic/setup.bash
source /workspace/project/svo_ws/devel/setup.bash

rosbag play /workspace/project/data_local/euroc/vicon_room1/V1_01_easy/V1_01_easy.bag \
  --clock \
  -r 0.5 \
  --duration=30
```
When playback finishes, stop the recorder with Ctrl+C.

2. Convert SVO poses to TUM

Inside Docker:
```bash
cd /workspace/project
source /opt/ros/noetic/setup.bash
source /workspace/project/svo_ws/devel/setup.bash

python3 scripts/rosbag_pose_to_tum.py \
  --bag outputs/svo_v1_01_clean/svo_pose_cam.bag \
  --topic /svo/pose_cam/0 \
  --output outputs/svo_v1_01_clean/svo_pose_cam_tum.txt
```

extract cam0 frames V1_01_easy:
```bash
cd /workspace/project
source /opt/ros/noetic/setup.bash
source /workspace/project/svo_ws/devel/setup.bash

python3 scripts/extract_rosbag_images.py \
  --bag data_local/euroc/vicon_room1/V1_01_easy/V1_01_easy.bag \
  --topic /cam0/image_raw \
  --output_dir outputs/euroc_v1_01/cam0_rgb \
  --timestamps outputs/euroc_v1_01/cam0_timestamps.csv \
  --max_frames 100 \
  --every_n 5
```

frames to use 22-37:
```bash
cd ~/Documents/Diyari_M_salih_2026/masters-internship-svo-da3-fusion

rm -rf outputs/euroc_v1_01/cam0_rgb_022_037
mkdir -p outputs/euroc_v1_01/cam0_rgb_022_037

for i in $(seq -w 22 37); do
  cp outputs/euroc_v1_01/cam0_rgb/frame_000${i}.png \
     outputs/euroc_v1_01/cam0_rgb_022_037/
done
```
This will include frames 0–37, but our PLY script can use only 22–37.
```bash
python scripts/sync_frames_to_svo.py \
  --frames_csv outputs/euroc_v1_01/cam0_timestamps.csv \
  --svo_tum outputs/svo_v1_01_clean/svo_pose_cam_tum.txt \
  --output outputs/euroc_v1_01/sync_022_037.csv \
  --max_frames 38 \
  --max_dt 0.20
```

finally run:
```bash
python scripts/make_pcl_chunk_from_da3_npz.py \
  --rgb_dir outputs/euroc_v1_01/cam0_rgb \
  --da3_npz outputs/euroc_v1_01/da3_022_037/exports/mini_npz/results.npz \
  --sync_csv outputs/euroc_v1_01/sync_022_037.csv \
  --output outputs/euroc_v1_01/fusion/pcl_022_037.ply \
  --start_frame 22 \
  --end_frame 37 \
  --stride 6 \
  --max_dt 0.20 \
  --max_depth 10
```
output
```text
16 synchronized frames
4,536 points per frame
72,576 total points

much more, This Vicon result is much better. It is still raw and sparse, but now the room geometry is recognizable
```

- quality is still low on the 3D reconstruction:
```
DA3 demo:
DA3 depth + DA3 pose/extrinsics + DA3 confidence/export pipeline

Our current pipeline:
DA3 depth + SVO pose + raw back-projection + no scale alignment + no filtering
```

The current result tells us:
```
✅ DA3 depth works
✅ SVO pose stream works
✅ frame/pose sync works
✅ multi-frame PLY fusion works
⚠️ scale/alignment/filtering is still missing
```

## Pose convention diagnostic

Tested three pose sources for DA3 depth fusion:
- `identity`
- `svo`
- `da3`
- `da3_inv`

Result:
- Direct DA3 extrinsics produced smeared geometry.
- Inverting DA3 extrinsics produced the best reconstruction so far.
- This suggests DA3 `extrinsics` are not directly usable as `T_world_camera` for our backprojection convention; they should be inverted before use.
- SVO poses are active but still need scale/alignment before they match the DA3 depth scale.

Current best visual result:
- `outputs/euroc_v1_01/fusion/pcl_022_037_da3inv.ply`

Follow-up completed on 21 May 2026:
- Added scale/alignment diagnostics for SVO poses.
- Estimated Sim(3) between SVO trajectory and DA3-inverted trajectory over the matched frame chunk.
- Tested SVO Sim(3) per-frame fusion and DA3-inv chunk-level alignment.

## le 21 mai 2026

### Current fusion decision: DA3 local chunks + SVO anchoring

Visual comparison on EuRoC `V1_01_easy`, frames 22-37, showed:

- `pcl_022_037_da3inv.ply` is still the best local reconstruction.
- `pcl_022_037_svo.ply` is worse.
- `pcl_022_037_svo_sim3_da3inv.ply` aligns SVO camera centers toward DA3-inv centers, but still does not match DA3-inv visual quality.
- `pcl_022_037_da3inv_aligned_to_svo.ply` preserves the DA3-inv reconstruction quality while moving the whole finished chunk into the SVO trajectory frame.

Conclusion:

Use DA3 depth + DA3-inverted poses to build locally coherent reconstruction chunks. Use SVO for trajectory/world anchoring. Do not use SVO poses directly to place each DA3 depth map unless/until the SVO/DA3 pose convention and rotation mismatch are fully resolved.

Current preferred pipeline:

1. Build the chunk in the DA3-inverted frame:
```bash
python scripts/make_pcl_chunk_from_da3_npz.py \
  --rgb_dir outputs/euroc_v1_01/cam0_rgb \
  --da3_npz outputs/euroc_v1_01/da3_022_037/exports/mini_npz/results.npz \
  --sync_csv outputs/euroc_v1_01/sync_022_037.csv \
  --output outputs/euroc_v1_01/fusion/pcl_022_037_da3inv.ply \
  --start_frame 22 \
  --end_frame 37 \
  --stride 6 \
  --max_dt 0.20 \
  --max_depth 10 \
  --pose_source da3_inv
```

2. Align the finished DA3-inv chunk into the SVO frame:
```bash
python scripts/align_da3inv_ply_to_svo.py \
  --input_ply outputs/euroc_v1_01/fusion/pcl_022_037_da3inv.ply \
  --output_ply outputs/euroc_v1_01/fusion/pcl_022_037_da3inv_aligned_to_svo.ply \
  --report_json outputs/euroc_v1_01/fusion/pcl_022_037_da3inv_aligned_to_svo_report.json \
  --sync_csv outputs/euroc_v1_01/sync_022_037.csv \
  --da3_npz outputs/euroc_v1_01/da3_022_037/exports/mini_npz/results.npz \
  --start_frame 22 \
  --end_frame 37 \
  --max_dt 0.20
```

The chunk-level alignment estimates:
```text
SVO centers ~= scale * R * DA3_inv_centers + t
```

V1_01_easy frames 22-37 result:
```text
scale: 1.119350852
center alignment rmse: 0.070743933
median: 0.051226038
max: 0.137270800
points: 72576
```

This is different from forcing SVO into the per-frame depth fusion. The winning approach is:

```text
DA3:
  build coherent local point-cloud chunks
  using DA3 depth + DA3-inverted poses

SVO:
  estimate camera trajectory over time
  provide world/trajectory anchoring

Fusion/alignment:
  align finished DA3 chunks to SVO trajectory afterward
```

### Center alignment diagnostic

Created `scripts/compare_svo_da3_centers.py`.

It:
```text
loads SVO centers from the sync CSV
loads DA3 extrinsics from results.npz
inverts DA3 extrinsics and extracts DA3 camera centers
fits DA3_inv ~= scale * R * SVO + t with Umeyama
prints path lengths, scale, rotation, translation, and alignment error
```

Ran with:
```bash
python scripts/compare_svo_da3_centers.py \
  --sync_csv outputs/euroc_v1_01/sync_022_037.csv \
  --da3_npz outputs/euroc_v1_01/da3_022_037/exports/mini_npz/results.npz \
  --start_frame 22 \
  --end_frame 37 \
  --max_dt 0.20
```

Key results:
```text
SVO path length:          0.919503047
DA3 inverted path length: 0.778154457
DA3/SVO ratio:           0.846277193
Sim(3) scale:            0.810515800
RMSE:                    0.060198660
max error:               0.123495918
```

Fitting Sim(3) and visually comparing the PLY files showed:
```text
DA3 depth + DA3-inverted poses = best local 3D reconstruction

SVO poses = useful independent trajectory estimate

SVO poses forced into DA3 map = currently worse reconstruction
```

### Second aligned chunk: frames 38-53

After fixing the NVIDIA driver/library mismatch, CUDA worked again in the `da3` env:
```text
Torch: 2.3.1+cu121
Torch CUDA build: 12.1
CUDA available: True
Device count: 1
GPU: NVIDIA GeForce RTX 4070
```

Ran DA3 on the next 16-frame chunk:
```bash
da3 images outputs/euroc_v1_01/cam0_rgb_038_053 \
  --export-dir outputs/euroc_v1_01/da3_038_053 \
  --export-format mini_npz \
  --device cuda \
  --process-res 504 \
  --auto-cleanup
```

DA3 output:
```text
outputs/euroc_v1_01/da3_038_053/exports/mini_npz/results.npz
depth:      (16, 322, 504)
conf:       (16, 322, 504)
extrinsics: (16, 3, 4)
intrinsics: (16, 3, 3)
```

Created sync CSV for frames through 53:
```bash
python scripts/sync_frames_to_svo.py \
  --frames_csv outputs/euroc_v1_01/cam0_timestamps.csv \
  --svo_tum outputs/svo_v1_01_clean/svo_pose_cam_tum.txt \
  --output outputs/euroc_v1_01/sync_038_053.csv \
  --max_frames 54 \
  --max_dt 0.20
```

Frames 38-53 have exact sync:
```text
dt = 0.000000000 for frames 38-53
```

Built the DA3-inv local chunk:
```bash
conda run -n da3 python scripts/make_pcl_chunk_from_da3_npz.py \
  --rgb_dir outputs/euroc_v1_01/cam0_rgb \
  --da3_npz outputs/euroc_v1_01/da3_038_053/exports/mini_npz/results.npz \
  --sync_csv outputs/euroc_v1_01/sync_038_053.csv \
  --output outputs/euroc_v1_01/fusion/pcl_038_053_da3inv.ply \
  --start_frame 38 \
  --end_frame 53 \
  --stride 6 \
  --max_dt 0.20 \
  --max_depth 10 \
  --pose_source da3_inv
```

Output:
```text
outputs/euroc_v1_01/fusion/pcl_038_053_da3inv.ply
16 synchronized frames
4,536 points per frame
72,576 total points
```

Aligned the finished DA3-inv chunk into the SVO frame:
```bash
python scripts/align_da3inv_ply_to_svo.py \
  --input_ply outputs/euroc_v1_01/fusion/pcl_038_053_da3inv.ply \
  --output_ply outputs/euroc_v1_01/fusion/pcl_038_053_da3inv_aligned_to_svo.ply \
  --report_json outputs/euroc_v1_01/fusion/pcl_038_053_da3inv_aligned_to_svo_report.json \
  --sync_csv outputs/euroc_v1_01/sync_038_053.csv \
  --da3_npz outputs/euroc_v1_01/da3_038_053/exports/mini_npz/results.npz \
  --start_frame 38 \
  --end_frame 53 \
  --max_dt 0.20
```

Alignment result:
```text
scale: 0.950041474
center alignment rmse: 0.046832859
median: 0.032263243
mean: 0.038980903
max: 0.104224733
points: 72576
```

Files to compare visually:
```text
outputs/euroc_v1_01/fusion/pcl_022_037_da3inv_aligned_to_svo.ply
outputs/euroc_v1_01/fusion/pcl_038_053_da3inv_aligned_to_svo.ply
```

### Third aligned chunk: frames 54-69

Ran DA3 on the third 16-frame chunk:
```bash
da3 images outputs/euroc_v1_01/cam0_rgb_054_069 \
  --export-dir outputs/euroc_v1_01/da3_054_069 \
  --export-format mini_npz \
  --device cuda \
  --process-res 504 \
  --auto-cleanup
```

DA3 output:
```text
outputs/euroc_v1_01/da3_054_069/exports/mini_npz/results.npz
depth:      (16, 322, 504)
conf:       (16, 322, 504)
extrinsics: (16, 3, 4)
intrinsics: (16, 3, 3)
```

Created sync CSV for frames through 69:
```bash
python scripts/sync_frames_to_svo.py \
  --frames_csv outputs/euroc_v1_01/cam0_timestamps.csv \
  --svo_tum outputs/svo_v1_01_clean/svo_pose_cam_tum.txt \
  --output outputs/euroc_v1_01/sync_054_069.csv \
  --max_frames 70 \
  --max_dt 0.20
```

Frames 54-69 have exact sync:
```text
dt = 0.000000000 for frames 54-69
```

Built the DA3-inv local chunk:
```bash
conda run -n da3 python scripts/make_pcl_chunk_from_da3_npz.py \
  --rgb_dir outputs/euroc_v1_01/cam0_rgb \
  --da3_npz outputs/euroc_v1_01/da3_054_069/exports/mini_npz/results.npz \
  --sync_csv outputs/euroc_v1_01/sync_054_069.csv \
  --output outputs/euroc_v1_01/fusion/pcl_054_069_da3inv.ply \
  --start_frame 54 \
  --end_frame 69 \
  --stride 6 \
  --max_dt 0.20 \
  --max_depth 10 \
  --pose_source da3_inv
```

Output:
```text
outputs/euroc_v1_01/fusion/pcl_054_069_da3inv.ply
16 synchronized frames
4,536 points per frame
72,576 total points
```

Aligned the finished DA3-inv chunk into the SVO frame:
```bash
python scripts/align_da3inv_ply_to_svo.py \
  --input_ply outputs/euroc_v1_01/fusion/pcl_054_069_da3inv.ply \
  --output_ply outputs/euroc_v1_01/fusion/pcl_054_069_da3inv_aligned_to_svo.ply \
  --report_json outputs/euroc_v1_01/fusion/pcl_054_069_da3inv_aligned_to_svo_report.json \
  --sync_csv outputs/euroc_v1_01/sync_054_069.csv \
  --da3_npz outputs/euroc_v1_01/da3_054_069/exports/mini_npz/results.npz \
  --start_frame 54 \
  --end_frame 69 \
  --max_dt 0.20
```

Alignment result:
```text
scale: 1.048061252
center alignment rmse: 0.044326714
median: 0.042417228
mean: 0.040959101
max: 0.071762554
points: 72576
```

Files to compare visually:
```text
outputs/euroc_v1_01/fusion/pcl_022_037_da3inv_aligned_to_svo.ply
outputs/euroc_v1_01/fusion/pcl_038_053_da3inv_aligned_to_svo.ply
outputs/euroc_v1_01/fusion/pcl_054_069_da3inv_aligned_to_svo.ply
```

### Fourth and fifth aligned chunks: frames 70-85 and 86-99

Ran DA3 on the remaining chunks:
```bash
da3 images outputs/euroc_v1_01/cam0_rgb_070_085 \
  --export-dir outputs/euroc_v1_01/da3_070_085 \
  --export-format mini_npz \
  --device cuda \
  --process-res 504 \
  --auto-cleanup

da3 images outputs/euroc_v1_01/cam0_rgb_086_099 \
  --export-dir outputs/euroc_v1_01/da3_086_099 \
  --export-format mini_npz \
  --device cuda \
  --process-res 504 \
  --auto-cleanup
```

DA3 output:
```text
outputs/euroc_v1_01/da3_070_085/exports/mini_npz/results.npz
depth:      (16, 322, 504)
conf:       (16, 322, 504)
extrinsics: (16, 3, 4)
intrinsics: (16, 3, 3)

outputs/euroc_v1_01/da3_086_099/exports/mini_npz/results.npz
depth:      (14, 322, 504)
conf:       (14, 322, 504)
extrinsics: (14, 3, 4)
intrinsics: (14, 3, 3)
```

Built and aligned the DA3-inv chunks:
```bash
conda run -n da3 python scripts/make_pcl_chunk_from_da3_npz.py \
  --rgb_dir outputs/euroc_v1_01/cam0_rgb \
  --da3_npz outputs/euroc_v1_01/da3_070_085/exports/mini_npz/results.npz \
  --sync_csv outputs/euroc_v1_01/sync_070_085.csv \
  --output outputs/euroc_v1_01/fusion/pcl_070_085_da3inv.ply \
  --start_frame 70 \
  --end_frame 85 \
  --stride 6 \
  --max_dt 0.20 \
  --max_depth 10 \
  --pose_source da3_inv

python scripts/align_da3inv_ply_to_svo.py \
  --input_ply outputs/euroc_v1_01/fusion/pcl_070_085_da3inv.ply \
  --output_ply outputs/euroc_v1_01/fusion/pcl_070_085_da3inv_aligned_to_svo.ply \
  --report_json outputs/euroc_v1_01/fusion/pcl_070_085_da3inv_aligned_to_svo_report.json \
  --sync_csv outputs/euroc_v1_01/sync_070_085.csv \
  --da3_npz outputs/euroc_v1_01/da3_070_085/exports/mini_npz/results.npz \
  --start_frame 70 \
  --end_frame 85 \
  --max_dt 0.20

conda run -n da3 python scripts/make_pcl_chunk_from_da3_npz.py \
  --rgb_dir outputs/euroc_v1_01/cam0_rgb \
  --da3_npz outputs/euroc_v1_01/da3_086_099/exports/mini_npz/results.npz \
  --sync_csv outputs/euroc_v1_01/sync_086_099.csv \
  --output outputs/euroc_v1_01/fusion/pcl_086_099_da3inv.ply \
  --start_frame 86 \
  --end_frame 99 \
  --stride 6 \
  --max_dt 0.20 \
  --max_depth 10 \
  --pose_source da3_inv

python scripts/align_da3inv_ply_to_svo.py \
  --input_ply outputs/euroc_v1_01/fusion/pcl_086_099_da3inv.ply \
  --output_ply outputs/euroc_v1_01/fusion/pcl_086_099_da3inv_aligned_to_svo.ply \
  --report_json outputs/euroc_v1_01/fusion/pcl_086_099_da3inv_aligned_to_svo_report.json \
  --sync_csv outputs/euroc_v1_01/sync_086_099.csv \
  --da3_npz outputs/euroc_v1_01/da3_086_099/exports/mini_npz/results.npz \
  --start_frame 86 \
  --end_frame 99 \
  --max_dt 0.20
```

Alignment results:
```text
70-85:
scale: 0.881106288
center alignment rmse: 0.061002403
median: 0.052257319
max: 0.110568903
points: 72576

86-99:
scale: 0.809408675
center alignment rmse: 0.047805449
median: 0.042195102
max: 0.084149186
points: 63504
```

All five aligned chunks now exist:
```text
outputs/euroc_v1_01/fusion/pcl_022_037_da3inv_aligned_to_svo.ply
outputs/euroc_v1_01/fusion/pcl_038_053_da3inv_aligned_to_svo.ply
outputs/euroc_v1_01/fusion/pcl_054_069_da3inv_aligned_to_svo.ply
outputs/euroc_v1_01/fusion/pcl_070_085_da3inv_aligned_to_svo.ply
outputs/euroc_v1_01/fusion/pcl_086_099_da3inv_aligned_to_svo.ply
```

### Merged V1_01_easy DA3-inv + SVO reconstruction

Added `scripts/merge_ascii_ply.py` to concatenate generated ASCII PLY files.

Merged the five SVO-aligned DA3-inv chunks:
```bash
python scripts/merge_ascii_ply.py \
  --output outputs/euroc_v1_01/fusion/pcl_022_099_da3inv_aligned_to_svo_merged.ply \
  outputs/euroc_v1_01/fusion/pcl_022_037_da3inv_aligned_to_svo.ply \
  outputs/euroc_v1_01/fusion/pcl_038_053_da3inv_aligned_to_svo.ply \
  outputs/euroc_v1_01/fusion/pcl_054_069_da3inv_aligned_to_svo.ply \
  outputs/euroc_v1_01/fusion/pcl_070_085_da3inv_aligned_to_svo.ply \
  outputs/euroc_v1_01/fusion/pcl_086_099_da3inv_aligned_to_svo.ply
```

Result:
```text
outputs/euroc_v1_01/fusion/pcl_022_099_da3inv_aligned_to_svo_merged.ply

Loaded chunks:
22-37: 72,576 points
38-53: 72,576 points
54-69: 72,576 points
70-85: 72,576 points
86-99: 63,504 points

Total: 353,808 points
```

### Visual inspection screenshots

Saved visual inspection screenshots under:
```text
outputs/visual outputs/
```

Screenshots:
```text
outputs/visual outputs/pcl_022_037_da3inv_aligned_to_svo.png
outputs/visual outputs/pcl_038_053_da3inv_aligned_to_svo.png
outputs/visual outputs/pcl_054_069_da3inv_aligned_to_svo.png
outputs/visual outputs/pcl_070_085_da3inv_aligned_to_svo.png
outputs/visual outputs/pcl_086_099_da3inv_aligned_to_svo.png
outputs/visual outputs/pcl_022_099_da3inv_aligned_to_svo.png
```

Visual reading:
```text
The five individual SVO-aligned chunks look locally coherent.
The simple concatenated merged cloud is not good enough as a final map.
The failure is likely from independent per-chunk Sim(3) alignments:
each chunk is adjusted into SVO coordinates, but each uses its own scale,
rotation, and translation, so the chunks are not mutually consistent enough
for raw concatenation.
```

Next fusion direction:
```text
Do not treat the raw merged PLY as final.
Try a global alignment strategy or overlapping-chunk alignment before merging:
1. Estimate one shared Sim(3) over all DA3-inv camera centers and SVO centers.
2. Apply that shared transform to all DA3-inv chunks.
3. Re-merge and compare against the current per-chunk-Sim(3) merge.
```

### Supervisor recommendation: overlapping chunks from SVO trajectory

Supervisor recommendation:
```text
Use SVO to create/select the trajectory and keyframes.
Then create DA3 chunks around those keyframes with overlapping frames.
Use the overlap frames to estimate where neighboring chunks should merge.
Alternatively, split the SVO trajectory into equal motion-length segments and
create DA3 reconstructions from those trajectory-based windows.
```

Reason:
```text
The previous non-overlapping chunks were each locally coherent, but the raw
merged map failed because each chunk used an independent Sim(3) alignment.
Overlapping chunks provide shared frames/geometry between neighboring DA3 runs,
which can be used to estimate chunk-to-chunk alignment before global merging.
```

First overlap experiment:
```text
chunk A: 22-37, already exists
chunk B: 30-45, new overlapping chunk
overlap: 30-37
```

Prepared DA3 input folder:
```text
outputs/euroc_v1_01/cam0_rgb_030_045/
16 images: frame_000030.png through frame_000045.png
```

Initial DA3 command at process resolution 504 hit CUDA OOM, so the overlap
diagnostic was run at process resolution 448:
```bash
da3 images outputs/euroc_v1_01/cam0_rgb_030_045 \
  --export-dir outputs/euroc_v1_01/da3_030_045_r448 \
  --export-format mini_npz \
  --device cuda \
  --process-res 448 \
  --auto-cleanup
```

DA3 output:
```text
outputs/euroc_v1_01/da3_030_045_r448/exports/mini_npz/results.npz
depth:      (16, 280, 448)
conf:       (16, 280, 448)
extrinsics: (16, 3, 4)
intrinsics: (16, 3, 3)
```

Built the overlap chunk:
```bash
conda run -n da3 python scripts/make_pcl_chunk_from_da3_npz.py \
  --rgb_dir outputs/euroc_v1_01/cam0_rgb \
  --da3_npz outputs/euroc_v1_01/da3_030_045_r448/exports/mini_npz/results.npz \
  --sync_csv outputs/euroc_v1_01/sync_030_045.csv \
  --output outputs/euroc_v1_01/fusion/pcl_030_045_r448_da3inv.ply \
  --start_frame 30 \
  --end_frame 45 \
  --stride 6 \
  --max_dt 0.20 \
  --max_depth 10 \
  --pose_source da3_inv
```

Output:
```text
outputs/euroc_v1_01/fusion/pcl_030_045_r448_da3inv.ply
16 synchronized frames
3,525 points per frame
56,400 total points
```

Estimated chunk-to-chunk Sim(3) using overlap frames 30-37:
```bash
python scripts/align_da3_overlap_ply.py \
  --source_ply outputs/euroc_v1_01/fusion/pcl_030_045_r448_da3inv.ply \
  --output_ply outputs/euroc_v1_01/fusion/pcl_030_045_r448_da3inv_aligned_to_022_037_overlap.ply \
  --source_da3_npz outputs/euroc_v1_01/da3_030_045_r448/exports/mini_npz/results.npz \
  --target_da3_npz outputs/euroc_v1_01/da3_022_037/exports/mini_npz/results.npz \
  --source_start_frame 30 \
  --target_start_frame 22 \
  --overlap_start_frame 30 \
  --overlap_end_frame 37 \
  --report_json outputs/euroc_v1_01/fusion/pcl_030_045_r448_da3inv_aligned_to_022_037_overlap_report.json
```

Overlap alignment result:
```text
scale: 1.018218831
center alignment rmse: 0.017975386
median: 0.014775559
mean: 0.016396625
max: 0.033323237
points: 56,400
```

Merged only the two overlapping chunks as a diagnostic:
```bash
python scripts/merge_ascii_ply.py \
  --output outputs/euroc_v1_01/fusion/pcl_022_045_da3inv_overlap_merged.ply \
  outputs/euroc_v1_01/fusion/pcl_022_037_da3inv.ply \
  outputs/euroc_v1_01/fusion/pcl_030_045_r448_da3inv_aligned_to_022_037_overlap.ply
```

Diagnostic output:
```text
outputs/euroc_v1_01/fusion/pcl_022_045_da3inv_overlap_merged.ply
22-37 target chunk: 72,576 points
30-45 aligned source chunk: 56,400 points
Total: 128,976 points
```

Next visual check:
```text
Open outputs/euroc_v1_01/fusion/pcl_022_045_da3inv_overlap_merged.ply
and check whether the overlap region 30-37 is coherent.
```

Visual result:
```text
outputs/euroc_v1_01/fusion/pcl_022_045_da3inv_overlap_merged.ply
looks much better than the previous raw concatenation of independently
SVO-aligned chunks. A few features appear lost or weaker, likely because the
new overlap chunk was run at lower process resolution 448 after the 504 run hit
CUDA OOM. Despite that, the spatial alignment is much more coherent.
```

Interpretation:
```text
Overlapping chunks are a promising direction.
The overlap-frame Sim(3) between DA3 chunk coordinate systems is more stable
than independently aligning every chunk to SVO and concatenating.
Next, repeat the overlap strategy for more neighboring windows, preferably at
process resolution 504 when GPU memory allows, or keep 448 for diagnostics.
```

### Second overlap pair: 30-45 and 38-53

Tested the next neighboring overlap:
```text
target chunk: 30-45, r448
source chunk: 38-53, original 504
overlap: 38-45
```

Estimated chunk-to-chunk Sim(3):
```bash
python scripts/align_da3_overlap_ply.py \
  --source_ply outputs/euroc_v1_01/fusion/pcl_038_053_da3inv.ply \
  --output_ply outputs/euroc_v1_01/fusion/pcl_038_053_da3inv_aligned_to_030_045_overlap.ply \
  --source_da3_npz outputs/euroc_v1_01/da3_038_053/exports/mini_npz/results.npz \
  --target_da3_npz outputs/euroc_v1_01/da3_030_045_r448/exports/mini_npz/results.npz \
  --source_start_frame 38 \
  --target_start_frame 30 \
  --overlap_start_frame 38 \
  --overlap_end_frame 45 \
  --report_json outputs/euroc_v1_01/fusion/pcl_038_053_da3inv_aligned_to_030_045_overlap_report.json
```

Overlap alignment result:
```text
scale: 1.060842746
center alignment rmse: 0.016028364
median: 0.013522100
mean: 0.014874990
max: 0.024717956
points: 72,576
```

Merged only the second overlapping pair as a diagnostic:
```bash
python scripts/merge_ascii_ply.py \
  --output outputs/euroc_v1_01/fusion/pcl_030_053_da3inv_overlap_merged.ply \
  outputs/euroc_v1_01/fusion/pcl_030_045_r448_da3inv.ply \
  outputs/euroc_v1_01/fusion/pcl_038_053_da3inv_aligned_to_030_045_overlap.ply
```

Diagnostic output:
```text
outputs/euroc_v1_01/fusion/pcl_030_053_da3inv_overlap_merged.ply
30-45 target chunk: 56,400 points
38-53 aligned source chunk: 72,576 points
Total: 128,976 points
```

Next visual check:
```text
Open outputs/euroc_v1_01/fusion/pcl_030_053_da3inv_overlap_merged.ply
and check whether overlap frames 38-45 are coherent.
```

Visual result:
```text
outputs/euroc_v1_01/fusion/pcl_030_053_da3inv_overlap_merged.ply
looks visibly rotated/misaligned even though center RMSE is low.
This suggests center-only Sim(3) can be underconstrained for short overlap
windows: camera centers can align while camera/world orientation and dense
surfaces still disagree.
```

### Orientation-constrained overlap alignment diagnostic

Updated `scripts/align_da3_overlap_ply.py` with:
```text
--rotation_mode centers       # previous behavior, full Sim(3) from centers
--rotation_mode orientations  # estimate rotation from DA3-inv camera rotations,
                              # then estimate scale/translation from centers
--rotation_mode hybrid        # estimate rotation from both normalized center
                              # motion and DA3-inv camera orientations, then
                              # estimate scale/translation from centers
```

Rationale:
```text
For overlap frames, DA3 provides both camera centers and camera orientations.
Instead of letting a short camera-center trajectory choose the rotation freely,
we can estimate R_align from overlapping DA3-inv camera orientations:

target_R_i ~= R_align @ source_R_i

Then scale and translation are fit from the camera centers using that fixed
rotation.
```

Reran the problematic pair with orientation-constrained rotation:
```bash
python scripts/align_da3_overlap_ply.py \
  --source_ply outputs/euroc_v1_01/fusion/pcl_038_053_da3inv.ply \
  --output_ply outputs/euroc_v1_01/fusion/pcl_038_053_da3inv_aligned_to_030_045_overlap_orient.ply \
  --source_da3_npz outputs/euroc_v1_01/da3_038_053/exports/mini_npz/results.npz \
  --target_da3_npz outputs/euroc_v1_01/da3_030_045_r448/exports/mini_npz/results.npz \
  --source_start_frame 38 \
  --target_start_frame 30 \
  --overlap_start_frame 38 \
  --overlap_end_frame 45 \
  --rotation_mode orientations \
  --report_json outputs/euroc_v1_01/fusion/pcl_038_053_da3inv_aligned_to_030_045_overlap_orient_report.json
```

Orientation-constrained result:
```text
scale: 1.059865877
center alignment rmse: 0.018349295
median center error: 0.016426594
max center error: 0.025937289

rotation rmse: 1.504206884 deg
median rotation error: 1.165625462 deg
max rotation error: 2.901086922 deg
points: 72,576
```

Merged orientation-constrained pair:
```bash
python scripts/merge_ascii_ply.py \
  --output outputs/euroc_v1_01/fusion/pcl_030_053_da3inv_overlap_orient_merged.ply \
  outputs/euroc_v1_01/fusion/pcl_030_045_r448_da3inv.ply \
  outputs/euroc_v1_01/fusion/pcl_038_053_da3inv_aligned_to_030_045_overlap_orient.ply
```

Next visual comparison:
```text
outputs/euroc_v1_01/fusion/pcl_030_053_da3inv_overlap_merged.ply
outputs/euroc_v1_01/fusion/pcl_030_053_da3inv_overlap_orient_merged.ply
```

### Hybrid pose overlap alignment diagnostic

Added hybrid rotation mode to use both overlap camera centers and camera
orientations:
```text
camera centers constrain translation/scale and motion direction
camera orientations constrain the rotation gauge
```

Tested `38-53 -> 30-45` with hybrid mode:
```bash
python scripts/align_da3_overlap_ply.py \
  --source_ply outputs/euroc_v1_01/fusion/pcl_038_053_da3inv.ply \
  --output_ply outputs/euroc_v1_01/fusion/pcl_038_053_da3inv_aligned_to_030_045_overlap_hybrid_w1.ply \
  --source_da3_npz outputs/euroc_v1_01/da3_038_053/exports/mini_npz/results.npz \
  --target_da3_npz outputs/euroc_v1_01/da3_030_045_r448/exports/mini_npz/results.npz \
  --source_start_frame 38 \
  --target_start_frame 30 \
  --overlap_start_frame 38 \
  --overlap_end_frame 45 \
  --rotation_mode hybrid \
  --orientation_weight 1.0 \
  --report_json outputs/euroc_v1_01/fusion/pcl_038_053_da3inv_aligned_to_030_045_overlap_hybrid_w1_report.json
```

Hybrid result:
```text
scale: 1.060141389
center alignment rmse: 0.017725714
median center error: 0.015568945
max center error: 0.025186463

rotation rmse: 1.613789171 deg
median rotation error: 1.063680598 deg
max rotation error: 3.161348191 deg
points: 72,576
```

Merged hybrid pair:
```bash
python scripts/merge_ascii_ply.py \
  --output outputs/euroc_v1_01/fusion/pcl_030_053_da3inv_overlap_hybrid_w1_merged.ply \
  outputs/euroc_v1_01/fusion/pcl_030_045_r448_da3inv.ply \
  outputs/euroc_v1_01/fusion/pcl_038_053_da3inv_aligned_to_030_045_overlap_hybrid_w1.ply
```

Next visual comparison:
```text
outputs/euroc_v1_01/fusion/pcl_030_053_da3inv_overlap_merged.ply
outputs/euroc_v1_01/fusion/pcl_030_053_da3inv_overlap_orient_merged.ply
outputs/euroc_v1_01/fusion/pcl_030_053_da3inv_overlap_hybrid_w1_merged.ply
```

### Next overlap diagnostic: 46-61 -> 38-53

To check whether the bad `38-53 -> 30-45` overlap merge is local to that pair
or repeatable, generated the next overlapping DA3 chunk:
```bash
da3 images outputs/euroc_v1_01/cam0_rgb_046_061 \
  --export-dir outputs/euroc_v1_01/da3_046_061 \
  --export-format mini_npz \
  --device cuda \
  --process-res 504 \
  --auto-cleanup
```

DA3 succeeded at full `504` process resolution:
```text
frames: 46-61
images: 16
depth shape: 322 x 504
export: outputs/euroc_v1_01/da3_046_061/exports/mini_npz/results.npz
```

Built the local DA3-inverted reconstruction:
```bash
python scripts/make_pcl_chunk_from_da3_npz.py \
  --rgb_dir outputs/euroc_v1_01/cam0_rgb \
  --da3_npz outputs/euroc_v1_01/da3_046_061/exports/mini_npz/results.npz \
  --sync_csv outputs/euroc_v1_01/sync_046_061.csv \
  --output outputs/euroc_v1_01/fusion/pcl_046_061_da3inv.ply \
  --start_frame 46 \
  --end_frame 61 \
  --stride 6 \
  --max_dt 0.20 \
  --max_depth 10 \
  --pose_source da3_inv
```

Output:
```text
outputs/euroc_v1_01/fusion/pcl_046_061_da3inv.ply
points: 72,576
```

Aligned source `46-61` into target `38-53` using overlap frames `46-53`:
```bash
python scripts/align_da3_overlap_ply.py \
  --source_ply outputs/euroc_v1_01/fusion/pcl_046_061_da3inv.ply \
  --output_ply outputs/euroc_v1_01/fusion/pcl_046_061_da3inv_aligned_to_038_053_overlap.ply \
  --source_da3_npz outputs/euroc_v1_01/da3_046_061/exports/mini_npz/results.npz \
  --target_da3_npz outputs/euroc_v1_01/da3_038_053/exports/mini_npz/results.npz \
  --source_start_frame 46 \
  --target_start_frame 38 \
  --overlap_start_frame 46 \
  --overlap_end_frame 53 \
  --report_json outputs/euroc_v1_01/fusion/pcl_046_061_da3inv_aligned_to_038_053_overlap_report.json
```

Center-only overlap result:
```text
scale: 1.128511042
center alignment rmse: 0.020200962
median center error: 0.019882278
max center error: 0.026823371

rotation rmse: 8.897612971 deg
median rotation error: 8.659081888 deg
max rotation error: 10.164186260 deg
points: 72,576
```

Merged diagnostic pair:
```bash
python scripts/merge_ascii_ply.py \
  --output outputs/euroc_v1_01/fusion/pcl_038_061_da3inv_overlap_merged.ply \
  outputs/euroc_v1_01/fusion/pcl_038_053_da3inv.ply \
  outputs/euroc_v1_01/fusion/pcl_046_061_da3inv_aligned_to_038_053_overlap.ply
```

Visual check target:
```text
outputs/euroc_v1_01/fusion/pcl_038_061_da3inv_overlap_merged.ply
```

Interpretation so far:
```text
Low overlap center RMSE is not sufficient for a visually correct merge.
For 46-61 -> 38-53, the centers align tightly but the DA3-inverted camera
orientations disagree by roughly 9 degrees across the same overlap.
This supports the suspicion that each DA3 chunk has its own local pose gauge,
and overlap merging needs an orientation/geometry-aware constraint rather than
camera-center Sim(3) alone.
```

### Overlap-only geometry ICP diagnostic

Added a lightweight diagnostic script:
```text
scripts/refine_da3_overlap_icp.py
```

Purpose:
```text
Initialize source -> target from overlap camera-center Sim(3), then build
overlap-only point clouds directly from the two DA3 NPZ files and refine the
alignment with nearest-neighbor point-to-point Sim(3) ICP.
```

Important caveat:
```text
This is only a diagnostic. The overlap geometry has repeated planes and sparse
walls, so unconstrained nearest-neighbor ICP can drift toward wrong planar
matches even when the nearest-neighbor error decreases.
```

Tested bad overlap pair `46-61 -> 38-53`, overlap `46-53`:
```bash
python scripts/refine_da3_overlap_icp.py \
  --source_ply outputs/euroc_v1_01/fusion/pcl_046_061_da3inv.ply \
  --target_ply outputs/euroc_v1_01/fusion/pcl_038_053_da3inv.ply \
  --output_ply outputs/euroc_v1_01/fusion/pcl_046_061_da3inv_aligned_to_038_053_overlap_icp.ply \
  --merged_output_ply outputs/euroc_v1_01/fusion/pcl_038_061_da3inv_overlap_icp_merged.ply \
  --source_da3_npz outputs/euroc_v1_01/da3_046_061/exports/mini_npz/results.npz \
  --target_da3_npz outputs/euroc_v1_01/da3_038_053/exports/mini_npz/results.npz \
  --source_start_frame 46 \
  --target_start_frame 38 \
  --overlap_start_frame 46 \
  --overlap_end_frame 53 \
  --stride 8 \
  --max_depth 10 \
  --icp_iterations 20 \
  --max_corr 0.08 \
  --report_json outputs/euroc_v1_01/fusion/pcl_046_061_da3inv_aligned_to_038_053_overlap_icp_report.json
```

20-iteration ICP result:
```text
overlap points: source=20,664 target=20,664
initial scale: 1.128511042
refined scale: 1.097454155
final overlap nearest-neighbor rmse: 0.040088950
median: 0.030002105
max: 0.079983544
correspondences within 0.08: 5,768
```

Also tested a short 4-iteration version because the ICP history improved early,
then began drifting:
```bash
python scripts/refine_da3_overlap_icp.py \
  --source_ply outputs/euroc_v1_01/fusion/pcl_046_061_da3inv.ply \
  --target_ply outputs/euroc_v1_01/fusion/pcl_038_053_da3inv.ply \
  --output_ply outputs/euroc_v1_01/fusion/pcl_046_061_da3inv_aligned_to_038_053_overlap_icp4.ply \
  --merged_output_ply outputs/euroc_v1_01/fusion/pcl_038_061_da3inv_overlap_icp4_merged.ply \
  --source_da3_npz outputs/euroc_v1_01/da3_046_061/exports/mini_npz/results.npz \
  --target_da3_npz outputs/euroc_v1_01/da3_038_053/exports/mini_npz/results.npz \
  --source_start_frame 46 \
  --target_start_frame 38 \
  --overlap_start_frame 46 \
  --overlap_end_frame 53 \
  --stride 8 \
  --max_depth 10 \
  --icp_iterations 4 \
  --max_corr 0.08 \
  --report_json outputs/euroc_v1_01/fusion/pcl_046_061_da3inv_aligned_to_038_053_overlap_icp4_report.json
```

4-iteration ICP result:
```text
overlap points: source=20,664 target=20,664
initial scale: 1.128511042
refined scale: 1.136395430
final overlap nearest-neighbor rmse: 0.038621939
median: 0.029367544
max: 0.079981728
correspondences within 0.08: 3,851
```

Visual comparison targets:
```text
outputs/euroc_v1_01/fusion/pcl_038_061_da3inv_overlap_merged.ply
outputs/euroc_v1_01/fusion/pcl_038_061_da3inv_overlap_icp_merged.ply
outputs/euroc_v1_01/fusion/pcl_038_061_da3inv_overlap_icp4_merged.ply
```

Interpretation:
```text
Geometry ICP can reduce local nearest-neighbor distances, but the low inlier
count and scale drift suggest this is not yet a reliable merge method. The next
refinement should constrain ICP more strongly, for example by using only stable
overlap frames/regions, adding color/feature constraints, or optimizing against
matched overlap frame surfaces instead of one pooled nearest-neighbor cloud.
```

### Smaller chunk / heavier overlap test

Hypothesis:
```text
Using shorter DA3 chunks with more overlap may reduce chunk-to-chunk gauge
differences. Instead of 16-frame chunks with 8-frame overlap, test 9-frame
chunks with 6-frame overlap near the bad region.
```

Test pair:
```text
target chunk: 38-46
source chunk: 41-49
overlap: 41-46
chunk length: 9 frames
overlap length: 6 frames
overlap ratio: 67%
```

Prepared image folders:
```text
outputs/euroc_v1_01/cam0_rgb_038_046
outputs/euroc_v1_01/cam0_rgb_041_049
```

Generated DA3 mini NPZ exports at full `504` process resolution:
```bash
da3 images outputs/euroc_v1_01/cam0_rgb_038_046 \
  --export-dir outputs/euroc_v1_01/da3_038_046 \
  --export-format mini_npz \
  --device cuda \
  --process-res 504 \
  --auto-cleanup

da3 images outputs/euroc_v1_01/cam0_rgb_041_049 \
  --export-dir outputs/euroc_v1_01/da3_041_049 \
  --export-format mini_npz \
  --device cuda \
  --process-res 504 \
  --auto-cleanup
```

Both DA3 runs succeeded:
```text
images per chunk: 9
depth shape: 322 x 504
export format: mini_npz
```

Built DA3-inverted local chunks:
```bash
conda run -n da3 python scripts/make_pcl_chunk_from_da3_npz.py \
  --rgb_dir outputs/euroc_v1_01/cam0_rgb \
  --da3_npz outputs/euroc_v1_01/da3_038_046/exports/mini_npz/results.npz \
  --sync_csv outputs/euroc_v1_01/sync_038_046.csv \
  --output outputs/euroc_v1_01/fusion/pcl_038_046_da3inv.ply \
  --start_frame 38 \
  --end_frame 46 \
  --stride 6 \
  --max_dt 0.20 \
  --max_depth 10 \
  --pose_source da3_inv

conda run -n da3 python scripts/make_pcl_chunk_from_da3_npz.py \
  --rgb_dir outputs/euroc_v1_01/cam0_rgb \
  --da3_npz outputs/euroc_v1_01/da3_041_049/exports/mini_npz/results.npz \
  --sync_csv outputs/euroc_v1_01/sync_041_049.csv \
  --output outputs/euroc_v1_01/fusion/pcl_041_049_da3inv.ply \
  --start_frame 41 \
  --end_frame 49 \
  --stride 6 \
  --max_dt 0.20 \
  --max_depth 10 \
  --pose_source da3_inv
```

Outputs:
```text
outputs/euroc_v1_01/fusion/pcl_038_046_da3inv.ply
points: 40,824

outputs/euroc_v1_01/fusion/pcl_041_049_da3inv.ply
points: 40,824
```

Aligned source `41-49` into target `38-46` using overlap frames `41-46`:
```bash
python scripts/align_da3_overlap_ply.py \
  --source_ply outputs/euroc_v1_01/fusion/pcl_041_049_da3inv.ply \
  --output_ply outputs/euroc_v1_01/fusion/pcl_041_049_da3inv_aligned_to_038_046_overlap.ply \
  --source_da3_npz outputs/euroc_v1_01/da3_041_049/exports/mini_npz/results.npz \
  --target_da3_npz outputs/euroc_v1_01/da3_038_046/exports/mini_npz/results.npz \
  --source_start_frame 41 \
  --target_start_frame 38 \
  --overlap_start_frame 41 \
  --overlap_end_frame 46 \
  --report_json outputs/euroc_v1_01/fusion/pcl_041_049_da3inv_aligned_to_038_046_overlap_report.json
```

Center-only Sim(3) result:
```text
scale: 0.943515259
center alignment rmse: 0.014392858
median center error: 0.012496073
max center error: 0.023326947

rotation rmse: 1.491379520 deg
median rotation error: 1.427345887 deg
max rotation error: 1.777885332 deg
points: 40,824
```

Merged visual diagnostic:
```bash
python scripts/merge_ascii_ply.py \
  --output outputs/euroc_v1_01/fusion/pcl_038_049_da3inv_overlap_9f6o_merged.ply \
  outputs/euroc_v1_01/fusion/pcl_038_046_da3inv.ply \
  outputs/euroc_v1_01/fusion/pcl_041_049_da3inv_aligned_to_038_046_overlap.ply
```

Visual check target:
```text
outputs/euroc_v1_01/fusion/pcl_038_049_da3inv_overlap_9f6o_merged.ply
```

Initial interpretation:
```text
This smaller/heavier-overlap pair is numerically much healthier than the bad
46-61 -> 38-53 pair. The orientation disagreement dropped from roughly 9 deg
to roughly 1.5 deg. The visual merged cloud will decide whether the improvement
is enough to justify rebuilding the sequence with 9-frame chunks and 6-frame
overlap.
```

### Scale-preserving 9-frame overlap test

Visual inspection of the `38-49` 9-frame/6-overlap merge looked better, but
showed a clear wall-height difference. Since the estimated Sim(3) scale was
`0.943515259`, tested a scale-preserving variant.

Updated `scripts/align_da3_overlap_ply.py` with:
```text
--scale_override
```

Behavior:
```text
Estimate rotation as before, then force the scale to the requested value and
recompute translation from the overlap camera centers. Use `--scale_override 1.0`
to test a no-scale-change merge.
```

Scale-fixed alignment:
```bash
python scripts/align_da3_overlap_ply.py \
  --source_ply outputs/euroc_v1_01/fusion/pcl_041_049_da3inv.ply \
  --output_ply outputs/euroc_v1_01/fusion/pcl_041_049_da3inv_aligned_to_038_046_overlap_scale1.ply \
  --source_da3_npz outputs/euroc_v1_01/da3_041_049/exports/mini_npz/results.npz \
  --target_da3_npz outputs/euroc_v1_01/da3_038_046/exports/mini_npz/results.npz \
  --source_start_frame 41 \
  --target_start_frame 38 \
  --overlap_start_frame 41 \
  --overlap_end_frame 46 \
  --scale_override 1.0 \
  --report_json outputs/euroc_v1_01/fusion/pcl_041_049_da3inv_aligned_to_038_046_overlap_scale1_report.json
```

Scale-fixed result:
```text
scale: 1.000000000
center alignment rmse: 0.015938823
median center error: 0.015336089
max center error: 0.021680187

rotation rmse: 1.491379520 deg
median rotation error: 1.427345887 deg
max rotation error: 1.777885332 deg
points: 40,824
```

The center error increased only slightly compared with the estimated-scale
result:
```text
estimated scale 0.943515259: center rmse 0.014392858
forced scale    1.000000000: center rmse 0.015938823
```

Merged scale-fixed visual diagnostic:
```bash
python scripts/merge_ascii_ply.py \
  --output outputs/euroc_v1_01/fusion/pcl_038_049_da3inv_overlap_9f6o_scale1_merged.ply \
  outputs/euroc_v1_01/fusion/pcl_038_046_da3inv.ply \
  outputs/euroc_v1_01/fusion/pcl_041_049_da3inv_aligned_to_038_046_overlap_scale1.ply
```

Visual comparison targets:
```text
outputs/euroc_v1_01/fusion/pcl_038_049_da3inv_overlap_9f6o_merged.ply
outputs/euroc_v1_01/fusion/pcl_038_049_da3inv_overlap_9f6o_scale1_merged.ply
```

Interpretation to check visually:
```text
If the scale-fixed merge reduces the wall-height mismatch, then the estimated
Sim(3) scale was over-correcting. If it worsens, then DA3's adjacent 9-frame
chunks really have a local scale difference and we need a constrained but not
fully fixed scale model.
```

### 9-frame / 7-overlap test

Since the scale-fixed `9f6o` merge looked worse, tested more overlap while
keeping 9-frame chunks:
```text
target chunk: 38-46
source chunk: 40-48
overlap: 40-46
chunk length: 9 frames
overlap length: 7 frames
overlap ratio: 78%
```

Prepared:
```text
outputs/euroc_v1_01/cam0_rgb_040_048
outputs/euroc_v1_01/sync_040_048.csv
```

Generated DA3 at full `504` process resolution:
```bash
da3 images outputs/euroc_v1_01/cam0_rgb_040_048 \
  --export-dir outputs/euroc_v1_01/da3_040_048 \
  --export-format mini_npz \
  --device cuda \
  --process-res 504 \
  --auto-cleanup
```

DA3 succeeded:
```text
images: 9
depth shape: 322 x 504
export: outputs/euroc_v1_01/da3_040_048/exports/mini_npz/results.npz
```

Built the local DA3-inverted chunk:
```bash
conda run -n da3 python scripts/make_pcl_chunk_from_da3_npz.py \
  --rgb_dir outputs/euroc_v1_01/cam0_rgb \
  --da3_npz outputs/euroc_v1_01/da3_040_048/exports/mini_npz/results.npz \
  --sync_csv outputs/euroc_v1_01/sync_040_048.csv \
  --output outputs/euroc_v1_01/fusion/pcl_040_048_da3inv.ply \
  --start_frame 40 \
  --end_frame 48 \
  --stride 6 \
  --max_dt 0.20 \
  --max_depth 10 \
  --pose_source da3_inv
```

Output:
```text
outputs/euroc_v1_01/fusion/pcl_040_048_da3inv.ply
points: 40,824
```

Aligned source `40-48` into target `38-46` using overlap frames `40-46`:
```bash
python scripts/align_da3_overlap_ply.py \
  --source_ply outputs/euroc_v1_01/fusion/pcl_040_048_da3inv.ply \
  --output_ply outputs/euroc_v1_01/fusion/pcl_040_048_da3inv_aligned_to_038_046_overlap.ply \
  --source_da3_npz outputs/euroc_v1_01/da3_040_048/exports/mini_npz/results.npz \
  --target_da3_npz outputs/euroc_v1_01/da3_038_046/exports/mini_npz/results.npz \
  --source_start_frame 40 \
  --target_start_frame 38 \
  --overlap_start_frame 40 \
  --overlap_end_frame 46 \
  --report_json outputs/euroc_v1_01/fusion/pcl_040_048_da3inv_aligned_to_038_046_overlap_report.json
```

Center-only Sim(3) result:
```text
scale: 0.947548344
center alignment rmse: 0.018355146
median center error: 0.009372649
max center error: 0.034818015

rotation rmse: 3.011830900 deg
median rotation error: 2.906453151 deg
max rotation error: 3.329079735 deg
points: 40,824
```

Merged visual diagnostic:
```bash
python scripts/merge_ascii_ply.py \
  --output outputs/euroc_v1_01/fusion/pcl_038_048_da3inv_overlap_9f7o_merged.ply \
  outputs/euroc_v1_01/fusion/pcl_038_046_da3inv.ply \
  outputs/euroc_v1_01/fusion/pcl_040_048_da3inv_aligned_to_038_046_overlap.ply
```

Visual comparison targets:
```text
outputs/euroc_v1_01/fusion/pcl_038_049_da3inv_overlap_9f6o_merged.ply
outputs/euroc_v1_01/fusion/pcl_038_048_da3inv_overlap_9f7o_merged.ply
```

Initial interpretation:
```text
More overlap did not automatically improve the DA3 chunk gauge. Compared with
the 9f6o pair, scale stayed similar but orientation disagreement increased:

9f6o 41-49 -> 38-46: scale 0.943515259, rotation rmse 1.491379520 deg
9f7o 40-48 -> 38-46: scale 0.947548344, rotation rmse 3.011830900 deg

This suggests the exact frame window and DA3 reference-view/context choice
matter, not just overlap ratio.
```

### Fusion output cleanup

Cleaned `outputs/euroc_v1_01/fusion` non-destructively by moving generated
diagnostics into subfolders. No point clouds or reports were deleted.

New organization:
```text
outputs/euroc_v1_01/fusion/README.md
outputs/euroc_v1_01/fusion/00_legacy_pose_tests/
outputs/euroc_v1_01/fusion/10_da3inv_local_chunks/
outputs/euroc_v1_01/fusion/20_svo_aligned_chunks/
outputs/euroc_v1_01/fusion/30_overlap_16f/
outputs/euroc_v1_01/fusion/40_overlap_9f/
```

Current visual paths after cleanup:
```text
Best 9f6o candidate:
outputs/euroc_v1_01/fusion/40_overlap_9f/pcl_038_049_da3inv_overlap_9f6o_merged.ply

Scale-fixed 9f6o variant, visually worse:
outputs/euroc_v1_01/fusion/40_overlap_9f/pcl_038_049_da3inv_overlap_9f6o_scale1_merged.ply

9f7o variant, visually worse:
outputs/euroc_v1_01/fusion/40_overlap_9f/pcl_038_048_da3inv_overlap_9f7o_merged.ply

Bad 16-frame overlap reference:
outputs/euroc_v1_01/fusion/30_overlap_16f/pcl_038_061_da3inv_overlap_merged.ply
```

Note:
```text
Older progress entries may show the pre-cleanup flat `fusion/*.ply` paths.
Those files now live under the categorized subfolders above.
```

### Same-frame DA3 context consistency diagnostic

Added:
```text
scripts/compare_da3_same_frame.py
```

Purpose:
```text
Test whether the exact same RGB frame reconstructed inside two different DA3
chunks produces compatible geometry. If the same frame differs between chunks,
then chunk-to-chunk Sim(3) stitching is fundamentally limited.
```

Diagnostic method:
```text
For one shared frame:
1. Load the target chunk's DA3 depth/intrinsics/extrinsics for that frame.
2. Load the source chunk's DA3 depth/intrinsics/extrinsics for the same frame.
3. Backproject both into their own DA3-inverted local worlds.
4. Align source to target using the same-frame DA3 camera pose relation.
5. Also test pose alignment plus a per-frame source->target depth scale.
6. Write colored merged PLYs and a JSON report.
```

Tested current best pair:
```text
target chunk: 38-46
source chunk: 41-49
shared frames tested: 41, 44, 46
```

Example command:
```bash
conda run -n da3 python scripts/compare_da3_same_frame.py \
  --target_da3_npz outputs/euroc_v1_01/da3_038_046/exports/mini_npz/results.npz \
  --source_da3_npz outputs/euroc_v1_01/da3_041_049/exports/mini_npz/results.npz \
  --target_start_frame 38 \
  --source_start_frame 41 \
  --frame_id 44 \
  --rgb_dir outputs/euroc_v1_01/cam0_rgb \
  --output_dir outputs/euroc_v1_01/fusion/50_same_frame_checks \
  --label frame_044_038046_vs_041049 \
  --stride 6 \
  --max_depth 10
```

Results:
```text
frame 41:
  source/target depth ratio median: 1.043478538
  source->target depth scale: 0.960148660
  pose-only NN rmse: 0.111875038
  pose+depth-scale NN rmse: 0.044997705

frame 44:
  source/target depth ratio median: 1.034060399
  source->target depth scale: 0.966470216
  pose-only NN rmse: 0.094449058
  pose+depth-scale NN rmse: 0.036497768

frame 46:
  source/target depth ratio median: 1.035772071
  source->target depth scale: 0.965843876
  pose-only NN rmse: 0.097086789
  pose+depth-scale NN rmse: 0.042759543
```

Visual check targets:
```text
outputs/euroc_v1_01/fusion/50_same_frame_checks/frame_041_038046_vs_041049_pose_aligned_merged_colored.ply
outputs/euroc_v1_01/fusion/50_same_frame_checks/frame_041_038046_vs_041049_pose_depthscale_aligned_merged_colored.ply

outputs/euroc_v1_01/fusion/50_same_frame_checks/frame_044_038046_vs_041049_pose_aligned_merged_colored.ply
outputs/euroc_v1_01/fusion/50_same_frame_checks/frame_044_038046_vs_041049_pose_depthscale_aligned_merged_colored.ply

outputs/euroc_v1_01/fusion/50_same_frame_checks/frame_046_038046_vs_041049_pose_aligned_merged_colored.ply
outputs/euroc_v1_01/fusion/50_same_frame_checks/frame_046_038046_vs_041049_pose_depthscale_aligned_merged_colored.ply
```

Interpretation:
```text
The same RGB frames are not identical across DA3 chunk contexts. The source
chunk depth is consistently about 3-4% larger than the target chunk depth for
the tested frames. Applying a per-frame depth scale improves the same-frame
nearest-neighbor error by roughly 2x to 3x.

This explains why chunk-level Sim(3) merges can look worse than expected: DA3
chunk differences are not only one clean global pose transform. There is also
context-dependent depth scale, and likely some residual non-uniform geometry
difference after scale correction.
```

Project conclusion from same-frame check:
```text
The same frame inside different DA3 chunks does not produce identical depth,
pose-relative geometry, or scale. Therefore, even if two chunks are aligned
with Sim(3), there will still be residual drift or doubled surfaces in the
overlap. The error is partly inside DA3's context-dependent reconstruction, not
only in our merge transform.

Practical consequence:
DA3 chunk fusion should not assume that overlapping chunks are related by one
perfect global Sim(3). We can reduce the error with smaller windows, overlap,
orientation constraints, and per-frame/per-overlap scale correction, but a
pure chunk-level Sim(3) merge cannot make all overlapping surfaces coincide
when DA3 predicts different geometry for the same RGB frames in different
chunks.
```

### Per-frame depth-scale merge test

Next tested whether the same-frame depth-scale correction helps the actual
chunk merge for the best pair:
```text
target chunk: 38-46
source chunk: 41-49
overlap: 41-46
```

Added:
```text
scripts/make_da3_depth_scaled_chunk.py
```

Purpose:
```text
Estimate source->target depth scales from the shared DA3 frames, then rebuild
the source DA3-inverted point cloud after multiplying each source depth map by
its overlap-derived scale. For non-overlap frames, clamp to the nearest overlap
scale.
```

Built per-frame depth-scaled source:
```bash
conda run -n da3 python scripts/make_da3_depth_scaled_chunk.py \
  --source_da3_npz outputs/euroc_v1_01/da3_041_049/exports/mini_npz/results.npz \
  --target_da3_npz outputs/euroc_v1_01/da3_038_046/exports/mini_npz/results.npz \
  --source_start_frame 41 \
  --source_end_frame 49 \
  --target_start_frame 38 \
  --overlap_start_frame 41 \
  --overlap_end_frame 46 \
  --rgb_dir outputs/euroc_v1_01/cam0_rgb \
  --output outputs/euroc_v1_01/fusion/40_overlap_9f/pcl_041_049_da3inv_depthscaled_to_038_046.ply \
  --report_json outputs/euroc_v1_01/fusion/40_overlap_9f/pcl_041_049_da3inv_depthscaled_to_038_046_report.json \
  --scale_mode per_frame_nearest \
  --stride 6 \
  --max_depth 10
```

Applied scales:
```text
frame 41: 0.960148660
frame 42: 0.961668432
frame 43: 0.962484538
frame 44: 0.966470216
frame 45: 0.961439350
frame 46: 0.965843876
frame 47: 0.965843876
frame 48: 0.965843876
frame 49: 0.965843876
```

Output:
```text
outputs/euroc_v1_01/fusion/40_overlap_9f/pcl_041_049_da3inv_depthscaled_to_038_046.ply
points: 40,824
```

Aligned the depth-scaled source two ways:
```text
1. normal center-estimated Sim(3), scale 0.943515259
2. scale-fixed alignment, scale 1.000000000
```

Merged visual diagnostics:
```text
Normal Sim(3) after source depth scaling:
outputs/euroc_v1_01/fusion/40_overlap_9f/pcl_038_049_da3inv_overlap_9f6o_depthscaled_merged.ply

Scale-fixed alignment after source depth scaling:
outputs/euroc_v1_01/fusion/40_overlap_9f/pcl_038_049_da3inv_overlap_9f6o_depthscaled_scale1_merged.ply
```

Compare against previous baselines:
```text
Original 9f6o merge:
outputs/euroc_v1_01/fusion/40_overlap_9f/pcl_038_049_da3inv_overlap_9f6o_merged.ply

Scale-fixed without depth scaling, visually worse:
outputs/euroc_v1_01/fusion/40_overlap_9f/pcl_038_049_da3inv_overlap_9f6o_scale1_merged.ply
```

Interpretation to check visually:
```text
If depthscaled_merged improves the wall-height mismatch, then per-frame depth
scale correction is useful, but still compatible with chunk-level camera-center
Sim(3). If depthscaled_scale1_merged improves more, then the Sim(3) scale was
double-counting the depth correction. If both are worse, then per-frame depth
scale alone is insufficient and remaining rotation/nonuniform geometry is the
dominant error.
```

### End-of-day status and next dataset

Visual inspection after the per-frame depth-scale test:
```text
Best/most useful candidates:
outputs/euroc_v1_01/fusion/40_overlap_9f/pcl_038_049_da3inv_overlap_9f6o_merged.ply
outputs/euroc_v1_01/fusion/40_overlap_9f/pcl_038_049_da3inv_overlap_9f6o_depthscaled_scale1_merged.ply

Less useful:
outputs/euroc_v1_01/fusion/40_overlap_9f/pcl_038_049_da3inv_overlap_9f6o_depthscaled_merged.ply
```

Interpretation:
```text
The original 9-frame/6-overlap Sim(3) merge remains competitive, and the
per-frame depth-scaled + scale-fixed merge is also promising. This suggests
that local depth-scale correction is useful, but stacking per-frame depth
scaling with another global Sim(3) scale can over-correct. The cleaner model is
likely:

1. correct source depths locally/per-frame or per-overlap
2. align chunks with rotation + translation, or tightly constrained scale
```

Major lesson from today:
```text
DA3 chunk overlap errors are not only caused by bad Sim(3) estimation. The same
RGB frame reconstructed in different DA3 chunk contexts can have different
depth scale and residual geometry. Therefore pure chunk-level Sim(3) stitching
cannot guarantee perfect overlap, even when the estimated transform is
reasonable.
```

Checked newly added TUM RGB-D Freiburg1 datasets:
```text
data_local/rgbd_freiburg1/rgbd_dataset_freiburg1_desk
data_local/rgbd_freiburg1/rgbd_dataset_freiburg1_room
```

`fr1/desk` sanity check:
```text
RGB images: 613
Depth images: 595
RGB list entries: 613
Depth list entries: 595
Ground-truth poses: 2335
Accelerometer entries: 11815
RGB format: 640 x 480, 8-bit RGB PNG
Depth format: 640 x 480, 16-bit grayscale PNG
```

`fr1/room` sanity check:
```text
RGB images: 1362
Depth images: 1360
RGB list entries: 1362
Depth list entries: 1360
Ground-truth poses: 4887
Accelerometer entries: 24569
RGB format: 640 x 480, 8-bit RGB PNG
Depth format: 640 x 480, 16-bit grayscale PNG
```

Next recommended work session:
```text
Start with TUM RGB-D fr1/desk. Prepare a small helper that associates RGB,
depth, and ground-truth poses by timestamp, then export short RGB chunks for DA3
plus matching ground-truth/depth metadata. This gives a real RGB test case and
lets us directly measure DA3 depth scale against sensor depth, instead of only
comparing DA3 chunks to each other.
```

### Short finding summary before trajectory-first path

Current DA3 chunk-stitching findings:
```text
1. Individual DA3 chunks can look coherent.
2. Actual merged clouds are the real test; transformed source chunks alone can
   look good while merged overlays are bad.
3. EuRoC grayscale DA3 chunks showed context-dependent same-frame scale drift:
   the same RGB/grayscale frame reconstructed in two chunks differed by about
   3-4% depth scale.
4. Real RGB from TUM fr1/desk improved same-frame consistency, but the actual
   9-frame/6-overlap merge was still bad.
5. Therefore the issue is not only grayscale input. DA3 chunk coordinate gauges
   are still not stable enough for simple DA3-to-DA3 Sim(3) concatenation.
```

Decision:
```text
Move toward the supervisor's trajectory-first approach:
SVO/trajectory owns global placement; DA3 provides local dense geometry inside
trajectory intervals. Overlap is used for local scale/pose correction and
duplicate handling, not as the only global stitching mechanism.
```

Next gate before trajectory-first fusion:
```text
Verify SVO trajectory quality by aligning SVO to ground truth with Sim(3) and
reporting ATE-style error. If SVO is stable enough, use it as the global path
for placing DA3 interval reconstructions.
```

### SVO trajectory sanity check against V1_01 ground truth

Added:
```text
scripts/compare_trajectory_to_gt.py
```

Purpose:
```text
Align an estimated TUM-format trajectory to a ground-truth TUM-format trajectory
with Sim(3), then report ATE-style position error.
```

Command:
```bash
python scripts/compare_trajectory_to_gt.py \
  --estimate outputs/svo_v1_01_clean/svo_pose_cam_tum.txt \
  --groundtruth svo_ws/src/rpg_trajectory_evaluation/results/euroc_vislam_mono/laptop/vislam_ba/laptop_vislam_ba_V1_01/stamped_groundtruth.txt \
  --max_dt 0.01 \
  --report_json outputs/svo_v1_01_clean/svo_vs_gt_sim3_report.json \
  --aligned_tum outputs/svo_v1_01_clean/svo_pose_cam_aligned_to_gt_tum.txt
```

Result:
```text
matched poses: 459 / 459
time span: 22.900 s
timestamp max dt: 0.000000000

estimated path length: 8.179511891
ground-truth path length: 7.502642636
estimate/GT path ratio before alignment: 1.090217446

Sim(3) scale, SVO -> GT: 0.990382414

ATE after Sim(3):
rmse:   0.074960917 m
median: 0.072111152 m
mean:   0.069553772 m
p95:    0.114548589 m
max:    0.144877287 m
```

Outputs:
```text
outputs/svo_v1_01_clean/svo_vs_gt_sim3_report.json
outputs/svo_v1_01_clean/svo_pose_cam_aligned_to_gt_tum.txt
```

Interpretation:
```text
The clean SVO V1_01 trajectory is usable as a global path for the next
trajectory-first fusion test. The current check is position-only Sim(3), so it
is a sanity gate rather than a full visual-inertial calibration/evaluation, but
the roughly 7.5 cm RMSE over the available 22.9 s segment is good enough to
proceed with SVO-owned global placement experiments.
```

### Repository cleanup and current best-result summary

Current best results to show externally:
```text
SVO trajectory sanity:
outputs/svo_v1_01_clean/svo_vs_gt_trajectory_compare.png
outputs/svo_v1_01_clean/svo_vs_gt_sim3_report.json

Best larger-window DA3 overlap:
outputs/euroc_v1_01/fusion/30_overlap_16f/pcl_030_045_r448_da3inv_aligned_to_022_037_overlap.ply

Best smaller-window DA3 overlap:
outputs/euroc_v1_01/fusion/40_overlap_9f/pcl_038_049_da3inv_overlap_9f6o_merged.ply

Same-frame context diagnostic:
outputs/euroc_v1_01/fusion/50_same_frame_checks/frame_044_038046_vs_041049_report.json
```

Supervisor email package:
```text
outputs/email_supervisor_current_findings_2026-05-22/
outputs/email_supervisor_current_findings_2026-05-22.zip
```

Main interpretation:
```text
1. SVO V1_01 trajectory is good enough to use as a global path after Sim(3)
   sanity checking against ground truth.
2. DA3 local chunks can be coherent and small DA3-to-DA3 overlap windows can
   align visually.
3. DA3-to-DA3 stitching does not scale reliably to the full sequence.
4. The same RGB/grayscale frame can reconstruct differently in different DA3
   chunk contexts, so the issue is not only Sim(3) alignment error.
5. Full-scene DA3 overlap-chain attempts are retained as negative diagnostics,
   not as current best results.
```

Recent full-scene negative diagnostics:
```text
outputs/euroc_v1_01/fusion/70_overlap_chain_whole_scene/pcl_022_099_da3inv_overlap_chain16_newframes_merged.ply
outputs/euroc_v1_01/fusion/70_overlap_chain_whole_scene/chain_9f6o/pcl_022_099_da3inv_overlap_chain9f6o_newframes_merged.ply
```

Next technical direction:
```text
Before more full-scene fusion, verify DA3 depth-ray/camera-frame conventions
against SVO poses and intrinsics. Then continue with the supervisor's
trajectory-first approach: SVO owns global placement, DA3 supplies local dense
geometry, and overlaps are used for local scale/depth correction rather than as
the only global stitching mechanism.
```
