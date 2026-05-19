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