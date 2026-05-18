### Quick useful commands
- get short 3 level depth repo structure
```text
tree -a -L 3 -I '.git|__pycache__|*.pyc'
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
  - pytorch
  - torchvision
  - torchaudio
  - pytorch-cuda=12.4
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