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