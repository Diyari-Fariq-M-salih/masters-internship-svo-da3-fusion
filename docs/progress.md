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