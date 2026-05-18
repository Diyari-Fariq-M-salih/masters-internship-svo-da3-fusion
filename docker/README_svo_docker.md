# SVO Docker Environment

SVO Pro is kept separate from the host and from the DA3 Conda environment.

## Reason

SVO Pro is tested on:

- Ubuntu 18.04 + ROS Melodic
- Ubuntu 20.04 + ROS Noetic

The lab machine uses a newer Ubuntu/kernel setup and has ROS2-related packages, so building SVO directly on the host may create dependency conflicts.

## Planned architecture

- DA3 runs on host in Conda env `da3`
- SVO runs in Docker with Ubuntu 20.04 + ROS Noetic
- Fusion scripts run in the main project repo

## File-based interface for prototype

SVO Docker should output:

```text
outputs/svo/
  svo_traj_raw.txt
  keyframes.txt
```
DA3 should output:
```
outputs/da3/
  depth.npy
  intrinsics.npy
  conf.npy
```

Fusion reads both and produces:
```
outputs/fusion/
  pcl_000_015.ply
  trajectory.txt
```

### Notes

The upstream SVO dependency file uses SSH GitHub URLs. For reproducibility, this project keeps an HTTPS version at:

patches/svo/dependencies_https.yaml