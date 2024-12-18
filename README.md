# qcm-cbf
An implementation of the robot navigation using Quasi-conformal Mappings (QCM) and Control Barrier Functions (CBF) approach described in [Notomista, Choi, and Saveriano, 2024](https://arxiv.org/abs/2411.14908).

## Demos description
- `demo_one_obstacle.m`: a demo to run QCM-CBF (full mapping) to avoid a single obstacle.
- `demo_full_qc_incremental.m`: a demo to run QCM-CBF (full mapping) to avoid a two obstacles dynamically added to the workspace.
- `demo_partial_qc_incremental.m`: a demo to run QCM-CBF (partial mapping) to avoid a two obstacles dynamically added to the workspace.
- `demo_pan_tilt_camera.m`: a demo to run QCM-CBF (full mapping) to avoid unsafe regions observed by a Pan-Tilt camera.
- `demo_office_navigation.m`: a demo to run QCM-CBF (full mapping) to avoid collisions in a complex office space.


## Software Requirements
The code is developed and tested under `Matlab2023b`.

## References
Please acknowledge the authors in any academic publication that used parts of these codes.
```
@article{Notomista2023Reactive,
  title={Reactive Robot Navigation Using Quasi-conformal Mappings and Control Barrier Functions},
  author = {Gennaro Notomista and Gary P. T. Choi and Matteo Saveriano},
journal = {IEEE Transactions on Control Systems Technology},
volume = {},
pages = {},
year = {2024 (to appear)}
}
```

## Note
This source code is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY.
