# Frenet Local Path-Planning

Generation of Local Path using Frenet Frame on a Global Path using Spline Interpolation of Waypoints given.

## Requirments

```bash
pip install numpy
pip install opencv-python
```

## Installation

```bash
git clone https://github.com/mradul2/frenet-planner.git
```

## Usage

### Generation of Global Path

Cubic Spline Transformation is used to Generate Waypoints for a Global Path.

```bash
python3 spline.py
```

![Spline](/assets/spline.png)

### Detection using OpenCV

OpenCV functionalities of contour detection are used to find the position of Ego Vehicle and Waypoints which are present on the Target Image.

```bash
python3 detection.py
```

![Detection](/assets/detection.png)

### Generation of a Frenet Path

Using Frenet frame method, here we generate the Frenet trajectories along our global path which we have determined earlier.

```bash
python3 planner.py
```

<img src="assets/planning.png" alt="planning" width="600"/>
<img src="assets/frenet.png" alt="frenet" width="600"/>

## Refrences

Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame: [Link](https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame)
