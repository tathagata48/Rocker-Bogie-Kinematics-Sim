
# 🤖 Mars Rover Kinematics and Dynamics Simulation (Rocker-Bogie vs. Solid Axle)

A comprehensive MATLAB simulation analyzing the comparative kinematics and dynamics of Rocker-Bogie and Solid Axle rover suspensions during obstacle traversal. The analysis focuses on critical performance metrics like chassis pitch angle and front wheel vertical force distribution, providing a detailed look at chassis stability under load.


# 🛠️ How to Run the Simulation

## Prerequisites

MATLAB: Ensure you have MATLAB installed (R2018a or later is recommended).

Toolboxes: The script requires the Optimization Toolbox for the fsolve function used in the Rocker-Bogie kinematic analysis.

## Execution

Download: Clone this repository or download the RockerBogieKinematics.m file.

Open: Launch MATLAB and open the RockerBogieKinematics.m script.

Run: Click the Run button in the MATLAB editor, or type RockerBogieKinematics in the Command Window.

The script will execute the three-phase simulation and automatically display the resulting figure, showing the rover configuration at the three critical points of obstacle traversal.
## ⚙️ Key Parameters

The physical parameters of the rover and obstacle can be easily modified in SECTION 1 of the MATLAB script:

| Parameter | Variable| Default Value | Description |
|---|---|---|---|
| Wheel Diameter| L_wd| 0.5m | Diameter of the rover wheels. |
| Rocker Length | L_r | 0.72m | Length of the main rocker link. |
| Wheel Center Spacing | L_span | 0.8m | Horizontal distance between wheel centers (F-M and M-R). |
| Obstacle Height | H_obs | 0.3m | The height of the step obstacle to be traversed. |

# 📚 Simulation Methodology

## Rocker-Bogie Kinematics

The Rocker-Bogie system's movement is solved using an Inverse Kinematics approach based on two fundamental distance constraints for the chassis pivot point P:

Constraint 1: ||P' - M'||^2 = L_r^2 

Constraint 2: ||P' - F'||^2 = L_PF^2

Where:

P' is the chassis pivot point.

M' and F' are the centers of the middle and front wheels, respectively.

L_r is the Rocker Link length.

L_PF is the fixed distance between the chassis pivot P' and the front wheel pivot F'.

The MATLAB fsolve function is used to numerically determine the coordinates of P' that satisfy these two non-linear equations in each phase.

## Traversal Phases

The simulation breaks the obstacle traversal into three critical phases:

## Phase 1 (F up): 
The front wheel ($F'$) is on top of the step, while the middle ($M'$) and rear ($R'$) wheels are on the ground.

## Phase 2 (M up): 
The front and middle wheels ($F'$ and $M'$) are on the step, and the rear wheel ($R'$) is on the ground.

## Phase 3 (R up): 
All three wheels ($F', M', R'$) have successfully climbed and are now resting on the top of the step.

 <img width="1580" height="775" alt="image" src="https://github.com/user-attachments/assets/e5d53f01-55d4-426e-9166-91db2cb5297a" />

