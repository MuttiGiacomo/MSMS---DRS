# Modelling and Simulation of Mechatronics Systems course 
## Drag Reduction System - Analisys and Optimization

Authors:
@[muttigiacomo](https://github.com/muttigiacomo)
@[mastrogiuseppematteo](https://github.com/matteomastrogiuseppe),
@[riccardoperiotto](https://github.com/riccardoperiotto),

![Alt Text](https://github.com/MuttiGiacomo/MSMS---DRS/blob/main/media/pull.gif =250x250)
![Alt Text](https://github.com/MuttiGiacomo/MSMS---DRS/blob/main/media/rocker.gif =250x250)
![Alt Text](https://github.com/MuttiGiacomo/MSMS---DRS/blob/main/media/pull.gif =250x250)

This project focuses on the mechanical analysis and optimization of 3 different Formula 1 Drag Reduction System mechanisms. 

Key words: 
QFD, constraint equations, Newton_Euler equations, Lagrange equations, virtual work principle, cost function, control

### Engineering specification
To gather the requirements we needed for the development of the project we followed what’s called the Quality Function Deployment (QFD). The name refers to a process and a set of tools used to effectively define customer requirements and convert them into detailed engineering specifications and plans to produce the products that fulfill those requirements. The tool we mainly used is the “House of Quality” through which we identified the specifications we should focus on: 
 - opening time
 - weight 
 - force generated by the piston
 - piston stroke

### Kinematic and Dynamic analysis
The kinematic analysis performed on the three mechanisms is divided into three sub-studies: position analysis, inverse kinematics and velocity analysis. Results are further developed to gain more informations such as velocity ratios, working space and inverse kinematics.
Dynamic analysis is performed in 3 different ways which lead to the same solution: Neuton-Euler equations, Lagrange equations and virtual work theorem.

### Optimization
Optimization is performed minimizing a global cost function composed many singular cost functions with different weights depending on each specificaion's importance. 
Using this approach The new mechanism has 40% shorter piston stroke and a more velocity profile while requiring acceptable maximum force and weight.

### Further analysis 
- Control loop
Position and velocity control loops were implemented on the piston force making the system follow a predefined profile
- Hydraulic system simulation
An hydraulic system provided by the professor was utilized to simulate pump flow, the pump pressure and valve cross section chenging over time as the piston follows the desired profile

