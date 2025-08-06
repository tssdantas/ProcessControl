## Process Control Implementations

This repository contains a porfolio of process control simulations created to showcase some of my knowledge in this field.

## Contents
- ### Model predictive control with linear space-state models.
    - [IHMPC for Zone Control](https://github.com/tssdantas/ProcessControlPortfolio/tree/main/ZoneControlIHMPC) In some chemical processes, the exact value of the controlled outputs is not important, as long as they remain within specified boundaries, or zones. This is the controler defined in problem P1b in the article "Alvarez., L.A., Odloak, D. Robust integration of real time optimization with linear model predictive control. Computers and Chemical Engineering v.34, p. 1937–1944, 2010.". In addition to Zone Control, it also acccept targets for inputs and other mathematical properties found in the controller proposed in Odloak, 2004, such as recursive feasibility, proof of stability and the outputs will allways converge to values inside the zones.
    
    - [IHMPC](https://github.com/tssdantas/Process_Control_Portfolio/tree/main/IHMPC) An algorithm in MATLAB was built from scratch to implement an Infinite Horizon Model Predictive Controller (IHMPC) presented in the P1 problem of the paper "Odloak, D., 2004. Extended robust model predictive control. AIChE Journal, 50(8), pp.1824-1836.". The main features of this linear controller with restrictions are: 1) It's based on space-state model in velocity form, hence its offset free. 2) Recursive Feasibility: once a feasible input sequence is computed its guaranteed that subsequent optimization computations will remain feasible 3) Convergence properties: The control law obtained in the solution of the optimization problem will always drive the outputs to its reference signal. 4) Formal Proof Stability was presented in paper, hence the stability of the closed loop system is not dependent on tunning parameters 5) Large domain of attraction due to softing of the constraints with slack variables. Simulation with a MIMO Transfer Function model is presented.
    - [DMC](https://github.com/tssdantas/Process_Control_Portfolio/tree/main/DMC) An algorithm in MATLAB was build from the ground up to implement a classic model predictive control approach known as Dynamic Matrix Control. It is based on a linear multi-input multi-output (MIMO) step-response based space-state model. This approach was proposed by Cutler and Remaker, 1980, in the article "Dynamic matrix control – a computer control algorithm". This code computes the optimum control sequence for the case where the process doesn't have restrictions.
             
## Licensing information for this project.

This code is licensed under GNU General Public license v3.0

## Contact information

Any questions should be directed to Tarcisio S. S. Dantas at tssdantas@gmail.com
