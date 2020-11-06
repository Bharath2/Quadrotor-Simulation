## Quadrotor simulation

### This project contains:
- Informed RRT* to plan a collision free path from start to goal position in an N-dimensional Map.
- A minimum snap trajectory generator. The segment times between consecutive waypoints are also optimised.
- Nonlinear geometric controller for aggressive trajectory tracking and hover controller
- Quadrotor Simulator

### Usage

### Future Work

- Adding disturbance to quadrotor model
- Collision checking after trajectory generation

### References
- [Polynomial Trajectory Planning for Aggressive Quadrotor Flight in Dense Indoor Environments - Charles Richter, Adam Bry, and Nicholas Roy](https://groups.csail.mit.edu/rrg/papers/Richter_ISRR13.pdf)
- [Minimum Snap Trajectory Generation and Control for Quadrotors - Daniel Mellinger and Vijay Kumar](https://arxiv.org/pdf/1706.06478.pdf)
- [Motion Planning as Search - Russ Tedrake](http://underactuated.csail.mit.edu/planning.html#section2)
