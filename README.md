# Model Predictive Control of Steering Angle and Throttle
This is a solution to the Udacity Car-ND MPc project. The starter code provided at the following link was used:
https://github.com/udacity/CarND-MPC-Project

## Model 
The controller used a kinetic model to describe the state nad actuation of the car. The state of the car was determined by its x, y position, orientaiton, velocity, crosstrack error, and orientation error. The steering angle and throttle of the car was predicted using these state variables. The model used for the project was very similar to the one provided in the solution for the mpc quiz, except for certain specifics described in the subsection below.

The state equations are provided in MPC.cpp and are the following:

```C++

fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
fg[2 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
fg[2 + epsi_start + i] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);

```

### Tuning the Model specifics
The contraints on the model were kept the same as the ones in the quiz solution. The one change that was made was that the steering angle was contrained between -1 and 1, instead of -deg2Rads(25) and deg2Rads(25). By doing so, no conversion was needed while passing values to the simulator.

The objective function for the model was tuned ot obtian optimal results. This function had three parts:

1. **State Cost**: This incorporated cost of changes in crosstrack error (cte), heading error and speed error. The three costs were weighted differently to assing relative importace. The main goal of the model is to drive at the center of the track. For that reason a high weight of 200 was used with cte and heading cost. The speed error cost term was used to not have the vehicle stop if it was perfectly centered. However, reaching and staying at maximum speed was not that important. Therefor a low cost of 0.1 was used.

2. **Actuation cost**: This pasrt was used to minimize the use of actuation.

3. **Cost of changing actuation**: To prevent the car from suddently acelerating and changing the steering angle abruptly, the cost dependent on changes in actuation was added. A high weight of 500 was used for steering angle change to force the car to make smoother turns and prevent it from swinging on the track.  

The following code implementing the above costs is in MPC.cpp:

```C++
fg[0] = 0;

// Reference State Cost
for (int i = 0; i < N; i++) {
  fg[0] += 200. * CppAD::pow(vars[cte_start + i] - ref_cte, 2);
  fg[0] += 200. * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
  fg[0] += .1 * CppAD::pow(vars[v_start + i] - ref_v, 2);
}

// Minimize the use of actuators.
for (int i = 0; i < N - 1; i++) {
  fg[0] += CppAD::pow(vars[delta_start + i], 2);
  fg[0] += CppAD::pow(vars[a_start + i], 2);
}

// Minimize the value gap between sequential actuations.
for (int i = 0; i < N - 2; i++) {
  fg[0] += 500. * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
  fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
}
```

## Receding Horizon

The MPC is used to predict the state of the system in the following "N" steps based on the current state. The first step of actuation is performed and the predition is carried out again for the next "N" steps. Thus the horizon of the controller is changing. After trying different combination of number of steps (N) and time differnece between states (dt), I settled on N = 20 and dt = 0.1s. 

My target was to get the car to a maximum of 60mph and these values provided a good balance. At that speed 2sec horizon was useful in fitting the way points around a curved section of the track an optimal amount of time before the car had to actually turn. I tried smaller dt values and found the car behaving erractily at curves and then seinging from one end of the track to another.

## Polynomial fitting and Preprocessing

The simulator sends way points and state of the car to the controller, to be used for prediction. The way points can be used to calculate the coefficients of the polynomial for the path the car needs to follow. As the cross track and heading errors are not provided by the simulator, they need to be computed. These errors are much easier to calculate if the equation of the path is in car's local coordinate system, with the x-axis in the direction of car's heading and y-axis to the left of it. 

The way points sent by the simulator are in the global coordinate system. The following equations in main.cpp were used to convert them to car's coordinates.

```C++

for (int i = 0; i < ptsx.size(); i++) {
  way_x.push_back((ptsx[i] - px)*cos(psi) + (ptsy[i] - py)*sin(psi));
  way_y.push_back(-(ptsx[i] - px)*sin(psi) + (ptsy[i] - py)*cos(psi));

}

```

Once the way points were converted to local coordinates, a third degree polynomial fit was calculated.

## Latency

The controller had to account for the latency in the car's system as observed in real world. In real world scenario, changes in actuation (steering angle and throttle) take a certain amount of time to take effect. This results in actuation intended to apply at the current time to be appiled after a certain delay. This is the latency in the system. Humans are very efficient in learning is latency and incorporate in their control without specifically having to think about it at each step. A computer based controller is naive to such situations. In this project a 100ms latency had to be accounted for. 

To accomplish this, I again went back to the kinetic model and used it to predict the state of the car after the latency period. This predicted state was then used as an inout to the solver. The solver was thus able to precit the actuation after the latency period. This actuation was passed to the simulator. The calculation of state after latency is in the following piece of code in main.cpp:

```C++
// Predict state after latency before passing to the solver
double dt = 0.1;
px = v * dt;
psi = -v * steer_angle * dt / 2.67;

double cte = polyeval(coeffs, px);
double epsi = -atan(coeffs[1] + 2 * px * coeffs[2] + 3 * px * px * coeffs[2]);

std::cout << "CTE: " << cte << std::endl;
std::cout << "Epsi: " << epsi << std::endl;

// State to initialize the solver
Eigen::VectorXd state(6);
state << px, 0.0, psi, v, cte, epsi;

```

## Results
This implementation was able to smoothly run the car around the track achieving max speed close to 60mph. The model accelerated on straight sections of the track and braked on sharp turns to slow the car down.  
---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
