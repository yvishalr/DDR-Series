# Differential Drive Simulation

This project is the a compilation of differential drive robot simulations made for learning and understanding the kinematic equations for a differential drive robot.

It also contains a simulink blockset used to graph the step response of a well-tuned DDR.

![](/resources/simulink%20model.png)

## differentialDriveFK.m

This script plots out an animation for the forward kinematics of a two wheel differential drive robot. The physical parameters of the bot must remain fixed and the path it takes must be pre-defined before running the script.

Physical parameters of the bot can be changed by altering the `axleLength` and `radius` (i.e, wheel radius) parameters.
Velocities for individual wheels can be set under the variables `rightVelocity` and `leftVelocity`.

Initial pose (position and orientation) of the bot can be changed using `initialTheta`, `initialX` and `initialY` parameters.

The `time_run` and `del_t` variables responsible for the simulation. The former defines the total duration of the simulation while the latter defines the timesteps where plotting actually occurs to produce an animation.

After all these definitions, the major components are within the for loop.
`omega` represents the angular velocity of the DDR whereas,
`RICC` is the _radius of the instantaneous centre of curvature_ and must be computed at every timestep. Two cases arise during this calculation which must be accounted for while trying to determine the pose of the bot at the next iterations.

##### Case 1 : When RICC is finite

The `differentialDriveTransformation` function is called and calculates the next pose.

##### Case 2 : When RICC becomes infinite (rightVelocity = leftVelocity)

This causes the `Rotate` matrix to cause an **Inf** condition while trying to calculate the pose under the following lines

```Matlab
Rotate = [cos(w*t) -sin(w*t) 0;sin(w*t) cos(w*t) 0;0 0 1];
poseICC = [a-ICC(1);b-ICC(2);theta];

FinalPose = Rotate*poseICC + [ICC(1);ICC(2);w*t];
```

Hence a seperate condition is specified within the for loop. This block contains linear transforms to produce the forward kinematics at the following timestep as shown below :

```Matlab
if -pi/2 < initialTheta && initialTheta < pi/2

x1 = p(1)+iterate;
y1 = m*x1+c;

elseif initialTheta == pi/2

x1 = p(1);
y1 = p(2)+ iterate;

elseif initialTheta == -pi/2

x1 = p(1);
y1 = p(2)-iterate;

else

x1 = p(1)-iterate;
y1 = m*x1+c;

end
```

#### Defining a path

The path can only be changed by altering the right and left wheel velocities at any given timestep. An example is provided below :

```Matlab
% Add required path changes at desired timestep here
 if(i == del_t*25)
     temp = rightVelocity;
     rightVelocity = leftVelocity;
     leftVelocity = temp;
 elseif(i == del_t*45)
     rightVelocity = leftVelocity;
 elseif(i == del_t*80)
     leftVelocity = leftVelocity + 2.5;
     rightVelocity = rightVelocity + 1.5;
 end
```

Which produces the following birds eye view plot :

![](/resources/Forward%20Kinematics%20Plot.gif)

## stepResponseDDR.slx

This is a simulink blockset that outputs the system response of a generic two wheel DDR on the scope block. If the physial parameters are changed, re-tuning of the PID values will be required.

The step input example is displayed below :

![](/resources/Step%20Response.gif)

This simulink file also outputs coordinate values into the `states` array.
The **simulinkDDRPlotter.m** script can be run to visualise this as a 2-D plot.
