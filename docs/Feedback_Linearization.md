# Feedback Linearization 
Feedback Linearization is a non-linear control technique used for non-linear systems. The rotational physics of a quadcopter are described by [Euler's Equation](https://en.wikipedia.org/wiki/Euler%27s_equations_(rigid_body_dynamics)) for Rigid Bodies and are non-linear. As can be shown from the equation, the axes are coupled to one another and rotation about two axes of quadcopter creates a torque on the third axis. A PID controller is a linear controller and isn't designed for this kind of system. However, it is seen that in many practical applications a PID controller with gain scheduling (TPA, Antigravity, EMUBoost etc) can provide sufficient performance. However, feedback linearization makes a non-linear system look linear from the perspective of the controller allowing use of a linear controller on a non-linear system. The hypothesis is that feedback linearization will improve performance of the quadcopter in situations with rotational velocties on multiple axis such as coordinated turns, split-s turns, cross compensated rolls/flips, hard 180 turns etc. Furthermore, it should help with propwash since propwash often shows up on multiple axes at the same time.

The code is designed so that physical parameters of the quadcopter don't need to be directly known. The physical parameters are described instead by non-dimensional constants which may be tuned intuitively by someone who isn't academically trained in the dynamics of a quadcopter. 

## Physical Constants
There are 5 parameters that need to be calculated:
1. torqueInertia ratio
2. pitchInertia ratio
3. yawInertia ratio
4. pitchTorque ratio
5. yawTorque ratio
The Torque to Inertia ratio describes the ratio of the maximum torque on the roll axis to it's moment of inertia. This is similar to the concept of thrust to weight ratio.
The 2nd two constants are the pitchInertia and yawInertia ratios. They describe the relative size of the pitch and yaw moments of inertia with respect to the roll moment of inertia. Due to the elongated body and typical distribution of mass along the pitch axis, The pitchInertia and yawInertia ratios are almost always greater than one with the yaw ratio being larger than the pitch ratio \(1\<kp\<ky).
The last two parameters are the pitchTorque and yawTorque ratios describe the relative abilities of the motors to produce torque along a given axis. This depends on the motor geometry. A stretch-X frame will have a pitchTorque ratio greater than one but a wide-x frame will have lower ratio. The yawTorque ratio is often less than one since the propeller drag constant is less than the propeller thrust constant but the value for it may be determined from a simple hover test as long as the the flight controller has a current meter.

The torqueInertia ratio may be multiplied by the corresponding inertia and torque ratio's for the other axis to calculate their torque to inertia ratios:
torqueInertiaPitch = torqueInertia \* pitchTorque / pitchInertia

## Calculating the Parameters
It is assumed that the frame is symmetrical ie\(any X or H design) and that the motor layout and weights have the largest effect on moments of inertia. Note that this is a rough estimation only. The estimation can be improved by acounting for the inertia due to the battery, body, HD camera, and electronics. If one has a 3d model of their exact build they could use a 3d modelling program to directly calculate the moments of inertia. That would give the most accurate results.

There are several measurements that need to be taken:
1. W is the width of the motor layout in millimeters
2. L is the length of the motor layout in millimeters
3. m is the mass of a motor + propellor + mounting in grams
4. M is the all up weight of the quad in grams

1. TWR is the thrust to weight ratio of the quad

Perform a hover test and obtain the following pieces of information:
1. Average current at hover
2. Average motor RPM at hover

The parameters are calculated as follows
1. torqueInertia = 
2. pitchInertia = L^2 / W^2
3. yawInertia = = \(L^2 + W^2) / W^2
4. pitchTorque = L / W
5. yawTorque = 