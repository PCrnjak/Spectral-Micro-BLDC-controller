

## What is Gravity Compensation?

Gravity compensation is a fundamental concept in robotic control that helps counteract the effects of gravity on a robot's joints and links. By applying the necessary motor current to balance this torque, the robotic joint can remain stationary at any position without additional external forces.

This blog post will cover a lot of theory on how gravity compensation works and by the end of the blog there will be a guide on how to replicate the demo from the video. For code explaination and pseudo code go to the bottom of the blog.

## Why is it so important?

In robotics, a significant portion of motor power is consumed just fighting against gravity. Gravity compensation algorithms are standard in all industrial robots and collaborative robot arms because they provide: improved position accuracy and control, smoother motion planning, and more precise force control applications. The gravity compensation algorithm works as a feedforward component to the motor PID control loops, reducing their computational load and improving overall performance.

## The Approach

In this example, we use the Robotics Toolbox for Python to model a "robotic arm" with a single revolute joint. We compute the required torque using the Recursive Newton-Euler Algorithm (RNEA) and convert this torque into a motor current/torque that the motor controller will command the motor. In this blog post we will explain in short pseudo code snippets how the algorithm works on a single DOF arm. 

### Dynamics equations

Joints and links experience forces and torques due to:

* Gravity: Always acting on the robot, depending on its configuration.
* Inertia: Required to accelerate or decelerate the joints.
* Coriolis and Centrifugal Effects: Arise from the coupling between velocity and the mass distribution of the links.
* External Forces and Torques . ie. impacts with the world and environment, human touching the robots...


The equation for applied joint/motor torque wourld look like this: 
τ=M(q)q¨+C(q,q˙)q˙+g(q) + τ_external

Where:
* τ - vector of applied joint torque
* q - joint position
* q˙ - joint velocity
* q¨ - joint acceleration
* g - torque due the gravity (function of q)
* M - Inertia matrix fucntion of joint coordiantes. Multiplied by joint acceleration gives the torque needed to accelerate out joint
* C - Coriolis & centripetal term (function of positon and velocitcy); gives torques that act on a joint because of rotation and movement of other joints of the robot
* τ_external - external torque

In our case since we want to compensate gravity term only our equation will look like this:

τ = g(q)

NOTE that this equation will hold the joint/robot in the same positon UNTIL we apply τ_external

NOTE there would also be motor rotor dynamic parameters and fristion but we are also ignoring them

Where we know the positon and model of our motor and system and we want to apply the torque to our motor based on that systems position so that it cancels the gravity.
Meaning that our Commanded current will be:
Iq_command = -τ = -g(q)


#### Coriolis/centrifugal force and Inertia

Why do we not include other compensations?
You would include Inertia and Coriolis/Centrifugal compensation when commanding the robot to move dynamically, such as when you're:

* Moving at a Specific Speed: Counteracting dynamic effects ensures the robot can maintain the desired speed without being affected by forces like Coriolis, centrifugal, or gravity.
* Tracking a Trajectory: Ensuring smooth and accurate motion from one position to another, especially for high-speed or high-precision tasks.

ALSO Coriolis and centripetal forces do NOT exist in a 1-DOF system! Here is why:

Coriolis Effect Occurs when one joint's motion affects another joint's motion and Requires AT LEAST 2 DOF to exist

Centripetal Force: In robotics context, also requires multiple DOF. These forces emerge from the coupling between joints and Cannot occur with single rotational or linear motion alone


#### Recursive Newton-Euler Algorithm (RNE function)

τ = rne(q, qd, qdd, grav) is the joint torque required for the robot to achieve the specified joint position q (1xn), velocity qd (1xn) and acceleration qdd (1xn), where n is the number of robot joints.

τ=M(q)q¨+C(q,q˙)q˙+g(q)

Inputs:
* radians_position:  joint angle in radians.
* radians_per_second:  joint velocity in radians per second.
* joint_acceleration:  joint acceleration in radians per second squared.

This will return the total torque required to:

* Balance the gravitational forces.
* Counteract Coriolis and centrifugal effects.
* Accelerate or decelerate the joint. (fight Inertia)

Standard Practice in Industry: Gravity compensation is always calculated using the current joint positions because gravity depends directly on the robot's actual configuration, not the commanded one.
Reason: Gravity torque depends only on the current configuration of the robot, as it's a static force

Dynamic Effects (Velocity and Acceleration)
For Coriolis & centrifugal  effects, you use current positions and current velocity (can also use commanded velocity).
Reason: These forces emerge from the current motion state and are velocity-dependent


For Inertial Forces you use current positon and commanded acceleration.
Reason: Inertial forces are computed as feedforward terms, so we use desired/commanded accelerations from the trajectory planner


**Position (ALWAYS CURRENT VALUE) :** Most reliably measured signal, Gravity is often the largest force to compensate, Critical for safety and collision detection, Position sensors (encoders) are typically very accurate


**Velocity (CAN USE EITHER):**

Commanded: Smoother, no noise, Good for high-speed motions, Works well when tracking is good

Current:Better for precise force control, More accurate at low speeds, Better for detecting disturbances, Can be noisy (especially if derived from position)

**Acceleration (USUALLY COMMANDED):**

Commanded is most common because:Acceleration is rarely directly measured, Derived acceleration is very noisy, Feedforward nature of inertial compensation, Available from trajectory planner




## Our setup 

### Our test jig

### Modeling the jig in CAD
 
Designing the Jig in CAD
We began by modeling our jig in a CAD software (SolidWorks). This allowed us to visualize and refine the design before moving to the next stages.

Creating the Kinematic Diagram
Once the jig design was complete, we created a kinematic diagram of the model. We defined a reference frame at the end of the arm, referred to as the link frame, which served as the basis for all subsequent calculations.

3D Printing and Assembly
With the design finalized, we 3D-printed the jig parts and assembled them. For the weight component, we used a NEMA14 motor, ensuring the system was both functional and representative of the intended application.

Measuring the Mass
After assembly, we measured the jig’s total mass by placing it on a scale. This step provided a crucial data point for validating our CAD model and calculating physical parameters.

Determining Mass Properties
Using SolidWorks’ Mass Properties tool, we calculated critical values such as the center of mass and inertia parameters.

It’s essential to select the correct reference frame—in our case, the previously defined link frame—to ensure accurate calculations.
Key Values for System Modeling
To accurately model the system, we needed the following parameters:

Mass (measured directly and verified in CAD).
Center of Mass (from the CAD model).
Link Length (defined in the design).
Inertia Tensor (calculated using the CAD software).
Motor Parameters
For the motor, the critical value we required was the Kt (torque constant). Using the Spectral Micro, this value can be calculated automatically during the motor's calibration process.


### Modeling errors

The performance of this kind of system depends on how preciselly you model it.
The system WILL have modeling errors!
Some of the errors that can appear are:
* Wrong mass
* Wrong center of mass
* Wrong motor initial positon and thus the system thinks it is in a position it is not.
* Motor Kt value (Different a bit for each "same" motor, it is also at some point non linaer and depends on temperature...)
* Friction
* Undefined motor dynamics


## Hardware
* CANvas adapter
* Spectral micro BLDC driver 
* 24V PSU
* Wires to connect everyting
* PC

These are some esential you will need to have. Other than that you can design your own jig and extract your  Mass, Center of mass, link length, Inertia tensor from the model of your custom jig. If you want to replicate ours check this BOM!


## The algorithm


Prerequisites:
* We have Mass, Center of mass, link length, Inertia tensor values
* We have assemled our jig
* Note at what encoder position the motor encoder is when it is unpowered that is our zero_pos value.


Steps:
* We define motor object and parameters in the code
* Cretea a robot object with DH parameters
* Create a timer that will trigger every 5ms
* Send Current/Torque command to the motor (Commanded_current value)
* Once we get a response from the motor unpack that and extract position of the motor from that
* Run RNE function: Grav_load = robot.rne([radians_position],[0],[0])  
* Calculate Commanded_current value: T = Kt * Iq; Iq = T / Kt and Iq is our Commanded_current value
* Loop

That is it in the nutshell. It is imporatnt for this code to run at at least 10ms loop time. If it is slower for example 50ms it will become unstable and not work. If there is some drift common problems are: slightly different Kt value or slightly incorect zero_pos value. 


## The Pseudocode

```python 
L1 = RevoluteDH(a=l1, m=m1, r=com, I = I, Jm = Jm, B = B, Tc = Tc)
robot = DHRobot([L1], gravity=[0, g, 0])

Communication1 = Spectral.CanCommunication(bustype='slcan', channel='COM32', bitrate=1000000)
Motor1 = Spectral.SpectralCAN(node_id=0, communication=Communication1)

#We create a timer that will activate every 5 ms.

#Now to the main loop in pseudo code:

while(1):

    Motor1.Send_data_pack_1(Position=None, Speed= None, Current = Commanded_Current)

    message, UnpackedMessageID = Communication1.receive_can_messages(timeout=None) 

    if message is not None:
        

        Motor1.UnpackData(message,UnpackedMessageID)


        # Convert the received position to radians and offset everything to match our DH diagram
        # - 4096 is 90 degre; motor is at 0 position when it looks like this:  O--
        radians_position  = revert_to_single_turn(Motor1.position - zero_pos - 4096)


        Grav_load = robot.rne([radians_position],[0],[0])  
 

        # T = Kt * Iq
        # NOTE the commanded current needs to be reverese of calculated current!
        # Since we want our motor to "fight" the gravity
        Commanded_Current = -int((Grav_load[0] / Kt) * 1000) # Units of mA


    timer.checkpt()
```

## Replicate this demo

### Full code

Above example is just a pseudo code for a full python code check our github repo

## Learn more
