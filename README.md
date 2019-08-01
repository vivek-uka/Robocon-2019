# Robocon-2019

FOUR WHEEL OMNI DIRECTIONAL ROBOT

UpperControl- It consists of :
              Localization algorithm - is such that it computes the instantaneous pose of the robot independent of it's orientation.
              Trajectory Planning - reliable and robust methods involving movement with any orientation, involving control systems.
              Tracing of different curves - line, sine wave, circle, arcs
              Actuation of major mechanisms
              
LowerControl- It deals with the inverse kinematics of the robot and a PID controller is used to maintain the required RPM of each wheel.

Ps2_send- It receives the PS2 data and sends it to the UpperControl for desired manual operations.
              
