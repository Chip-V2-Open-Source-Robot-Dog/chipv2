%chip_leg_model_3D.m
%
%This is a Matlab Script that takes the parameters of the CHIP
%LEG SYSTEM (CLS) and models the DH-derived-origins and the
%inverse kinematics of the CLS.

%system representation/geometric variables
syms L1 L2 L3 L4 L5 L6 X12 X23 X3E Xe R1 R2
%system controllable variables
syms theta_1 theta_2 theta_3
%system output variables
syms xe ye ze

%SOLVING FOR FORWARD KINEMATICS
%vectors
X12 = [0;0;L1]
X23 = [0;-L3;L2]
X3E = [L5;-L6;L4]
Xe = [xe;ye;ze]

%rotation matricies
R1 = roty(-90)*[cos(theta_1),-sin(theta_1),0;sin(theta_1),cos(theta_1),0;0,0,1]*[1,0,0;0,cos(-theta_2),-sin(-theta_2);0,sin(-theta_2),cos(-theta_2)]
R2 = roty(90)*rotz(-90)*[1,0,0;0,cos(theta_3),-sin(theta_3);0,sin(theta_3),cos(theta_3)]

%kinematic equations calculations 
eqs_FK=Xe==X12+R1*X23+R1*R2*X3E

%%%%% TEST CODE %%%%
%sanity check
CHECK=subs(eqs_FK, [L1, L2, L3, L4, L5, L6, theta_1, theta_2, theta_3], [1.5, 1.5, 9, 0.5, 0.5, 12, pi/4, 0, -pi/2])
%%%%% END TEST CODE %%%%%

%SOLVING FOR INVERSE KINEMATICS




