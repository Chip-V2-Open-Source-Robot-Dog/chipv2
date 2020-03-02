%chip_leg_model_FINAL_3D.m
%
%This is a Matlab Script that takes the parameters of the CHIP
%LEG SYSTEM (CLS) and models the inverse kinematics of the CLS.
%
%
%Note the zero position of the leg, theta_1 is a rotation around Z,
%theta_2 is a rotation around Y, and theta_3 is again about Z.

%system representation/geometric variables
syms L1 L2 L3 L4 L5 L6 X01 X12 X23 X3E Xe R1 R2 R3
%system controllable variables
syms theta_1 theta_2 theta_3
%system output variables
syms xe ye ze

%define the required vectors
X01=[0;0;0];
X12=[0;-L2;L1];
X23=[L3;0;0];
X3E=[-L6;-L5;L4];
Xe=[xe;ye;ze];

%define the required rotation matricies 
R1=[cos(theta_1),-sin(theta_1),0;sin(theta_1),cos(theta_1),0;0,0,1];
R2=[1,0,0;0,cos(theta_2),-sin(theta_2);0,sin(theta_2),cos(theta_2)];
R3=[cos(theta_3),-sin(theta_3),0;sin(theta_3),cos(theta_3),0;0,0,1];

%setup the forward kinematics equations
FK_EQs=Xe==X01+R1*X12+R1*R2*X23+R1*R2*R3*X3E;

%%%% THIS IS TEST CODE %%%%
%CHECK=subs(FK_EQs, [theta_1, theta_2, theta_3], [0, 0, 0])
%%%%%% END TEST CODE %%%%%%

%Now what we want to do is caluclate the JACBOIAN of the system!
Xe=X01+R1*X12+R1*R2*X23+R1*R2*R3*X3E;
J=jacobian([Xe(1),Xe(2),Xe(3)],[theta_1, theta_2, theta_3]);
JT(theta_1, theta_2, theta_3)=transpose(J)

%find the determinant of the Jacobian of the leg
detJ=det(J);

%now let's run some angles! 
syms W
T=JT(-pi/4, 0, pi/2)*[0;-W;0]
Ts=subs(T, [W, L1, L2, L3, L4, L5, L6], [500, 0.04, 0.04, 0.30, 0.04, 0.04, 0.38])
T_1=double(Ts(1))
T_2=double(Ts(2))
T_3=double(Ts(3))

