syms z
Ko=10;
Kp=10*Ko;
Kd=2*Ko;
dt=0.001;

A_1=dt^2*Kp+dt*Kd-2;
A_2=1-dt*Kd;
B_1=dt^2*Kp+dt*Kd;
B_2=-dt*Kd;
Bd_1=dt^2;

G(z)=(B_1*z^-1+B_2*z^-2)/(1+A_1*z^-1+A_2*z^-2);
Gd(z)=Bd_1*z^-1/(1+A_1*z^-1+A_2*z^-2);
G(z)=simplify(G(z));
Gd(z)=simplify(Gd(z));

G(1);
Gd(1);

p=z^2+A_1*z+A_2;
A=solve(p, z);
a=abs(double(A(1)));
b=abs(double(A(2)));

n=[B_1]
d=[1, A_1, A_2]
[d,e,f]=residue(n,d)