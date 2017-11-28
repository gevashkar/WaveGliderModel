% Generate the generalised added mass and coriolis matrices based on the
% diagonal hydrodynamic force co-efficients (Xu,Yv,Zw,Kp,Mq,Nr) which are
% assumed positive and the body-fixed velocity vector (vb) where vb =
% [u;v;w;p;q;r].
% [M,C] = addedMC(Xu,Yv,Zw,Kp,Mq,Nr,vb)
function [M,C] = addedMC(Xu,Yv,Zw,Kp,Mq,Nr,vb)

u = vb(1);
v = vb(2);
w = vb(3);
p = vb(4);
q = vb(5);
r = vb(6);

M = -[Xu,0,0,0,0,0;
     0,Yv,0,0,0,0;
     0,0,Zw,0,0,0;
     0,0,0,Kp,0,0;
     0,0,0,0,Mq,0;
     0,0,0,0,0,Nr];
 
a1 = Xu*u;
a2 = Yv*v;
a3 = Zw*w;
b1 = Kp*p;
b2 = Mq*q;
b3 = Nr*r;
 
C = [0,0,0,0,-a3,a2;
     0,0,0,a3,0,-a1;
     0,0,0,-a2,a1,0;
     0,-a3,a2,0,-b3,b2;
     a3,0,-a1,b3,0,-b1;
     -a2,a1,0,-b2,b1,0];

C = C*[u;v;w;p;q;r];

end