% Function to handle generating the volume(Vol) and centroid(Cu) of the
% non-convex float hull section below a surface based on convex volumes.
% The function takes in the generalised coordinates of the system(q) and
% the parameters for the surface(surface = [dZdx;dZdy;dz]).
% [Vol,Cu] = FloatBuoy(q,surface)
function [Vol,Cu] = FloatBuoy(q,surface)
% Convex float parts
P1 = [                                     % Float Top
    1.5250,         0,   -0.1150;
    1.4750,   -0.1694,   -0.1150;
    1.2000,   -0.3675,   -0.1150;
    0.9250,   -0.4050,   -0.1150;
   -1.5250,   -0.4050,   -0.1150;
   -1.5250,    0.4050,   -0.1150;
    0.9250,    0.4050,   -0.1150;
    1.2000,    0.3524,   -0.1150;
    1.4750,    0.1543,   -0.1150;
    
    1.5250,         0,   -0.0750;
    1.4750,   -0.1694,   -0.0750;
    1.2000,   -0.3675,   -0.0750;
    0.9250,   -0.4050,   -0.0750;
   -1.5250,   -0.4050,   -0.0750;
   -1.5250,    0.4050,   -0.0750;
    0.9250,    0.4050,   -0.0750;
    1.2000,    0.3524,   -0.0750;
    1.4750,    0.1543,   -0.0750;
    ];
P2 = [                                       % Float Bottom
    1.5150,         0,    0.1150;
    0.7700,    0.1359,    0.1150;
    0.3295,    0.1659,    0.1150;
   -0.8084,    0.1732,    0.1150;
   -1.1706,    0.1530,    0.0833;
   -1.5250,    0.0700,    0.0523;
   -1.5250,   -0.0700,    0.0523;
   -1.1709,   -0.1530,    0.0833;
   -0.8084,   -0.1733,    0.1150;
    0.3295,   -0.1660,    0.1150;
    0.7700,   -0.1359,    0.1150;
    
    1.5150,         0,   -0.0750;
    1.1519,    0.1857,   -0.0750;
    0.6769,    0.2983,   -0.0750;
    0.0006,    0.3435,   -0.0750;
   -0.9577,    0.2983,   -0.0750;
   -1.3229,    0.1857,   -0.0750;
   -1.5250,    0.0700,   -0.0750;
   -1.5250,   -0.0700,   -0.0750;
   -1.3229,   -0.1857,   -0.0750;
   -0.9577,   -0.2983,   -0.0750;
    0.0034,   -0.3435,   -0.0750;
    0.6769,   -0.2983,   -0.0750;
    1.1519,   -0.1857,   -0.0750;
    ];

% Assigning variables
zf = q(3);
phi = q(4);
theta = q(5);
psi = q(6);
pos = [zf,phi,theta,psi];
Rphi = [1,0,0;
        0,cos(phi),-sin(phi);
        0,sin(phi),cos(phi)];
Rtheta = [cos(theta),0,sin(theta);
          0,1,0;
          -sin(theta),0,cos(theta)];
Rpsi = [cos(psi),-sin(psi),0;
        sin(psi),cos(psi),0;
        0,0,1];
R = Rpsi*Rtheta*Rphi;
P1F = zeros(size(P1));
P2F = zeros(size(P2));
for i = 1:size(P1,1)
    P1F(i,:) = (R*P1(i,:).').'+zf;
end
for i = 1:size(P2,1)
    P2F(i,:) = (R*P2(i,:).').'+zf;
end

% Calculating submerged regions, volume and CB
[Ct,Vt,Ctu,Vtu,P1U] = CenVol(P1,pos,surface);
[Cb,Vb,Cbu,Vbu,P2U] = CenVol(P2,pos,surface);

C = (Ct.*Vt+Cb.*Vb)/(Vt+Vb);

% If not submerged setting CB at CG
if(Vtu==0&&Vbu==0)
    Cu = C;
else
    Cu = (Ctu.*Vtu+Cbu.*Vbu)/(Vtu+Vbu);
end

Vol = Vtu+Vbu;
% end