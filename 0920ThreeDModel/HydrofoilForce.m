% Function to determine the hydrofoil forces and moments in the glider body
% frame. The hydrofoil is limited to the mechanical limits of between 40
% degrees and -20 degrees. The equations for the forces are depend on
% multiple variables.
function [F] = HydrofoilForce(vx,vz,fa,v)

% Hydrofoil angle and limitation
x = fa;
if (x >= (40*pi/180))
    x = 40*pi/180;
elseif (x <= (-20*pi/180))
    x = -20*pi/180;
end

% Hydrofoil pitch rate
dthetah = abs(v(12));
dthetahs = sign(v(12));

y1 = vx;
y2 = vz;

ef32 = [1;x;y1;x^2;x*y1;y1^2;x^3;x^2*y1;x*y1^2];
ef33 = [1;x;y2;x^2;x*y2;y2^2;x^3;x^2*y2;x*y2^2;y2^3];
ef34 = [1;x;y2;x^2;x*y2;y2^2;x^3;x^2*y2;x*y2^2;y2^3;x^3*y2;x^2*y2^2;x*y2^3;y2^4];

% Drag scaling along direction of motion
ds = 1;

Xvxp00 =      -2.493;
Xvxp10 =      -28.71;
Xvxp01 =        21.4;
Xvxp20 =       13.97;
Xvxp11 =       83.69;
Xvxp02 =      -18.21;
Xvxp30 =       78.06;
Xvxp21 =      -238.7;
Xvxp12 =      -50.38;
Xvx = [Xvxp00,Xvxp10,Xvxp01,Xvxp20,Xvxp11,Xvxp02,Xvxp30,Xvxp21,Xvxp12];

Zvxp00 =       9.361;
Zvxp10 =      -9.052;
Zvxp01 =      -15.08;
Zvxp20 =      -134.7;
Zvxp11 =      -56.71;
Zvxp02 =       3.669;
Zvxp30 =         184;
Zvxp21 =       164.7;
Zvxp12 =      -152.7;
Zvx = [Zvxp00,Zvxp10,Zvxp01,Zvxp20,Zvxp11,Zvxp02,Zvxp30,Zvxp21,Zvxp12];

Mvxp00 =     0.08385;
Mvxp10 =    -0.06752;
Mvxp01 =     -0.1364;
Mvxp20 =      -1.224;
Mvxp11 =     -0.5969;
Mvxp02 =     0.03879;
Mvxp30 =       1.639;
Mvxp21 =       1.617;
Mvxp12 =      -9.881;
Mvx = [Mvxp00,Mvxp10,Mvxp01,Mvxp20,Mvxp11,Mvxp02,Mvxp30,Mvxp21,Mvxp12];

Xvzp00 =      0.1504;
Xvzp10 =      -1.702;
Xvzp01 =      -2.825;
Xvzp20 =      -3.182;
Xvzp11 =      -104.4;
Xvzp02 =       4.934;
Xvzp30 =        8.73;
Xvzp21 =        17.1;
Xvzp12 =     -0.2249;
Xvzp03 =       1.195;
Xvzp31 =         113;
Xvzp22 =      -1.345;
Xvzp13 =         -56;
Xvzp04 =     -0.9051;
Xvz = [Xvzp00,Xvzp10,Xvzp01,Xvzp20,Xvzp11,Xvzp02,Xvzp30,Xvzp21,Xvzp12,Xvzp03,Xvzp31,Xvzp22,Xvzp13,Xvzp04];

Zvzp00 =      -3.384;
Zvzp10 =       5.974;
Zvzp01 =      -101.9;
Zvzp20 =       36.59;
Zvzp11 =        4.11;
Zvzp02 =      -1.541;
Zvzp30 =      -49.84;
Zvzp21 =       195.8;
Zvzp12 =      -1.312;
Zvzp03 =      -64.61;
Zvz = [Zvzp00,Zvzp10,Zvzp01,Zvzp20,Zvzp11,Zvzp02,Zvzp30,Zvzp21,Zvzp12,Zvzp03];

Mvzp00 =     -0.2982;
Mvzp10 =    -0.03749;
Mvzp01 =      -7.349;
Mvzp20 =       2.946;
Mvzp11 =      0.3783;
Mvzp02 =    -0.08356;
Mvzp30 =      -2.305;
Mvzp21 =       9.555;
Mvzp12 =      -2.616;
Mvzp03 =      -5.271;
Mvz = [Mvzp00,Mvzp10,Mvzp01,Mvzp20,Mvzp11,Mvzp02,Mvzp30,Mvzp21,Mvzp12,Mvzp03];

Xthetah = -0.015*dthetah^3 + 0.1762*dthetah^2 - 0.0041*dthetah;
Zthetah = dthetahs*(0.5177*dthetah^3 + 2.5707*dthetah^2 - 0.195*dthetah);
Thetahthetah = dthetahs*(0.0359*dthetah^3 - 0.23*dthetah^2 + 0.0138*dthetah);

F = [2.5*Xvx;2.5*Zvx;Mvx]*ef32 + [6*Xvz*ef34;6*Zvz*ef33;Mvz*ef33] ...
    +[6*Xthetah;6*Zthetah;Thetahthetah];

end