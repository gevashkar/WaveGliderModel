% Rudder forces on glider based on speed(body-fixed glider u in m/s) and 
% angle(rudder angle in rad).
% [F] = RudderForce(vx,ra).
function [F] = RudderForce(vx,ra)

x = abs(ra);
xs = -sign(ra);
y = vx;
ef = [1;x;y;x^2;x*y;y^2;x^3;x^2*y;x*y^2];

Xp00 =      0.4922;
Xp10 =      -2.025;
Xp01 =      -2.945;
Xp20 =      -1.971;
Xp11 =       19.28;
Xp02 =       2.248;
Xp30 =       4.804;
Xp21 =      -19.85;
Xp12 =      -16.85;
X = [Xp00,Xp10,Xp01,Xp20,Xp11,Xp02,Xp30,Xp21,Xp12];

Yp00 =    -0.07675;
Yp10 =      -1.615;
Yp01 =       2.021;
Yp20 =       7.774;
Yp11 =       -10.5;
Yp02 =      -2.254;
Yp30 =       -7.16;
Yp21 =       10.79;
Yp12 =      -16.01;
Y = [Yp00,Yp10,Yp01,Yp20,Yp11,Yp02,Yp30,Yp21,Yp12];

Zp00 =        1.24;
Zp10 =      -13.25;
Zp01 =    -0.09968;
Zp20 =       34.06;
Zp11 =      -2.215;
Zp02 =     -0.9949;
Zp30 =      -23.79;
Zp21 =       2.065;
Zp12 =       8.812;
Z = [Zp00,Zp10,Zp01,Zp20,Zp11,Zp02,Zp30,Zp21,Zp12];

Kp00 =     0.07327;
Kp10 =      -1.261;
Kp01 =      0.3866;
Kp20 =       3.987;
Kp11 =      -2.245;
Kp02 =      -0.538;
Kp30 =       -3.14;
Kp21 =       2.334;
Kp12 =      -1.214;
K = [Kp00,Kp10,Kp01,Kp20,Kp11,Kp02,Kp30,Kp21,Kp12];

Mp00 =       1.034;
Mp10 =      -11.21;
Mp01 =     0.07514;
Mp20 =       29.33;
Mp11 =      -3.285;
Mp02 =       -1.06;
Mp30 =      -20.72;
Mp21 =       3.178;
Mp12 =       9.805;
M = [Mp00,Mp10,Mp01,Mp20,Mp11,Mp02,Mp30,Mp21,Mp12];

Np00 =     -0.3303;
Np10 =        5.38;
Np01 =      -1.464;
Np20 =      -16.09;
Np11 =       7.519;
Np02 =       2.075;
Np30 =       12.32;
Np21 =      -7.711;
Np12 =       14.76;
N = [Np00,Np10,Np01,Np20,Np11,Np02,Np30,Np21,Np12];

F = [1;xs;1;xs;1;xs].*([X;Y;Z;K;M;N]*ef);

end