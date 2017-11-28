% Function to determine the linear and angular drag(Dl,Da) for the float
% based on the system velocities(dq) and the linear body-fixed float
% velocities(vfb).
% [Dl,Da] = DragFloat(dq,vfb)
function [Dl,Da] = DragFloat(dq,vfb)

% float body-fixed velocities 
vfbx = vfb(1);          
vfbys = sign(vfb(2));
vfby = abs(vfb(2));
vfbzs = sign(vfb(3));
vfbz = abs(vfb(3));
dphifs = -sign(dq(4));
dphif = -abs(dq(4));
dthetaf = dq(5);
dpsifs = sign(dq(6));
dpsif = abs(dq(6));

% Drag values for x motion
Xx = (-8.1624*vfbx^3 - 3.9216*vfbx^2 - 8.228*vfbx);               
Yx = 0;
Zx = 8.0959*vfbx^3 + 42.109*vfbx^2 + 8.5064*vfbx;
Phifx = 0;
Thetafx = 14.448*vfbx^3 + 27.359*vfbx^2 + 15.064*vfbx;
Psifx = 0;


% Drag values for y motion
Xy = 0;
Yy = vfbys*(0.917*vfby^3 - 381.15*vfby^2 - 0.1929*vfby);
Zy = (2.4808*vfby^3 - 117.37*vfby^2 + 0.0897*vfby);
Phify = vfbys*(0.2801*vfby^3 - 109.34*vfby^2 - 0.0355*vfby);
Thetafy = 0;
Psify = vfbys*(-0.5206*vfby^3 + 107.66*vfby^2 - 0.5488*vfby);


% Drag values for z motion
Xz = 0;
Yz = 0;
Phifz = 0;
if(vfbz>0)
    Zz = (-0.6377*vfbz^3 - 1348.6*vfbz^2 + 0.1858*vfbz);
    Thetafz = (0.7833*vfbz^3 - 136.77*vfbz^2 + 0.715*vfbz);
else
    Zz = 0;
    Thetafz = 0;
end
Psifz = 0;


% Drag values for x rotation
Xphif = 0;
Yphif = 0;
Zphif = (-132.37*dphif^3 - 80.306*dphif^2 - 72.191*dphif);
Phifphif = dphifs*(-14.65*dphif^3 - 7.9888*dphif^2 - 8.4392*dphif);
Thetafphif = (-5.5813*dphif^3 + 6.6891*dphif^2 - 5.8296*dphif);
Psifphif = dphifs*(7.5145*dphif^3 + 7.4319*dphif^2 + 3.0866*dphif);


% Drag values for y rotation
Xthetaf = 123.89*dthetaf^3 + 2.4015*dthetaf^2 + 47.15*dthetaf;     
Ythetaf = 0;
Zthetaf = -205.65*dthetaf^3 + 118.99*dthetaf^2 - 97.566*dthetaf;
Phifthetaf = 0;
Thetafthetaf = (-1111.2*dthetaf^3 - 63.908*dthetaf^2 - 420.92*dthetaf);
Psifthetaf = 0;


% Drag values for z rotation
Xpsif = 0;
Ypsif = dpsifs*(23.589*dpsif^3 + 212.59*dpsif^2 + 6.6241*dpsif);
Zpsif = (-98.579*dpsif^3 + 354.74*dpsif^2 - 31.61*dpsif);
Phifpsif = 0;
Thetafpsif = 0;
Psifpsif = dpsifs*(-194.17*dpsif^3 - 414.24*dpsif^2 - 59.167*dpsif);


% Drag matrices for body-fixed linear and angular drag components
Dl = [Xx,Xy,Xz,Xphif,Xthetaf,Xpsif;
      Yx,Yy,Yz,Yphif,Ythetaf,Ypsif;
      Zx,Zy,Zz,Zphif,Zthetaf,Zpsif];
Da = [Phifx,Phify,Phifz,Phifphif,Phifthetaf,Phifpsif;
      Thetafx,Thetafy,Thetafz,Thetafphif,Thetafthetaf,Thetafpsif;
      Psifx,Psify,Psifz,Psifphif,Psifthetaf,Psifpsif];


end


