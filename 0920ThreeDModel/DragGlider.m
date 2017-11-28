% Function to determine the linear and angular drag(Dl,Da) for the glider
% based on the system velocities(dq) and the linear body-fixed glider
% velocities(vfb).
% [Dl,Da] = DragGlider(dq,vgb)
function [Dl,Da] = DragGlider(dq,vgb)

% glider body-fixed velocities 
vgbx = abs(vgb(1));              
vgbxs = sign(vgb(1)); 
vgby = abs(vgb(2));
vgbys = sign(vgb(2)); 
vgbz = vgb(3);

dphig = -abs(dq(9));
dphigs = -sign(dq(9));
dthetag = dq(10);
dpsig = abs(dq(11));
dpsigs = sign(dq(11));

if vgbz>=0                  % Drag due to x and y motion is dependent on 
    Thetagx = vgbxs*(0.0198*vgbx^3 - 9.024*vgbx^2 + 0.3392*vgbx);
else
    Thetagx = vgbxs*(0.0314*vgbx^3 + 25.064*vgbx^2 - 0.0025*vgbx);
end
[Xx,Yx,Zx,Phigx,Psigx] = deal(0); %Calculated in hydrofoil drag or zero

    
% Drag for y motion
Xy = 0;
Yy = vgbys*(-3.4999*vgby^3 - 267.35*vgby^2 + 1.7462*vgby);
Zy = 0;
Phigy = vgbys*(2.1702*vgby^3 - 26.918*vgby^2 + 1.7615*vgby);
Thetagy = (2.7615*vgby^3 + 0.3829*vgby^2 + 1.3178*vgby);
Psigy = vgbys*(-0.21*vgby^3 + 22.337*vgby^2 - 0.7102*vgby);


% Drag for z motion
Thetagz = -6.5392*vgbz^3 - 45.97*vgbz^2 - 6.9985*vgbz;
[Xz,Yz,Zz,Phigz,Psigz] = deal(0); %Calculated in hydrofoil drag or zero


% Drag for x rotation
Xphig = 0;
Yphig = 0;
Zphig = 0;
Phigphig = dphigs*(5.0191*dphig^3 + 97.147*dphig^2 + 2.9108*dphig);
Thetagphig = 0;
Psigphig = 0;


% Drag for y rotation
Xthetag = 0;
Ythetag = 0;
Zthetag = 48.66*dthetag^3 - 5.279*dthetag^2 + 19.055*dthetag;
Phigthetag = 0;
Thetagthetag = (-90.147*dthetag^3 + 2.7851*dthetag^2 - 37.551*dthetag);
Psigthetag = 0;


% Drag for z rotation
Xpsig = 0;
Ypsig = 0;
Zpsig = 0;
Phigpsig = 0;
Thetagpsig = 0;
Psigpsig = dpsigs*(10.915*dpsig^3 - 144.75*dpsig^2 + 1.5921*dpsig);

% Drag matrices for linear and angular body-fixed components
Dl = [Xx,Xy,Xz,Xphig,Xthetag,Xpsig;
      Yx,Yy,Yz,Yphig,Ythetag,Ypsig;
      Zx,Zy,Zz,Zphig,Zthetag,Zpsig]; 
Da = [Phigx,Phigy,Phigz,Phigphig,Phigthetag,Phigpsig;
      Thetagx,Thetagy,Thetagz,Thetagphig,Thetagthetag,Thetagpsig;
      Psigx,Psigy,Psigz,Psigphig,Psigthetag,Psigpsig];

end




