% Generalised force acting at arbitrary point in system

[filename,~,~] = fileparts(mfilename('fullpath'));

syms Fx Fy Fz
syms xf yf zf dxf dyf dzf  
syms phif dphif 
syms thetaf dthetaf
syms psif dpsif 
syms thetat dthetat 
syms phig dphig 
syms thetag dthetag
syms psig dpsig 
syms ax ay az bx by bz cx cy cz 

q = [xf;yf;zf;phif;thetaf;psif;thetat;phig;thetag;psig];
dq = [dxf;dyf;dzf;dphif;dthetaf;dpsif;dthetat;dphig;dthetag;dpsig];

Rphif  =   [1,0,0;
           0,cos(phif),-sin(phif);
           0,sin(phif),cos(phif)];

Rthetaf  = [cos(thetaf),0,sin(thetaf);
            0,1,0;
           -sin(thetaf),0,cos(thetaf)];
       
Rpsif =    [cos(psif),-sin(phif),0;
            sin(phif),cos(psif),0;
            0,0,1];       
        
Rthetat   = [cos(thetat),0,sin(thetat);
            0,1,0;
           -sin(thetat),0,cos(thetat)];

Rphig  =   [1,0,0;
           0,cos(phig),-sin(phig);
           0,sin(phig),cos(phig)];       
       
Rthetag  = [cos(thetag),0,sin(thetag);
            0,1,0;
           -sin(thetag),0,cos(thetag)];

Rpsig =    [cos(psig),-sin(phig),0;
            sin(phig),cos(psig),0;
            0,0,1];     

F = [Fx;Fy;Fz];
r = [xf;yf;zf] + Rpsif*Rthetaf*Rphif*[ax;ay;az] + Rthetat*[bx;by;bz] + Rpsig*Rthetag*Rphig*[cx;cy;cz];

Q = sym(zeros([length(q),1]));

for j = 1:1:length(q)
    Q(j) = Q(j) + sum(F.'*diff(r,q(j)));
end
clear i j;
Q = simplify(Q,'IgnoreAnalyticConstraints',true);

inR = [ax;ay;az;bx;by;bz;cx;cy;cz];

matlabFunction(Q,'File',strcat(filename,'\model_variables\get_QGen'),'Vars', {q,dq,F,inR});