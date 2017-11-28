% Generalised force on float

[filename,~,~] = fileparts(mfilename('fullpath'));

syms Fx Fy Fz
syms xf yf zf dxf dyf dzf  
syms phif dphif 
syms thetaf dthetaf
syms psif dpsif 
syms phit dphit 
syms thetat dthetat 
syms phig dphig 
syms thetag dthetag
syms psig dpsig 
syms thetaa dthetaa
syms ax ay az

q = [xf;yf;zf;phif;thetaf;psif;phit;thetat;phig;thetag;psig;thetaa];
dq = [dxf;dyf;dzf;dphif;dthetaf;dpsif;dphit;dthetat;dphig;dthetag;dpsig;dthetaa];

p = get_model_param;
    A = p.A3D;  

F = [Fx;Fy;Fz];
pos = (DH(A,0,6)*[az;-ay;ax;1]);
r = pos(1:3);

Q = sym(zeros([length(q),1]));

for j = 1:1:length(q)
    Q(j) = Q(j) + sum(F.'*diff(r,q(j)));
end
clear i j;
% Q = simplify(Q,'IgnoreAnalyticConstraints',true);

inR = [ax;ay;az];

matlabFunction(Q,'File',strcat(filename,'\model_variables\get_QGenFloat'),'Vars', {q,dq,F,inR});