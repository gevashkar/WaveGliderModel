% Generalised force on tether

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
syms bx by bz

q = [xf;yf;zf;phif;thetaf;psif;phit;thetat;phig;thetag;psig;thetaa];
dq = [dxf;dyf;dzf;dphif;dthetaf;dpsif;dphit;dthetat;dphig;dthetag;dpsig;dthetaa];

p = get_model_param;
    A = p.A3D;
    ftx = p.ftx_tether; fty = p.fty_tether; ftz = p.ftz_tether;
    l1 = p.l_tether/2;

F = [Fx;Fy;Fz];
pos = DH(A,0,6)*([ftz;-fty;ftx;0]+DH(A,6,8)*[l1-bz;bx;by;1]);
r = pos(1:3);

Q = sym(zeros([length(q),1]));

for j = 1:1:length(q)
    Q(j) = Q(j) + sum(F.'*diff(r,q(j)));
end
clear i j;
% Q = simplify(Q,'IgnoreAnalyticConstraints',true);

inR = [bx;by;bz];

matlabFunction(Q,'File',strcat(filename,'\model_variables\get_QGenTether'),'Vars', {q,dq,F,inR});