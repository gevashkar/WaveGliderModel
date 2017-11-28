% Generates the rigid body equations of motion for the 3D Wave Glider
% system. The system is defined by three bodies: the float(f), 
% the tether(t) and the glider(g).
% The script generates a: Mass matrix(M), Coriolis matrix(C),
% Restorative matrix(G), simple damping matrix(D), Added Mass matrices 
%for the float and glider, and Coriolis and centripetal force 
%matrices for the float and glider. The system has 12
% generalised coordinates, q =
% [xf;yf;zf;phif;thetaf;psif;phit;thetat;phig;thetag;psig;thetaa]. The restorative
% matrix deals only with gravitational restorative forces for the system
% and the simple damping matrix is proportional to mass and for each
% coordinate.
clear;

tic;
display(datestr(now));
syms xf yf zf dxf dyf dzf ddxf ddyf ddzf
syms phif dphif ddphif
syms thetaf dthetaf ddthetaf
syms psif dpsif ddpsif
syms phit dphit ddphit
syms thetat dthetat ddthetat
syms phig dphig ddphig
syms thetag dthetag ddthetag
syms psig dpsig ddpsig
syms thetaa dthetaa ddthetaa
syms g
syms mf Jfy Jfx Jfz 
syms lt1 mt Jty Jtx ftx fty ftz gtx gty gtz
syms mg Jgy Jgx Jgz
syms AddedMmfx AddedMmfy AddedMmfz AddedMJfx AddedMJfy AddedMJfz
syms AddedMmgx AddedMmgy AddedMmgz AddedMJgx AddedMJgy AddedMJgz
syms ma Jay
syms ks sl0 la1 la2 dAG

[filename,~,~] = fileparts(mfilename('fullpath'));
filedate = datestr(date,'yyyy_mm_dd');

rho = 1000;
p = get_model_param;
    A = p.A3D;
    CG_float = [p.CG_float;1];
    CG_tether = [p.CG_tether;1];
    CG_glider = [p.CG_glider;1];
    CG_aero = [p.CG_aero;1];
                               
% Generalised coordinates
q = [xf;yf;zf;phif;thetaf;psif;phit;thetat;phig;thetag;psig;thetaa];
dq = [dxf;dyf;dzf;dphif;dthetaf;dpsif;dphit;dthetat;dphig;dthetag;dpsig;dthetaa];
ddq = [ddxf;ddyf;ddzf;ddphif;ddthetaf;ddpsif;ddphit;ddthetat;ddphig;ddthetag;ddpsig;ddthetaa];

% Positions
pos = (DH(A,0,6)*CG_float);
pf = pos(1:3);
pos = DH(A,0,6)*([ftz;-fty;ftx;0]+DH(A,6,8)*([lt1;0;0;0]+CG_tether));
pt = pos(1:3);
pos = DH(A,0,6)*([ftz;-fty;ftx;0]+DH(A,6,11)*([-gtx;-gty;-gtz;0]+CG_glider));
pg = pos(1:3);
pos = DH(A,0,6)*([ftz;-fty;ftx;0]+DH(A,6,11)*([-gtx;-gty;-gtz;0]+DH(A,11,12)*([-la1;0;0;0]+CG_aero)));
pa = pos(1:3);
pos = DH(A,0,6)*([ftz;-fty;ftx;0]+DH(A,6,11)*([-gtx;-gty;-gtz;0]+DH(A,11,12)*[-la2;0;0;1]));
patip = pos(1:3);
pos = DH(A,0,6)*([ftz;-fty;ftx;0]+DH(A,6,11)*([-gtx;-gty;-gtz;0]+[-dAG;0;0;1]));
pspringbase = pos(1:3);

sl = pspringbase-patip;
% Velocities
dpf = jacobian(pf,q)*dq;
dpt = jacobian(pt,q)*dq;
dpg = jacobian(pg,q)*dq;
dpa = jacobian(pa,q)*dq;
    display('Positions and velocities determined...');
    display(sprintf('Total time elapsed: %f s\n',toc));
    tStart = tic;

% % Accelerations
% ddpf = jacobian(dpf,q)*dq + jacobian(dpf,dq)*ddq;
% ddpt = jacobian(dpt,q)*dq + jacobian(dpt,dq)*ddq;   
% ddpg = jacobian(dpg,q)*dq + jacobian(dpg,dq)*ddq;  
        
% Calculate energy
T0 = sum(1/2*(mf*transpose(dpf)*dpf)) + (1/2*(Jfy*dthetaf^2)) + (1/2*(Jfx*dphif^2))+ (1/2*(Jfz*dpsif^2));
T1 = sum(1/2*(mt*transpose(dpt)*dpt)) + (1/2*(Jty*(dthetat)^2)) + (1/2*(Jtx*(dphit)^2));
T2 = sum(1/2*(mg*transpose(dpg)*dpg)) + (1/2*(Jgy*(dthetag)^2)) + (1/2*(Jgx*(dphig)^2)) + (1/2*(Jgz*(dpsig)^2));
T3 = sum(1/2*(ma*transpose(dpa)*dpa)) + (1/2*(Jay*(dthetaa)^2));
    display('Kinetic energy determined...');
    display(sprintf('\tTime taken: %f s',toc(tStart)));
    tStart = tic;
    display(sprintf('Total time elapsed: %f s\n',toc));
U =  mf*g*pf(3) + mt*g*pt(3) + mg*g*pg(3) + ma*g*pa(3) - (1/2)*ks*(sqrt(sl.'*sl)-sl0)^2;
    display('Potential energy determined...');
    display(sprintf('\tTime taken: %f s',toc(tStart)));
    tStart = tic;
    display(sprintf('Total time elapsed: %f s\n',toc));

% Energy for Added Mass
T0AddedM = 1/2*(AddedMmfx*dpf(1)*dpf(1)) + 1/2*(AddedMmfy*dpf(2)*dpf(2)) + 1/2*(AddedMmfz*dpf(3)*dpf(3)) + (1/2*(AddedMJfy*(dthetaf)^2)) + (1/2*(AddedMJfx*(dphif)^2)) + (1/2*(AddedMJfz*(dpsif)^2));
T2AddedM = 1/2*(AddedMmgx*dpg(1)*dpg(1)) + 1/2*(AddedMmgy*dpg(2)*dpg(2)) + 1/2*(AddedMmgz*dpg(3)*dpg(3)) + (1/2*(AddedMJgy*(dthetag)^2)) + (1/2*(AddedMJgx*(dphig)^2)) + (1/2*(AddedMJgz*(dpsig)^2));

% Simplify energies
Ttot = T0+T1+T2+T3;
clear T0 T1 T2 T3
Vtot = -U;
clear U


% Callen's Method
% Mass matrix
M = jacobian(jacobian(Ttot,dq).',dq); %same as differentiating twice and using loops to find your mass matrix, faster than looping for bigger systems
clear Ttot
    display('Mass matrix determined...');
    display(sprintf('\tTime taken: %f s',toc(tStart)));
    tStart = tic;
    display(sprintf('Total time elapsed: %f s\n',toc));
% Added Mass matrix
AddedMFloat = jacobian(jacobian(T0AddedM,dq).',dq); %same as differentiating twice and using loops to find your mass matrix, faster than looping for bigger systems
    display('Float Added Mass matrix determined...');
    display(sprintf('\tTime taken: %f s',toc(tStart)));
    tStart = tic;
    display(sprintf('Total time elapsed: %f s\n',toc));
AddedMGlider = jacobian(jacobian(T2AddedM,dq).',dq); %same as differentiating twice and using loops to find your mass matrix, faster than looping for bigger systems
    display('Glider Added Mass matrix determined...');
    display(sprintf('\tTime taken: %f s',toc(tStart)));
    tStart = tic;
    display(sprintf('Total time elapsed: %f s\n',toc));


% Centrifugal and coriolis accelerations matrix
C = symMat([length(q) length(q)],'C','real'); %Must use symMat so that its an actual matrix that can have variables passed to it!!
for i = 1 : length(q)
    for j = 1 : length(q)
        for k = 1: length(q)
            if k == 1
                C(i,j) = 0.5*(diff(M(i,j),q(k)) + diff(M(i,k),q(j)) - diff(M(j,k),q(i)))*dq(k);
            else
                C(i,j) = C(i,j)+ 0.5*(diff(M(i,j),q(k)) + diff(M(i,k),q(j)) - diff(M(j,k),q(i)))*dq(k);
            end
        end        
    end
end
clear i j k;
    display('Coriolis matrix determined...');
    display(sprintf('\tTime taken: %f s',toc(tStart)));
    tStart = tic;
    display(sprintf('Total time elapsed: %f s\n',toc));
AddedCf = symMat([length(q) length(q)],'AddedCf','real'); %Must use symMat so that its an actual matrix that can have variables passed to it!!
for i = 1 : length(q)
    for j = 1 : length(q)
        for k = 1: length(q)
            if k == 1
                AddedCf(i,j) = 0.5*(diff(AddedMFloat(i,j),q(k)) + diff(AddedMFloat(i,k),q(j)) - diff(AddedMFloat(j,k),q(i)))*dq(k);
            else
                AddedCf(i,j) = AddedCf(i,j)+ 0.5*(diff(AddedMFloat(i,j),q(k)) + diff(AddedMFloat(i,k),q(j)) - diff(AddedMFloat(j,k),q(i)))*dq(k);
            end
        end        
    end
end
clear i j k;
    display('Float Added Mass Coriolis matrix determined...');
    display(sprintf('\tTime taken: %f s',toc(tStart)));
    tStart = tic;
    display(sprintf('Total time elapsed: %f s\n',toc));
AddedCg = symMat([length(q) length(q)],'AddedCg','real'); %Must use symMat so that its an actual matrix that can have variables passed to it!!
for i = 1 : length(q)
    for j = 1 : length(q)
        for k = 1: length(q)
            if k == 1
                AddedCg(i,j) = 0.5*(diff(AddedMGlider(i,j),q(k)) + diff(AddedMGlider(i,k),q(j)) - diff(AddedMGlider(j,k),q(i)))*dq(k);
            else
                AddedCg(i,j) = AddedCg(i,j)+ 0.5*(diff(AddedMGlider(i,j),q(k)) + diff(AddedMGlider(i,k),q(j)) - diff(AddedMGlider(j,k),q(i)))*dq(k);
            end
        end        
    end
end
clear i j k;
    display('Glider Added Mass Coriolis matrix determined...');
    display(sprintf('\tTime taken: %f s',toc(tStart)));
    tStart = tic;
    display(sprintf('Total time elapsed: %f s\n',toc));


% Potential energy matrix
G = sym(zeros([length(q),1]));
for i = 1: length(q)
    G(i) = (diff(Vtot,q(i)));
end
clear i Vtot;
    display('Restoring matrix determined...');
    display(sprintf('\tTime taken: %f s',toc(tStart)));
    tStart = tic;
    display(sprintf('Total time elapsed: %f s\n',toc));

% Simple damping matrix
D = (mf+mt+mg+ma)*eye(12);
D = D*0.01;

% Assigning parameters
display('Substituting known values...');
    mf = p.M_float;     mt = p.M_tether;    mg = p.M_glider;    ma = p.M_aero;
    Jfy = p.Jy_float;   Jfx = p.Jx_float;   Jfz = p.Jz_float;
    Jty = p.Jy_tether;  Jtx = p.Jx_tether;
    Jgy = p.Jy_glider;  Jgx = p.Jx_glider;  Jgz = p.Jz_glider;
    Jay = p.Jy_aero;
    lt1 = p.l_tether/2;
    ftx = p.ftx_tether; fty = p.fty_tether; ftz = p.ftz_tether; 
    gtx = p.gtx_tether; gty = p.fty_tether; gtz = p.gtz_tether;
    dAG = p.dAG_aero;   la2 = p.l_aero;     la1 = la2/2;        sl0 = p.l0_aero;
    g  = p.g;

% Substituting in variable for system equations
M = subs(M);
AddedMassFloat = subs(AddedMFloat);
AddedMassGlider = subs(AddedMGlider);
C = subs(C);
AddedCf = subs(AddedCf);
AddedCg = subs(AddedCg);
G = subs(G);
D = subs(D);
dpf = subs(dpf);
dpg = subs(dpg);
pg = subs(pg);
sl = subs(sl);
    display(sprintf('\tTime taken: %f s',toc(tStart)));
    tStart = tic;
    display(sprintf('Total time elapsed: %f s\n',toc));

% Generating system equations
display('Generating system equations...');
matlabFunction(M,'File',strcat(filename,'/model_variables/get_M',filedate),'Vars', {q,dq});
    display(sprintf('\t Mass'));
    display(sprintf('\tTime passed: %f s',toc(tStart)));
matlabFunction(AddedMassFloat,'File',strcat(filename,'/model_variables/set_AddedMassFloat',filedate),'Vars', {q,dq,[AddedMmfx;AddedMmfy;AddedMmfz;AddedMJfx;AddedMJfy;AddedMJfz]});
    display(sprintf('\t FloatAddedMass'));
    display(sprintf('\tTime passed: %f s',toc(tStart)));
matlabFunction(AddedMassGlider,'File',strcat(filename,'/model_variables/set_AddedMassGlider',filedate),'Vars', {q,dq,[AddedMmgx;AddedMmgy;AddedMmgz;AddedMJgx;AddedMJgy;AddedMJgz]});
    display(sprintf('\t GliderAddedMass'));
    display(sprintf('\tTime passed: %f s',toc(tStart)));
matlabFunction(C(1:4,1:4),'File',strcat(filename,'/model_variables/get_C11',filedate),'Vars', {q,dq});
    display(sprintf('\t Coriolis 1'));
    display(sprintf('\tTime passed: %f s',toc(tStart)));
matlabFunction(C(1:4,5:8),'File',strcat(filename,'/model_variables/get_C12',filedate),'Vars', {q,dq});
    display(sprintf('\t Coriolis 2'));
    display(sprintf('\tTime passed: %f s',toc(tStart)));
matlabFunction(C(1:4,9:12),'File',strcat(filename,'/model_variables/get_C13',filedate),'Vars', {q,dq});
    display(sprintf('\t Coriolis 3'));
    display(sprintf('\tTime passed: %f s',toc(tStart)));
matlabFunction(C(5:8,1:4),'File',strcat(filename,'/model_variables/get_C21',filedate),'Vars', {q,dq});
    display(sprintf('\t Coriolis 4'));
    display(sprintf('\tTime passed: %f s',toc(tStart)));
matlabFunction(C(5:8,5:8),'File',strcat(filename,'/model_variables/get_C22',filedate),'Vars', {q,dq});
    display(sprintf('\t Coriolis 5'));
    display(sprintf('\tTime passed: %f s',toc(tStart)));
matlabFunction(C(5:8,9:12),'File',strcat(filename,'/model_variables/get_C23',filedate),'Vars', {q,dq});
    display(sprintf('\t Coriolis 6'));
    display(sprintf('\tTime passed: %f s',toc(tStart)));
matlabFunction(C(9:12,1:4),'File',strcat(filename,'/model_variables/get_C31',filedate),'Vars', {q,dq});
    display(sprintf('\t Coriolis 7'));
    display(sprintf('\tTime passed: %f s',toc(tStart)));
matlabFunction(C(9:12,5:8),'File',strcat(filename,'/model_variables/get_C32',filedate),'Vars', {q,dq});
    display(sprintf('\t Coriolis 8'));
    display(sprintf('\tTime passed: %f s',toc(tStart)));
matlabFunction(C(9:12,9:12),'File',strcat(filename,'/model_variables/get_C33',filedate),'Vars', {q,dq});
    display(sprintf('\t Coriolis 9'));
    display(sprintf('\tTime passed: %f s',toc(tStart)));
matlabFunction(AddedCf,'File',strcat(filename,'/model_variables/get_AddedCf',filedate),'Vars', {q,dq,[AddedMmfx;AddedMmfy;AddedMmfz;AddedMJfx;AddedMJfy;AddedMJfz]});
    display(sprintf('\t Float Added Mass Coriolis'));
    display(sprintf('\tTime passed: %f s',toc(tStart)));
matlabFunction(AddedCg(1:6,1:6),'File',strcat(filename,'/model_variables/get_AddedCg11',filedate),'Vars', {q,dq,[AddedMmgx;AddedMmgy;AddedMmgz;AddedMJgx;AddedMJgy;AddedMJgz]});
    display(sprintf('\t Glider Added Mass Coriolis 11'));
    display(sprintf('\tTime passed: %f s',toc(tStart)));
matlabFunction(AddedCg(1:6,7:12),'File',strcat(filename,'/model_variables/get_AddedCg12',filedate),'Vars', {q,dq,[AddedMmgx;AddedMmgy;AddedMmgz;AddedMJgx;AddedMJgy;AddedMJgz]});
    display(sprintf('\t Glider Added Mass Coriolis 12'));
    display(sprintf('\tTime passed: %f s',toc(tStart)));
matlabFunction(AddedCg(7:12,1:6),'File',strcat(filename,'/model_variables/get_AddedCg21',filedate),'Vars', {q,dq,[AddedMmgx;AddedMmgy;AddedMmgz;AddedMJgx;AddedMJgy;AddedMJgz]});
    display(sprintf('\t Glider Added Mass Coriolis 21'));
    display(sprintf('\tTime passed: %f s',toc(tStart)));
matlabFunction(AddedCg(7:12,7:12),'File',strcat(filename,'/model_variables/get_AddedCg22',filedate),'Vars', {q,dq,[AddedMmgx;AddedMmgy;AddedMmgz;AddedMJgx;AddedMJgy;AddedMJgz]});
    display(sprintf('\t Glider Added Mass Coriolis 22'));
    display(sprintf('\tTime passed: %f s',toc(tStart)));
matlabFunction(G,'File',strcat(filename,'/model_variables/get_G',filedate),'Vars', {q,dq,ks});
matlabFunction(D,'File',strcat(filename,'/model_variables/get_D',filedate),'Vars', {q,dq});
matlabFunction(dpf,'File',strcat(filename,'/model_variables/get_dpf',filedate),'Vars', {q,dq});
matlabFunction(dpg,'File',strcat(filename,'/model_variables/get_dpg',filedate),'Vars', {q,dq});
matlabFunction(pf,'File',strcat(filename,'/model_variables/get_pf',filedate),'Vars', {q,dq});
matlabFunction(pg,'File',strcat(filename,'/model_variables/get_pg',filedate),'Vars', {q,dq});
matlabFunction(sl,'File',strcat(filename,'/model_variables/get_sl',filedate),'Vars', {q,dq});
save(strcat(filename,'/model_variables/parameters',filedate,'.mat'),'p');
    display(sprintf('\tTime taken: %f s',toc(tStart)));
    tStart = tic;
    display(sprintf('Total time elapsed: %f s\n',toc));

display(datestr(now));
disp('done.')


