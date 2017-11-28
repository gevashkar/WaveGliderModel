% Generate the specific added mass and coriolis for the float and glider
% based on positions(p), velocities(v),linear body-fixed float velocity(vfb),
% linear body-fixed glider velocity(vgb), and boolean stating whether float is in
% the water(toggleFloatAddedMass).
% [Ma,Ca] = AddedMass(p,v,vfb,vgb,toggleFloatAddedMass)
function [Ma,Ca] = AddedMass(p,v,vfb,vgb,toggleFloatAddedMass)

phif = p(4);
thetaf = p(5);
psif = p(6);

phig = p(9);
thetag = p(10);
psig = p(11);

Rfx = [1,0,0;
       0,cos(phif),-sin(phif);
       0,sin(phif),cos(phif)];
Rfy = [cos(thetaf),0,sin(thetaf);
       0,1,0;
       -sin(thetaf),0,cos(thetaf)];
Rfz = [cos(psif),-sin(psif),0;
       sin(psif),cos(psif),0;
       0,0,1];
Rf = Rfz*Rfy*Rfx;
Rgx = [1,0,0;
       0,cos(phig),-sin(phig);
       0,sin(phig),cos(phig)];
Rgy = [cos(thetag),0,sin(thetag);
    0,1,0;
     -sin(thetag),0,cos(thetag)];
Rgz = [cos(psig),-sin(psig),0;
       sin(psig),cos(psig),0;
       0,0,1];
Rg = Rgx*Rgy*Rgz;

Xxf = 60;%15.8776;
Yyf = 150;%85.6257;
% if(vfb(3)>0)
    Zzf = 300;%256.8770;
% else
%     Zzf = 0;
% end
Phifphif = 2;%61.9528;
Thetafthetaf = 98;%266.0294;
Psifpsif = 46;%3.0067;
% Xxf = 0;
% Yyf = 0;
% Zzf = 0;
% Phifphif = 0;
% Thetafthetaf = 0;
% Psifpsif = 0;

Xxg = 126.8700;%-126.8700;
Yyg = 70.0687;%-70.0687;
Zzg = 208.0086;%-208.0086;
Phigphig = 105.38;%-78.4139;
Thetagthetag = 65.309;%-58.5737;
Psigpsig = 106.091;%-54.9386;
% Xxg = 0;
% Yyg = 0;
% Zzg = 0;
% Phigphig = 0;
% Thetagthetag = 0;
% Psigpsig = 0;

%Rotate components to correct directions
Mag11 = zeros(3);
Mag11 = Rg*[Xxg,0,0;
            0,Yyg,0;
            0,0,Zzg];
Xxg = sum(abs(Mag11(:,1)));
Yyg = sum(abs(Mag11(:,2)));
Zzg = sum(abs(Mag11(:,3)));

Mag = set_AddedMassGlider2017_09_21(p,v,[Xxg;Yyg;Zzg;Phigphig;Thetagthetag;Psigpsig]);
Cag11 = get_AddedCg112017_09_21(p,v,[Xxg;Yyg;Zzg;Phigphig;Thetagthetag;Psigpsig]);
Cag12 = get_AddedCg122017_09_21(p,v,[Xxg;Yyg;Zzg;Phigphig;Thetagthetag;Psigpsig]);
Cag21 = get_AddedCg212017_09_21(p,v,[Xxg;Yyg;Zzg;Phigphig;Thetagthetag;Psigpsig]);
Cag22 = get_AddedCg222017_09_21(p,v,[Xxg;Yyg;Zzg;Phigphig;Thetagthetag;Psigpsig]);
Cag = [Cag11,Cag12;
       Cag21,Cag22];

if(toggleFloatAddedMass)
    Maf11 = zeros(3);
    Maf11 = Rf*[Xxf,0,0;
                0,Yyf,0;
                0,0,Zzf];
    Xxf = sum(abs(Maf11(:,1)));
    Yyf = sum(abs(Maf11(:,2)));
    Zzf = sum(abs(Maf11(:,3)));
    Maf = set_AddedMassFloat2017_09_21(p,v,[Xxf;Yyf;Zzf;Phifphif;Thetafthetaf;Psifpsif]);
    Caf = get_AddedCf2017_09_21(p,v,[Xxf;Yyf;Zzf;Phifphif;Thetafthetaf;Psifpsif]);
else
    Maf = zeros(12,12);
    Caf = zeros(12,12);
end

Ma = Maf+Mag;
Ca = Caf+Cag;

% phif = p(4);
% thetaf = p(5);
% psif = p(6);
% 
% phig = p(9);
% thetag = p(10);
% psig = p(11);
% 
% uf = vfb(1);
% vf = vfb(2);
% if(vfb(3)>0)
%     wf = vfb(3);
% else
%     wf = 0;
% end
% pf = v(4);
% qf = v(5);
% rf = v(6);
% 
% ug = vgb(1);
% vg = vgb(2);
% wg = vgb(3);
% pg = v(9);
% qg = v(10);
% rg = v(11);
% 
% Xxf = -60;%15.8776;
% Yyf = -150;%85.6257;
% if(vfb(3)>0)
%     Zzf = -300;%256.8770;
% else
%     Zzf = 0;
% end
% Phifphif = -2;%61.9528;
% Thetafthetaf = -98;%266.0294;
% Psifpsif = -46;%3.0067;
% % Xxf = 0;
% % Yyf = 0;
% % Zzf = 0;
% % Phifphif = 0;
% % Thetafthetaf = 0;
% % Psifpsif = 0;
% 
% Xxg = -126.8700;%-126.8700;
% Yyg = -70.0687;%-70.0687;
% Zzg = -208.0086;%-208.0086;
% Phigphig = -78.4139;%-78.4139;
% Thetagthetag = -58.5737;%-58.5737;
% Psigpsig = -54.9386;%-54.9386;
% % Xxg = 0;
% % Yyg = 0;
% % Zzg = 0;
% % Phigphig = 0;
% % Thetagthetag = 0;
% % Psigpsig = 0;
% 
% Rfx = [1,0,0;
%        0,cos(phif),-sin(phif);
%        0,sin(phif),cos(phif)];
% Rfy = [cos(thetaf),0,sin(thetaf);
%        0,1,0;
%        -sin(thetaf),0,cos(thetaf)];
% Rfz = [cos(psif),-sin(psif),0;
%        sin(psif),cos(psif),0;
%        0,0,1];
% Rf = Rfz*Rfy*Rfx;
% Rgx = [1,0,0;
%        0,cos(phig),-sin(phig);
%        0,sin(phig),cos(phig)];
% Rgy = [cos(thetag),0,sin(thetag);
%     0,1,0;
%      -sin(thetag),0,cos(thetag)];
% Rgz = [cos(psig),-sin(psig),0;
%        sin(psig),cos(psig),0;
%        0,0,1];
% Rg = Rgx*Rgy*Rgz;
% 
% [Mg, Cg] = addedMC(Xxg,Yyg,Zzg,Phigphig,Thetagthetag,Psigpsig,[ug;vg;wg;pg;qg;rg]);
% Mg(1:3,1:3) = abs(Rg*Mg(1:3,1:3));          
% Mg(1:3,1:3) = diag(Mg(1:3,1))+diag(Mg(1:3,2))+diag(Mg(1:3,3)); % Mass (and similarly added mass) added along diag
% AddedMassGlider = set_AddedMassGlider2017_06_09(p,v,[Mg(1,1);Mg(2,2);Mg(3,3);Mg(4,4);Mg(5,5);Mg(6,6)]);
% 
% Cg(1:3) = Rg*Cg(1:3);
% Fcgl = getGenForce(p,v,Cg(1:3),zeros(9,1),3);
% 
% if(toggleFloatAddedMass)
%     [Mf, Cf] = addedMC(Xxf,Yyf,Zzf,Phifphif,Thetafthetaf,Psifpsif,[uf;vf;wf;pf;qf;rf]);
%     
%     Mf(1:3,1:3) = abs(Rf*Mf(1:3,1:3));
%     Mf = abs(Mf);
%     Mf(1:3,1:3) = diag(Mf(1:3,1))+diag(Mf(1:3,2))+diag(Mf(1:3,3)); % Mass (and similarly added mass) added along diag
%     
%     Ma = [Mf(1:3,1:3),zeros(3,9);
%          zeros(9,3),[Mf(4:6,4:6),zeros(3,6);
%                      zeros(6,3),zeros(6,6)]];
%                      
%     Ma = Ma+AddedMassGlider;
%                              
%     Cf(1:3) = Rf*Cf(1:3);
%     Fcfl = getGenForce(p,v,Cf(1:3),zeros(9,1),1);
%     
%     Ca = Fcfl+Fcgl+[zeros(3,1);Cf(4:6);zeros(6,1)]+[zeros(8,1);Cg(4:6);zeros(1,1)];%
% else
%     Ma = AddedMassGlider;
%                              
%     Ca = Fcgl+[zeros(8,1);Cg(4:6);zeros(1,1)];
% end

end

