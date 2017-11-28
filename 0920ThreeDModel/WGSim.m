%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Wave Glider simulation helper function.
% Author:    Gevashkar Rampersadh
% Created:   09/20/2017
% Modified:  09/20/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;

% % Update equations
% generalisedForceFloat;
% generalisedForceTether;
% generalisedForceGlider;
% clear;

% Simulation timing
T = 10;
dt = 1e-2;

% File parameters
testnumber = 3;
winddir = 0;    %// Azimuth

[foldername,~,~] = fileparts(mfilename('fullpath'));
filedate = datestr(date,'yyyy_mm_dd');
testversion = strcat(filedate,'_test',num2str(testnumber),'-',num2str(T),'s_1A0,1HzSeaState_Test-OlderEOM');
saveSea = strcat(filedate,'_test1');
readSea = strcat('2017_04_18','_test1');
Animate = 'y';

% Controlled Rudder
Rudder.signals.values = [repmat(0,1001,1)];
Rudder.signals.dimensions = length(Rudder.signals.values(1,:));
Rudder.time = (0:dt:T)';

% Desired Glider Yaw
GliderYaw.signals.values = [repmat(0,500,1);repmat(0*pi/180,501,1)];%;repmat(pi/4,2000,1);repmat(0,2001,1)];
GliderYaw.signals.dimensions = length(GliderYaw.signals.values(1,:));
GliderYaw.time = (0:dt:T)';

% Ocean parameters for Sinusoidal Sea State
param.time = 0;
meshsizeX  = 2^6;      %// main  grid size
meshsizeY  = 2^6;      %// main  grid size
patchsizeX = 2^8;     
patchsizeY = 2^8;     
param.signals.values = [meshsizeX,meshsizeY,patchsizeX,patchsizeY,winddir,dt,T];
param.signals.dimensions = length(param.signals.values(1,:));
% Sinusiodal sea parameters
SeaParam.time = 0;
numOfSignals = 1;
amplitude  = [1];      %// main  grid size
frequency  = [0.1*2*pi];      %// main  grid size
offset = 2^10;
SeaParam.signals.values = [numOfSignals,amplitude,frequency,offset,winddir];
SeaParam.signals.dimensions = size(SeaParam.signals.values);


% Initial conditions
xi = [0,0,0,0,0,0,0,0,0,0,0,0];
vi = [0,0,0,0,0,0,0,0,0,0,0,0];


% Set initial height to limit disturbance
ZData = 0;
t = 0;
g = 9.81;
px = xi(1);
py = xi(2);
ang = winddir*pi/180;
dir = [cos(ang) -sin(ang);sin(ang) cos(ang)]*[offset;0];
xoff = dir(1);
yoff = dir(2);
R = sqrt((px-xoff).^2 + (py-yoff).^2);
for i = 1:numOfSignals
    w = frequency(i);
    A = amplitude(i);
    Period = 1/w;
    k = w^2/g;
    phi = t/Period;
    ZData = ZData + A*sin(k*R-phi);
end
xi(3) = ZData;
clear ZData t g px py ang dir xoff yoff R w A Period k phi

% Simulating system
tic;
display('Starting sim...')
try
    sim('WG0920_3D');
catch ME
    display(ME.message);
end
toc;

% Saving outputs in folder
display('Saving results...')
mkdir(strcat(foldername,'\results\',testversion));
foldername = strcat(foldername,'\results\',testversion);
addpath(foldername);
save(strcat(foldername,'\WG','.mat'),'T','dt','WG','WGVel','WGAcc','FloatVel','GliderVel','GliderPos','FloatDrag','GliderDrag','Restorative','RudderForce','AddedMass','AddedCoriolis','Wave','RudderCommand','CB','param','SeaParam','HydroDrag','k','GliderYaw');%,'GliderYaw'

clear impA0 impD0 impW0 sA0 sD0 impA2 impD2 impW2 sA2 sD2 impAt impDt impWt sAt sDt tout;


%% Animating and displaying graphs

% Determining whether to generate animation
if strcmp(Animate,'y')
    
    % Generating local sea for animation
    [plane,X,Y] = GenSea(WG(:,1),WG(:,2),dt,SeaParam);
    
    % Determining animation duration
    T = min(T,size(WG,1)*dt);
    display('Starting animation...')
    
    % Video parameters
    rate = 30;
    filename = strcat(foldername,'\',testversion);   %filename and location
    filetype = 'MPEG-4';                  %type of file    

    % Loading model parameters
    p = get_model_param;
        lt  = p.l_tether;       lg = p.l_glider;    la = p.l_aero;
                                hg = p.h_glider;
        wf  = p.w_float;                            wg = p.w_glider;
        ft  = [p.ftx_tether;p.fty_tether;p.ftz_tether];
        gt  = [p.gtx_tether;p.fty_tether;p.gtz_tether];
        gax = p.gax_aero;
        gay = p.gay_aero;
        gaz = p.gaz_aero;
        FloatTop = p.FloatTop;
        FloatBottom = p.FloatBottom;
    
    way(1,1,:) = Wave(:,3);
    wax(1,1,:) = Wave(:,2);
    wh(1,1,:) = Wave(:,1);
    
    phif(1,1,:) = WG(:,4);
    thetaf(1,1,:) = WG(:,5);
    psif(1,1,:) = WG(:,6);
    phit(1,1,:) = WG(:,7);
    thetat(1,1,:) = WG(:,8);
    phig(1,1,:) = WG(:,9);
    thetag(1,1,:) = WG(:,10);
    psig(1,1,:) = WG(:,11);
    thetaa(1,1,:) = WG(:,12);
    psir(1,1,:) = RudderCommand(:,1);
    
    Rphif = [ones(1,1,size(phif,3)),zeros(1,1,size(phif,3)),zeros(1,1,size(phif,3));
            zeros(1,1,size(phif,3)),cos(phif),-sin(phif);
            zeros(1,1,size(phif,3)),sin(phif),cos(phif)];
    Rthetaf = [cos(thetaf),zeros(1,1,size(phif,3)),sin(thetaf);
              zeros(1,1,size(phif,3)),ones(1,1,size(phif,3)),zeros(1,1,size(phif,3));
              -sin(thetaf),zeros(1,1,size(phif,3)),cos(thetaf)];
    Rpsif = [cos(psif),-sin(psif),zeros(1,1,size(phif,3));
            sin(psif),cos(psif),zeros(1,1,size(phif,3));
            zeros(1,1,size(phif,3)),zeros(1,1,size(phif,3)),ones(1,1,size(phif,3))];
    
    Rphit = [ones(1,1,size(phif,3)),zeros(1,1,size(phif,3)),zeros(1,1,size(phif,3));
            zeros(1,1,size(phif,3)),cos(phit),-sin(phit);
            zeros(1,1,size(phif,3)),sin(phit),cos(phit)];
    Rthetat = [cos(thetat),zeros(1,1,size(phif,3)),sin(thetat);
              zeros(1,1,size(phif,3)),ones(1,1,size(thetat,3)),zeros(1,1,size(phif,3));
              -sin(thetat),zeros(1,1,size(phif,3)),cos(thetat)];
    
    Rphig = [ones(1,1,size(phig,3)),zeros(1,1,size(phif,3)),zeros(1,1,size(phif,3));
            zeros(1,1,size(phif,3)),cos(phig),-sin(phig);
            zeros(1,1,size(phif,3)),sin(phig),cos(phig)];
    Rthetag = [cos(thetag),zeros(1,1,size(phif,3)),sin(thetag);
              zeros(1,1,size(phif,3)),ones(1,1,size(phig,3)),zeros(1,1,size(phif,3));
              -sin(thetag),zeros(1,1,size(phif,3)),cos(thetag)];
    Rpsig = [cos(psig),-sin(psig),zeros(1,1,size(phif,3));
            sin(psig),cos(psig),zeros(1,1,size(phif,3));
            zeros(1,1,size(phif,3)),zeros(1,1,size(phif,3)),ones(1,1,size(phig,3))];
    
    Rthetaa = [cos(thetaa),zeros(1,1,size(phif,3)),sin(thetaa);
              zeros(1,1,size(phif,3)),ones(1,1,size(thetaa,3)),zeros(1,1,size(phif,3));
              -sin(thetaa),zeros(1,1,size(phif,3)),cos(thetaa)];
          
    Rpsir = [cos(psir),-sin(psir),zeros(1,1,size(psir,3));
            sin(psir),cos(psir),zeros(1,1,size(psir,3));
            zeros(1,1,size(psir,3)),zeros(1,1,size(psir,3)),ones(1,1,size(psir,3))];
             
    Rf = zeros(size(Rthetaf));
    Rt = zeros(size(Rthetat));
    Rg = zeros(size(Rthetag));
    for i = 1:size(WG,1)
        Rf(:,:,i) = Rpsif(:,:,i)*Rthetaf(:,:,i)*Rphif(:,:,i);
        Rt(:,:,i) = Rthetat(:,:,i)*Rphit(:,:,i);
        Rg(:,:,i) = Rphig(:,:,i)*Rthetag(:,:,i)*Rpsig(:,:,i);
    end
    
    xf = WG(:,1).';
    yf = WG(:,2).';
    zf = WG(:,3).';
    posf = [xf;yf;zf];
    posgbls = [-2*lg/5;wg/57;-hg/2]; %b/f (back,front) l/u (lower,upper) s/p (starboard,port)
    posgbus = [-2*lg/5;wg/57;hg/2];
    posgfls = [lg/2;wg/57;-hg/2];
    posgfus = [lg/2;wg/57;hg/2];
    posgblp = [-2*lg/5;-wg/57;-hg/2];
    posgbup = [-2*lg/5;-wg/57;hg/2];
    posgflp = [lg/2;-wg/57;-hg/2];
    posgfup = [lg/2;-wg/57;hg/2];
    posghull = [posgbls,posgbus,posgfls,posgfus,posgblp,posgbup,posgflp,posgfup];
    
    CBN = CB.';

    maxAxes = max([xf,yf,zf]) + 6;
    clear thetaf thetat thetag lf hf p;

    % Adjust and limit framerate
    frames = rate*T;
    if frames>size(xf,2)        % Avoid trying to create video with more frames than are available
       frames = size(xf,2);
       rate = round(frames/T);
    end

    % Creating and opening the video object
    v0 = VideoWriter(filename,filetype); %video with lossless compression
    v0.FrameRate = rate;
    v0.Quality = 100;
    open(v0);
    v1 = VideoWriter(strcat(filename,'-1. Side view'),filetype); %video with lossless compression
    v1.FrameRate = rate;
    v1.Quality = 100;
    open(v1);
    v2 = VideoWriter(strcat(filename,'-2. Front view'),filetype); %video with lossless compression
    v2.FrameRate = rate;
    v2.Quality = 100;
    open(v2);
    clear filename filetype rate

    % Reduce the number of samples used to make the video
    samples = round(linspace(1,size(xf,2),frames));
    
    posf = posf(:,samples);
    Rf = Rf(:,:,samples);
    Rt = Rt(:,:,samples);
    Rg = Rg(:,:,samples);
    Rthetaa = Rthetaa(:,:,samples);
    Rpsir = Rpsir(:,:,samples);
    
    CBN = CBN(:,samples);

    plane = plane(:,:,samples);
    X = X(:,:,samples);
    Y = Y(:,:,samples);
    
    % Connectivity map
    PlotFloatTop = delaunayn(FloatTop,{'QJ'});          
    PlotFloatBottom = delaunayn(FloatBottom,{'QJ'});
    
    Plotglider = delaunayn(posghull.',{'QJ'});
    
    % Generating frames for animation
    figure;
    seaColour = zeros([size(plane(:,:,1)),3]);
    for i = 1:size(posf,2)
        clf;
        
        % Float
        TFloatTop = (repmat(posf(:,i),1,size(FloatTop,1)) + Rf(:,:,i)*FloatTop.').';
        TFloatBottom = (repmat(posf(:,i),1,size(FloatBottom,1)) + Rf(:,:,i)*FloatBottom.').';
        
        tetramesh(PlotFloatTop,TFloatTop,'FaceAlpha',1,'FaceColor','red');
        hold on;
        tetramesh(PlotFloatBottom,TFloatBottom,'FaceAlpha',1,'FaceColor','red');

        % Tether
        postt  = posf(:,i) + Rf(:,:,i)*ft;
        postb  = postt + Rt(:,:,i)*[0;0;lt];
        
        tether = [postt,postb];
        plot3(tether(1,:),tether(2,:),tether(3,:),'g');
        
        % Glider
        Tposghull = zeros(size(posghull));
        for j = 1:length(posghull)
            Tposghull(:,j) = postb(:) + Rg(:,:,i)*(-gt+posghull(:,j));
        end
        
        tetramesh(Plotglider,Tposghull.','FaceAlpha',1,'FaceColor','cyan');
        
        % Aerofoils
        for j = 1:size(gax,1)
            posat = postb + Rg(:,:,i)*(-gt + [gax(j);gay(j);gaz]);
            lengtha = -sign(gay(j))*55*wg/114;
            posagt = postb + Rg(:,:,i)*(-gt + [gax(j);gay(j)+lengtha;gaz]);
            posab = posat + Rg(:,:,i)*Rthetaa(:,:,i)*[-la;0;0];
            posagb = posagt + Rg(:,:,i)*Rthetaa(:,:,i)*[-la;0;0];
            
            aero = [posat,posagt,posagb,posab];

            patch(aero(1,:),aero(2,:),aero(3,:),'r');
        end
        
        % Rudder
        posrgt = postb + Rg(:,:,i)*(-gt + [-2*lg/5;0;-hg/2]);
        posrgb = postb + Rg(:,:,i)*(-gt + [-2*lg/5;0;hg/2]);
        posrt = posrgt + Rg(:,:,i)*Rpsir(:,:,i)*[-lg/5;0;0];
        posrb = posrgb + Rg(:,:,i)*Rpsir(:,:,i)*[-lg/5;0;0];
        rudder = [posrgt,posrgb,posrb,posrt];
        patch(rudder(1,:),rudder(2,:),rudder(3,:),'r');
        
        seaColour(:,:,3) = plane(:,:,i)./(2*max(max(plane(:,:,i))))+0.5*ones(size(plane(:,:,1)));
        surf(X(:,:,i),Y(:,:,i),plane(:,:,i),seaColour);           % Actual Wave
        
        CBN(:,i) = Rthetaf(:,:,i)*CBN(:,i)+posf(:,i);
        scatter3(CBN(1,i),CBN(2,i),CBN(3,i),'p');
    
        % Setting appropriate axis for NED
        set(gca,'YDir','reverse')
        set(gca,'ZDir','reverse')
        axis([posf(1,i)-3 posf(1,i)+3 posf(2,i)-3 posf(2,i)+3 posf(3,i)-1 posf(3,i)+5]);
        daspect([1 1 1]);

        % Frame labels and capturing
        title('WG Model');
        xlabel('x displacement (m)');
        ylabel('y displacement (m)');
        zlabel('z displacement (m)');
        view(45,10);
        drawnow;
        frame = getframe(gcf);
        writeVideo(v0,frame);
        
        view(0,0);
        drawnow;
        frame = getframe(gcf);
        writeVideo(v1,frame);
        
        view(90,0);
        drawnow;
        frame = getframe(gcf);
        writeVideo(v2,frame);
        
        hold off;
    end
    close(v0);
    close(v1);
    close(v2);
    clear Tposfll Tposflu Tposfrl Tposfru Tposgll Tposglu Tposgrl Tposgru frame i
end

% Generating graphs
display('Generating graphs...')
% figure;
% plot(Energy);
% title('Energy of Wave Glider')
% frame = getframe(gcf);
% imwrite(frame.cdata,strcat(foldername,'\results\',testversion,'-Energy.png'));
% 
% figure;
% [ax,h1,h12] = plotyy(0:dt:T,WG(:,1:2),0:dt:T,WG(:,3:end));
% title('System position response to wave');
% axes(ax(1));ylabel('Linear generalised coordinates (m)');
% axes(ax(2));ylabel('Angular generalised coordinates (rad)');
% xlabel('time (s)');
% legend([h1;h12],'x_f','z_f','theta_f','theta_t','theta_g');
% frame = getframe(gcf);
% imwrite(frame.cdata,strcat(foldername,'\results\',testversion,'-genCoords.png'));
% % 
% figure;
% [ax,h2,h22] = plotyy(0:dt:T,WGVel(:,1:2),0:dt:T,WGVel(:,3:end));
% title('System velocity response to wave');
% axes(ax(1));ylabel('Linear velocity (m/s)');
% axes(ax(2));ylabel('Angular velocity (rad/s)');
% xlabel('time (s)');
% legend([h2;h22],'v_x','v_z','dtheta_f','dtheta_t','dtheta_g');
% frame = getframe(gcf);
% imwrite(frame.cdata,strcat(foldername,'\results\',testversion,'-vel.png'));
% 
% figure;
% [ax,h3,h32] = plotyy(0:dt:T,abs(GliderVel(:,2)),0:dt:T,GliderVel(:,1));
% title('Correlation between absolute vertical motion of glider and horizontal velocity of glider');
% axes(ax(1));ylabel('Absolute vertical velocity (m/s)');
% axes(ax(2));ylabel('Horizontal velocity (m/s)');
% xlabel('time (s)');
% legend([h3;h32],'v_z','v_x');
% frame = getframe(gcf);
% imwrite(frame.cdata,strcat(foldername,'\results\',testversion,'-propulsion.png'));
% 
% figure;
% [ax,h3,h32] = plotyy(0:dt:T,GliderPos(:,2),0:dt:T,GliderPos(:,1));
% title('Glider position');
% axes(ax(1));ylabel('Vertical position (m)');
% axes(ax(2));ylabel('Horizontal position (m)');
% xlabel('time (s)');
% legend([h3;h32],'z_g','x_g');
% frame = getframe(gcf);
% imwrite(frame.cdata,strcat(foldername,'\results\',testversion,'-gliderPosition.png'));
% 
% figure;
% [ax,h3,h32] = plotyy(0:dt:T,Wave(:,2),0:dt:T,WG(:,3));
% title('Lag between wave and angle of float');
% axes(ax(1));ylabel('Wave angle (rad)');
% axes(ax(2));ylabel('Angular position (rad)');
% xlabel('time (s)');
% legend([h3;h32],'w_a','theta_f');
% frame = getframe(gcf);
% imwrite(frame.cdata,strcat(foldername,'\results\',testversion,'-lagBetweenWaveAndTheta.png'));
% 
% figure;
% [ax,h3,h32] = plotyy(0:dt:T,GliderBuoyancy(:,5),0:dt:T,WG(:,5));
% title('Glider buoyancy moment and angle');
% axes(ax(1));ylabel('Force (Nm)');
% axes(ax(2));ylabel('Angle (rad)');
% xlabel('time (s)');
% legend([h3;h32],'M_g','Theta_g');
% frame = getframe(gcf);
% imwrite(frame.cdata,strcat(foldername,'\results\',testversion,'-GliderBuoyancyMomentVsAngle.png'));

% figure;
% [ax,h3,h32] = plotyy(0:dt:T,WGVel(:,1),0:dt:T,GliderVel(:,1));
% title('Velocity in the x-direction of float and glider');
% axes(ax(1));ylabel('Float velocity (m/s)');
% axes(ax(2));ylabel('Glider Velocity (m/s)');
% xlabel('time (s)');
% legend([h3;h32],'v_f','v_g');
% frame = getframe(gcf);
% imwrite(frame.cdata,strcat(foldername,'\results\',testversion,'-Speeds.png')); 

% figure;
% plot(0:dt:T,Wave(:,1),0:dt:T,WG(:,2));
% title('Sea state at float position and float vertical position');
% ylabel('height (m)');
% xlabel('time (s)');
% legend('w_h','z_f');
% frame = getframe(gcf);
% imwrite(frame.cdata,strcat(foldername,'\results\',testversion,'-SeaStateVsFloatHeight.png'));
% 
% figure;
% plot(0:dt:T,GliderThrust);
% title('GliderThrust');
% ylabel('');
% xlabel('time (s)');
% legend('Fx','Fz','Ftf','Ftt','Ftg');
% frame = getframe(gcf);
% imwrite(frame.cdata,strcat(foldername,'\results\',testversion,'-GliderThrustGenCoords.png'));
% 
% figure;
% plot(0:dt:T,GliderThrustBody);
% title('Glider Thrust on body frame');
% ylabel('');
% xlabel('time (s)');
% legend('Fx','Fz');
% frame = getframe(gcf);
% imwrite(frame.cdata,strcat(foldername,'\results\',testversion,'-GliderThrustOnBody.png'));
% 
% figure;
% plot(0:dt:T,Wave(:,1));
% title('Sea state at float position');
% ylabel('height (m)');
% xlabel('time (s)');
% frame = getframe(gcf);
% imwrite(frame.cdata,strcat(foldername,'\results\',testversion,'-SeaState.png'));

display('done')
beep;