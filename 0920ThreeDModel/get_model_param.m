% Function to store and transfer system parameters
function param = get_model_param
%% Setting generic model parameter which can be accessed elsewhere
syms xf yf zf phif thetaf psif phit thetat psig thetag phig thetaa
    
%% waveglider properties

    %float parameters
    param.M_float = 50;                % Mass of floater
    param.CG_float = [0;0;0];           % CG of floater in form [z,-y,x]
    param.l_float = 3.05;               % Length of float
    param.w_float = 0.81;               % Width of float
    param.h_float = 0.23;               % Height of float
    param.A_float = param.l_float*param.w_float;    % Water plane area
    param.Jy_float = param.M_float*(param.h_float^2+param.l_float^2)/12; % J about y-axis
    param.Jx_float = param.M_float*(param.h_float^2+param.w_float^2)/12; % J about x-axis
    param.Jz_float = param.M_float*(param.w_float^2+param.l_float^2)/12; % J about z-axis
    
    param.FloatTop = [                                     % Float Top
    1.5250,         0,   -0.1150;
    1.4750,   -0.1694,   -0.1150;
    1.2000,   -0.3675,   -0.1150;
    0.9250,   -0.4050,   -0.1150;
   -1.5250,   -0.4050,   -0.1150;
   -1.5250,    0.4050,   -0.1150;
    0.9250,    0.4050,   -0.1150;
    1.2000,    0.3524,   -0.1150;
    1.4750,    0.1543,   -0.1150;
    1.5250,         0,   -0.0750;
    1.4750,   -0.1694,   -0.0750;
    1.2000,   -0.3675,   -0.0750;
    0.9250,   -0.4050,   -0.0750;
   -1.5250,   -0.4050,   -0.0750;
   -1.5250,    0.4050,   -0.0750;
    0.9250,    0.4050,   -0.0750;
    1.2000,    0.3524,   -0.0750;
    1.4750,    0.1543,   -0.0750;
    ];
    param.FloatBottom = [                                       % Float Bottom
    1.5150,         0,    0.1150;
    0.7700,    0.1359,    0.1150;
    0.3295,    0.1659,    0.1150;
   -0.8084,    0.1732,    0.1150;
   -1.1706,    0.1530,    0.0833;
   -1.5250,    0.0700,    0.0523;
   -1.5250,   -0.0700,    0.0523;
   -1.1709,   -0.1530,    0.0833;
   -0.8084,   -0.1733,    0.1150;
    0.3295,   -0.1660,    0.1150;
    0.7700,   -0.1359,    0.1150;
    1.5150,         0,   -0.0750;
    1.1519,    0.1857,   -0.0750;
    0.6769,    0.2983,   -0.0750;
    0.0006,    0.3435,   -0.0750;
   -0.9577,    0.2983,   -0.0750;
   -1.3229,    0.1857,   -0.0750;
   -1.5250,    0.0700,   -0.0750;
   -1.5250,   -0.0700,   -0.0750;
   -1.3229,   -0.1857,   -0.0750;
   -0.9577,   -0.2983,   -0.0750;
    0.0034,   -0.3435,   -0.0750;
    0.6769,   -0.2983,   -0.0750;
    1.1519,   -0.1857,   -0.0750;
    ];
    
    %glider parameters
    param.M_glider = 150;              % Mass of glider
    param.CG_glider = [0;0;0];           % CG of glider in form [x,y,z]
    param.l_glider = 2.13;              % length of glider
    param.w_glider = 1.42;              % width of glider
    param.h_glider = 0.21;              % height of glider
    param.A_glider = param.l_glider*param.w_glider; % water plane area
    param.V_glider = param.l_glider*param.w_glider*param.h_glider*0.002; 
                                        % displaced volume; note volume is multiplied by percentage 
                                            %of expected displaced volume
                                            %since the glider is not a
                                            %solid cuboid
    param.Jy_glider = param.M_glider*(param.h_glider^2+param.l_glider^2)/12; % J about y-axis
    param.Jx_glider = param.M_glider*(param.h_glider^2+param.w_glider^2)/12; % J about x-axis
    param.Jz_glider = param.M_glider*(param.w_glider^2+param.l_glider^2)/12; % J about z-axis
    
    %tether parameters
    param.ftx_tether = 0;
    param.fty_tether = 0;
    param.ftz_tether = 0;
    param.gtx_tether = 0;               %glider to bottom tether in x-dir
    param.gtz_tether = -param.h_glider/2;
    param.gtz_tether = 0;
    param.M_tether = 10;
    param.CG_tether = [0;0;0];           % CG of tether in form [z,x,y]
    param.l_tether = 4;                  %length of tether
    param.r_tether = 0.005;              %radius of tether
    param.A_tether = pi*param.r_tether^2;    %water plane area
    param.V_tether = param.l_tether*param.A_tether;    %volume of tether
    param.Jy_tether = param.M_tether*param.r_tether^2/4 + param.M_tether*param.l_tether^2/3;
                                        % J about y-axis: J = mr^2/4 + ml^2/3
    param.Jx_tether = param.Jy_tether;
                                        
    %aerofoil parameters
    param.CG_aero = [0;0;0];           % CG of aero in form [x,z,-y]
    dAG = param.l_glider/(6+4);
    param.dAG_aero = dAG;
    param.gax_aero = [4*dAG;3*dAG;2*dAG;0;-1*dAG;-2*dAG;4*dAG;3*dAG;2*dAG;0;-1*dAG;-2*dAG];% Glider to aerofoil in x-dir
    param.gay_aero = [param.w_glider/2;param.w_glider/2;param.w_glider/2;param.w_glider/2;param.w_glider/2;param.w_glider/2;
                        -[param.w_glider/2;param.w_glider/2;param.w_glider/2;param.w_glider/2;param.w_glider/2;param.w_glider/2]];
    param.gaz_aero = 0;
    param.M_aero = 2;
    param.l_aero = 0.176;
    param.w_aero = 1.42;
    param.t_aero = 0.01;
    param.k_aero = 3600; % 500N force at x = 0.1375
    param.l0_aero = 0.037;
    param.V_aero = param.l_aero*param.w_aero*param.t_aero;
%     param.Jy_aero = param.M_aero*(param.l_aero^2 + param.t_aero^2 + (param.l_aero/2)^2);
%                                         % J about y-axis: J = m(l^2 + t^2)
%                                         % + m*r^2
    param.Jy_aero = 0.4472;

%% Set parameters

param.g    = 9.81;

    %3D 12DOF D-H reference frames
param.A3D = [0,-pi/2,yf,-pi/2;
             0,-pi/2,xf,-pi/2;
             0,-pi/2,zf,-pi/2;
             0,0,0,psif;
             0,-pi/2,0,-pi/2+thetaf;
             0,-pi/2,0,phif;
             0,pi/2,0,-phif+phit;
             0,pi/2,0,-thetaf+thetat;
             param.l_tether,-pi/2,0,-phif+phig;
             0,pi/2,0,-thetat+pi/2+thetag;
             0,pi/2,0,-psif+psig;
             0,-pi/2,0,thetaa-thetag];
end
