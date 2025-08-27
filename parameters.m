%% 6DoF dynamic model of teetering helicopter 
% parameters from the BELL AH-1 Cobra
% controls:= [ A1s, B1s, theta_mr, theta_tr ]
clear variables
clc

%% solver parameters
%RK4
ST = 0.01; % time step [s]

%% conversion factors
Lb_Kg   = 0.453592; %lb to kilograms
Ft_m    = 0.3048; %feet to meters
Kn_mps  = 0.514444; %knots to meters per second
Slug_Kg = 14.593903; %slugs to kilograms
hp_W    = 745.700; %horsepower to watts

%% flight condition
% steady and symmetrical, NED frame
fc  = [  0*Kn_mps ; % horizontal velocity [kts-->m/s]
         0        ; % side velocity [m/s]
         0       ]; % rate of climb [m/s]

%% initial conditions 
% position:
Xe_0   = [ 16404 0 -100 ]'*Ft_m; % initial position in NED (north east down) frame [ft-->m]
latref = 39.24744998520548; % reference latitude for earth fixed-origin [deg]
lonref = 9.050782438199448; % reference longitude for earth fixed-origin [deg]
href   = 0; % reference altitude for earth-fixed origin (zero if on surface but not necessarely at sea level!) [m]
% trim variables:
euler0 = [ 0 0 0 ]'*pi/180; % initial attitude (roll, pitch, yaw, angles) [deg-->rad]
V0     = eul2rotm(euler0','XYZ')'*[ fc(1) fc(2) fc(3) ]'; % initial velocity in body frame [m/s] 
omega0 = [ 0 0 0 ]'; % initial rotational velocity (roll, pitch, yaw rate) [rad/s] 
load('hover_trim.mat')

%% inertial parameters
m   = 8000*Lb_Kg; % take-off mass [lb-->kg]
Ixx = 2700*Slug_Kg*Ft_m^2;
Iyy = 12800*Slug_Kg*Ft_m^2;
Izz = 10800*Slug_Kg*Ft_m^2;
Ixz = 950*Slug_Kg*Ft_m^2;
inertia = [ Ixx 0 -Ixz; 0 Iyy 0; -Ixz 0 Izz ]; % matrix of inertia in body frame [slug*ft^2-->kg*m^2]

%% environmental conditions
rho = 1.225; % air density at sea level [kg/m3]
g = 9.81; % gravity acceleration [m/s2]

%% controls input range ([deg])
lon_cyc_range = [ -15 15 ]; % positive aft
lat_cyc_range = [ -15 15 ]; % positive right
mr_coll_range = [ 7.5 25 ]; 
tr_coll_range = [ -20 20 ]; 

%% main rotor
% teetering rotor
% counterclockwise rotation
% assumed uniform inflow equal to hover value
% rotor drag and flapping are considered 
OMEGA   = 314*(pi/30); % angular speed (supposed constant) [rpm]-->[rad/s]           
BLADES  = 2; % number of blades 
ROTOR   = 22*Ft_m; % rotor radius [ft-->m]
CHORD   = 2.25*Ft_m; % mean aerodynamic chord [ft-->m]
ASLOPE  = 6.28; % blade lift curve slope [1/rad]
AOP     = 2.75*pi/180; % precone angle (required only for teetering rotor) [deg-->rad]
GAMMA   = 5.216; % Lock number: ratio of inertial to aerodynamic flapping moment
THETT   = -10*(pi/180); % amount of twist from tip to root (theta_tip-theta_hub) [rad]
T_flap  = 16/GAMMA/OMEGA; % time constant for teetering main rotor flapping
CD_mr   = [0.009 0 0.3]; %[CD0,CD1,CD2] C_D = C_D0+C_D1*alpha+C_D2*alpha^2
k_ind = 1;
L_mr = [  -0.1  ; % aft of cg
            0   ;
          -2.03 ];

%% tail rotor
% bottom_forward rotation
% no effect of wind or flapping considered
AOPTR     = 0; % precone angle (required only for teetering rotor) [rad]
OMEGATR   = 1608.5*(pi/30); % angular speed (supposed constant) [rpm]-->[rad/s]           
BLADESTR  = 2; % number of blades 
ROTORTR   = 4.25*Ft_m; % rotor radius [ft-->m]
CHORDTR   = 0.7*Ft_m; % mean aerodynamic chord [ft-->m]
ASLOPETR  = 6.28; % blade lift curve slope [1/rad] 
GAMMATR   = 2.23; % Lock number: ratio of inertial to aerodynamic flapping moment
THETTTR   = 0*(pi/180); % amount of twist from tip to root (theta_tip-theta_hub) [rad]
                        % Theta = Theta0 + THETT*r; r=y/R; Theta0: blade root collective pich
CD_tr     = [0.009 0 0.3]; %[CD0,CD1,CD2]
L_tr = [  -8.25 ; 
            0   ; 
          -1.15 ];

%% fuselage equivalent flat plate drag area model 
L_fus = [ -0.1; % behind cg
            0 ; 
          0.48];% under cg
% equivalent drag area [ft^2-->m^2]:
Ax = 5.5*0.3048^2; 
Ay = 156.1*0.3048^2; 
Az = 84.7*0.3048^2;


%% linear dynamic model of the hovering helicopter 
% hyphothesis: thrust approximately constant to hover trim value

% parameters:
lz_mr = 2.03; % vertical distance between main rotor and cg [m]
lx_tr = 8.25; % longitudinal distance between tail rotor and cg [m]
k1_tr = 300*180/pi; % linearized relation between tail rotor thrust and collective [N/rad]
T_mr_trim = 35540; % trim hover value of main rotor thrust [N]
Q_mr_trim = 15614.5; % trim hover value of main rotor counter-torque [Nm]
cyc_range = 15*pi/180; % symmetric range of cyclic control for normalization
tr_range  = 20*pi/180; % symmetric range of tail rotor collective control for normalization

% affine attitude model, rates as state
% x_dot = f+B*u_dim+delta = f+B*B_dim*u+delta
% x = [ p, q, r ]
% u = [ deltaA, deltaB, deltaP ]
% u_dim = [ A1s, B1s, theta_tr ]

f  = [ 0; 0; Q_mr_trim/Izz];

B  = [ 1/Ixx*lz_mr*T_mr_trim,          0,                 0          ;
                 0,         -1/Iyy*lz_mr*T_mr_trim,       0          ;
                 0,                    0,         -1/Izz*lx_tr*k1_tr ];
B_dim = [ cyc_range    0       0;
              0   -cyc_range   0;
              0        0    tr_range];

B_inv = inv(B*B_dim);