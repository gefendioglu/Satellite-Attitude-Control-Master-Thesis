%% Initialization

clear;
close all;
clc;

%% Constants

%  mu = GM for the Earth (m^3/s^2)
global mu_earth;
mu_earth = 398600.4418e9;
% The eccentricity of Earth (unitless)
global e_earth;
e_earth =  0.08181919;
% The J2 correction constant
global J2;
J2 = 0.00108262668;
% The radius of the Earth (m)
global r_earth;
r_earth = 6378137;
% The rotation rate of earth (rad/s);
global w_earth;
w_earth = 7.292115e-5;
% The number of seconds per day
global secs_per_day;
secs_per_day = 24 * 60 * 60;
% The number of seconds per minute
global secs_per_min;
secs_per_min = 60;
% The amount of radians per revolution
global rads_per_rev;
rads_per_rev = 2 * pi;
% Radian to Degree 
global Radian2Degree;
Radian2Degree = 180/pi;
% Degree To Radian 
global Degree2Radian;
Degree2Radian = pi/180;

%% Two Lines of Elements (TLE)and Orbit Propagators

% TLE = [7090000;1.7145; 1.3302; 0.879; 2.0796;4.2059]; 
TLE_degree = [6972000;097.6;058.0490; 0.0015745;077.4852; 282.8127; 14.910027];
% The initial value of semi-major axis (m)
% semimajor_init = TLE_degree(1);
% The initial value of inclination 
inclination_init = deg2rad (TLE_degree(2));
% The initial value of right ascension of the ascending node (RAAN)
RAAN_init = deg2rad (TLE_degree(3));
% The initial value of eccentricity
eccentricity_init = TLE_degree(4);
% The initial value of argument of perigee
ArgOfPerigee_init = deg2rad (TLE_degree(5));
% The initial value of mean anomaly
MeanAnomaly_init = deg2rad (TLE_degree(6));
% The initial value of mean motion
meanMotion_init = TLE_degree(7);
meanMotion_init = meanMotion_init*2*pi/86400;
semimajor_init = sqrt(mu_earth^(1/3)/meanMotion_init^(2/3)); % meter

%% Satellite Initial Values

w_init = [0; 0; 0];
w_dot_init = [0;0;0];
% q = [q1;q2;q3;q4] --> q4 is scalar; (q1,q2,q3)--> vectoral
q_init = [0; 0; 0; 1];
Hrw_B_init = [0; 0; 0]; 
wRW_init = [0;0;0;0];
% Initial altitude of the satellite (m)
h = 700000; % meter
h_km = 700; % km
% Initial radius of the satellite (m)
R = 6378000 + h; % meter
M = 5.9742e24;
G = 6.669e-11;
% The initial angular velocity of satellite
% w0 = sqrt(M*G*10e-9/(R^3));
w0 = sqrt(mu_earth/(R^3));

%% Actuators Initial Value

rw_SensorToBody = [0 0.4741 0.4741 -0.9482;...
                  -0.9999 0.3333 0.3333 0.3333;...
                   0 -0.8165 0.8165 0 ];
RW_wmax = 300; % 3000 rpm ~ 293.215 rad/sn.
RW_Vmax = 20;   % V
RW_Vmin = 0.5;  % V
RW_Imax = 0.2;  % A
RW_Imin = 0.12; % A
RW_Hmax = 0.12;
RW_Mmax = 0.015; % Nm. 
RW_Ts = (1/10);  % sn.
RW_InnerLoop = (1/1000);% sn.
RW_OuterLoop = (1/100);% sn.

rw_Inertia = 5e-4; % kgm^2
R = 2;    % Ohm
L = 5.2e-3;  % H
b = 10e-6; % 10e-6 coefficient - kg.m^2/s (Nms) 
K = 0.1;  % 0.1 If coefficient Kt = Km 
Km = 0.1; % 0.1 coefficient - V/(rad/s) 
Kt = 0.1; % 0.1 coefficient - Nm/Amper 

% rw_Inertia = 5e-4; % kgm^2
% R = 2;    % Ohm
% L = 5.2e-3;  % H
% b = 10e-6; % 10e-6 coefficient - kg.m^2/s (Nms) 
% K = 0.1;  % 0.1 If coefficient Kt = Km 
% Km = 0.1; % 0.1 coefficient - V/(rad/s) 
% Kt = 0.1; % 0.1 coefficient - Nm/Amper 

%(J) = moment of inertia of the rotor     0.05 kg.m^2
%(b) = motor viscous friction constant    0.1 N.m.s
%(Km)= electromotive force constant       0.01 V/rad/sec
%(Kt)= motor torque constant              0.01 N.m/Amp
%(R) = electric resistance                2 Ohm
%(L) = electric inductance                0.5 H
% rw_Inertia =  0.05 ; % kg.m^2
% b =  0.1 ; 
% K = 0.01;
% R = 2;
% L = 0.5; 

%% Satellite Inertia Matrix

Ts = 0.01;
Isx = 7.066197;
Isy = 6.950219;
Isz = 8.555828;
satInertia = [Isx 0 0 ; 0 Isy 0 ; 0 0 Isz];

%% Equilibrium Points
% For satellite quaternion vector
% For satellite quaternion vector
q = [0;0;0;1];
q1 = q(1,1);
q2 = q(2,1);
q3 = q(3,1);
q4 = q(4,1);
q = [q1; q2; q3; q4];

% For satellite angular velocity - wIB_B
wIB_B = [0;0;0];
wIB_B1 = wIB_B(1,1);
wIB_B2 = wIB_B(2,1);
wIB_B3 = wIB_B(3,1);
wIB_B = [wIB_B1; wIB_B2; wIB_B3];

% For reaction wheel angular momentum 
Hrw_B = [0; 0; 0];
Hrw_B1 = Hrw_B(1,1);
Hrw_B2 = Hrw_B(2,1);
Hrw_B3 = Hrw_B(3,1);
Hrw_B = [Hrw_B1; Hrw_B2; Hrw_B3];

%% State Space Definition
% x = [ wIB_B; q ]            - (7x1)
% u = [ Mtot Hrw]             - (6x1)
% y = Hconf4 =[Hgyro;Hstr]    - (7x1)
% Hrw_B is calculated in RW model
% Mtot = Mc+Md (reaction wheels are using attitude actuators)

A11 = [0, (((Isy-Isz)*wIB_B3)/Isx)-(Hrw_B3/2*Isx), (((Isy-Isz)*wIB_B2)/Isx)+ (Hrw_B2/2*Isx);...
      (((Isz-Isx)*wIB_B3)/Isy)+(Hrw_B3/2*Isy), 0, (((Isz-Isx)*wIB_B1)/Isy)-(Hrw_B1/2*Isy);...
      (((Isx-Isy)*wIB_B2)/Isz)-(Hrw_B2/2*Isz), (((Isx-Isy)*wIB_B1)/Isz)+(Hrw_B1/2*Isz), 0];

A12 = zeros(3,4);

% q = [q1;q2;q3;q4]
A21 = 0.5*[q4, -q3, q2;...
           q3, q4, -q1;...
          -q2, q1, q4;...
          -q1, -q2, -q3];
       
skew_wIB_B = [0, wIB_B3, -wIB_B2, wIB_B1;...
              -wIB_B3, 0, wIB_B1, wIB_B2;...
              wIB_B2, -wIB_B1, 0, wIB_B3;...
              -wIB_B1, -wIB_B2, -wIB_B3, 0];

% skew_wOB_B = [0, wOB_B3, -wOB_B2, wOB_B1;...
%               -wOB_B3, 0, wOB_B1, wOB_B2;...
%               wOB_B2, -wOB_B1, 0, wOB_B3;...
%               -wOB_B1, -wOB_B2, -wOB_B3, 0];
          
A22 = (0.5*skew_wIB_B)+ w0.*[(q1*q3), (q1*q4), (1-q1^2), (-q1*q2);...
                             (q2*q3), (q2*q4), (-q1*q2), (1-q2^2);...
                             (-1+q3^2), (q3*q4), (-q1*q3), (-q2*q3);...
                             (q3*q4), (-1+q4^2), (-q1*q4), (-q2*q4)];
                         
% A22 = (0.5*skew_wIB_B)+ w0.*[(q0*q2), (q0*q3), (1-q0^2), (-q0*q1);...
%                              (q1*q2), (q1*q3), (-q0*q1), (1-q1^2);...
%                              (-1+q2^2), (q2*q3), (-q0*q2), (-q1*q2);...
%                              (q2*q3), (-1+q3^2), (-q0*q3), (-q1*q3)];
                         
A = [A11, A12; A21, A22];

B11 = [(1/Isx), 0, 0;...
       0, (1/Isy), 0;...
       0, 0, (1/Isz)];
   
B12 = [0, wIB_B3/(2*Isx), -wIB_B2/(2*Isx);...
       -wIB_B3/(2*Isy), 0, wIB_B1/(2*Isy);...
       wIB_B2/(2*Isz), -wIB_B1/(2*Isz), 0];
    
B21 = zeros(4,3);

B22 = zeros(4,3);

B = [B11,B12; B21,B22];

D = 0;

% Measurement Matrix
gyro = ones(1,3);
Hgyro = [diag(gyro) zeros(3,4)];
% Hgyro = [1 0 0 0 0 0 0 ;...
%          0 1 0 0 0 0 0 ;...
%          0 0 1 0 0 0 0 ];

str = ones(1,4);
Hstr = [zeros(4,3) diag(str)];
% Hstr = [0 0 0 1 0 0 0 ;...
%         0 0 0 0 1 0 0 ;...
%         0 0 0 0 0 1 0 ;...
%         0 0 0 0 0 0 1 ];

% C_ob_q1 = 2*[q1 q2 q3;...
%             q2 -q1 q0;...
%             q3 -q0 -q1];
%         
% C_ob_q2 = 2*[-q2 q1 -q0;...
%             q1 q2 q3;...
%             q0 q3 -q2]; 
%         
% C_ob_q3 = 2*[-q3 q0 q1;...
%             -q0 -q3 q2;...
%             q1 q2 q3]; 
% 
% C_ob_q0 =2*[q0 q3 -q2;...
%             -q3 q0 q1;...
%             q2 -q1 q0];
        
C_ob_q1 = 2*[q1 q2 q3;...
            q2 -q1 q4;...
            q3 -q4 -q1];
        
C_ob_q2 = 2*[-q2 q1 -q4;...
            q1 q2 q3;...
            q4 q3 -q2]; 
        
C_ob_q3 = 2*[-q3 q4 q1;...
            -q4 -q3 q2;...
            q1 q2 q3]; 
        
C_ob_q4 =2*[q4 q3 -q2;...
            -q3 q4 q1;...
            q2 -q1 q4]; 
        
% The measurement matrix of magnetometers
% Hmgm1 = C_ob_q1*B_orbit;
% Hmgm2 = C_ob_q2*B_orbit;
% Hmgm3 = C_ob_q3*B_orbit;
% Hmgm4 = C_ob_q4*B_orbit;
% Hmgm = [zeros(3,3) Hmgm1 Hmgm2 Hmgm3 Hmgm4]; 

% The measurement matrix of sun sensors 
% Hsus1 = C_ob_q1*SV_orbit;
% Hsus2 = C_ob_q2*SV_orbit;
% Hsus3 = C_ob_q3*SV_orbit;
% Hsus4 = C_ob_q4*SV_orbit;
% Hsus = [zeros(3,3) Hsus1 Hsus2 Hsus3 Hsus4]; 

% The measurement matrix of GPS Receivers - 1 
% Hgps11 = C_ob_q1*r_orbit;
% Hgps12 = C_ob_q2*r_orbit;
% Hgps13 = C_ob_q3*r_orbit;
% Hgps14 = C_ob_q4*r_orbit;
% Hgps1 = [zeros(3,3) Hgps11 Hgps12 Hgps13 Hgps14]; 

% The measurement matrix of GPS Receivers - 1 
% Hgps21 = C_ob_q1*vel_orbit;
% Hgps22 = C_ob_q2*vel_orbit;
% Hgps23 = C_ob_q3*vel_orbit;
% Hgps24 = C_ob_q4*vel_orbit;
% Hgps2 = [zeros(3,3) Hgps21 Hgps22 Hgps23 Hgps24];

% Process Noise Covariance Matrix
Q = 1e-10;
unit6 = ones(6,1);
unit7 = ones(7,1);

% Measurement Noise Covariance Matrix
Rgyro = 1e-12;
Rstr = 1e-12; 
Rmgm = 1e-7; 
Rsus = 1e-7; 
Rgps1 = 1e-7; 
Rgps2 = 1e-7; 

% Measurement Noise Covariance Matrices wrt. different configurations
% Configuration-1: MGM + SS + GPS
% Hconf1 =[];
% Rconf1 =[];

% Configuration-2: GYRO + MGM + SS + GPS
% Hconf2 =[];
% Rconf2 =[];

% Configuration-3: STR + MGM + SS + GPS
% Hconf3 =[];
% Rconf3 =[];

% Configuration-4: GYRO + STR
Hconf4 =[Hgyro;Hstr];
Rconf4 =[diag(ones(3,1))*Rgyro zeros(3,4);...
        zeros(4,3) diag(ones(1,4))*Rstr];
% SYS=ss(A,B,Hconf4,D);
% [MSYS,U] = minreal(SYS);
% [Amin, Bmin, Cmin, Dmin] = ssdata(MSYS);

