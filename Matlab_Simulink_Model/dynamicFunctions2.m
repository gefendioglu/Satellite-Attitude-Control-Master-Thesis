function x = dynamicFunctions2(x,u)

dt = 0.01; 
% Get states
%-----------

w_IB = x( 1: 3);
q = x( 4: 7);
satTorque = u(1:3);
Hrw_B = u(4:6);
%-----------

% R = 6378 + h_km; 
h = 700000;
R = 6378000 + h; % meter
mu_earth = 398600.4418e9;
w0 = sqrt(mu_earth/(R^3));
%-----------

Isx = 7.066197;
Isy = 6.950219;
Isz = 8.555828;
satInertia = diag([Isx Isy Isz]);
satInvInertia = inv(satInertia);

q1 = q(1,1);
q2 = q(2,1);
q3 = q(3,1);
q4 = q(4,1);

DCM = [2*(q1*q2 + q4*q3);...
       (q4^2)-(q1^2)+(q2^2)-(q3^2);...
       2*(q2*q3 - q4*q1)];
   
% Kinematics
%-----------
w_OB = zeros(3,1);
w_OB(1,1) = w_IB(1,1) + DCM(1,1)* w0;
w_OB(2,1) = w_IB(2,1) + DCM(2,1)* w0;
w_OB(3,1) = w_IB(3,1) + DCM(3,1)* w0;

Sw_OB = [0, w_OB(3),-w_OB(2),w_OB(1);...
       -w_OB(3), 0, w_OB(1), w_OB(2);...
        w_OB(2), -w_OB(1), 0, w_OB(3);...
       -w_OB(1), -w_OB(2), -w_OB(3), 0];

qD   = Sw_OB*q;  
qD   = 0.5*qD;
qDot = qD;

% Dynamics
%---------

% Sw_IB = [0 -w_IB(3) w_IB(2);...
%          w_IB(3) 0 -w_IB(1);...
%         -w_IB(2) w_IB(1) 0];
% rwInertia = eye(3,3)*(5e-4);
% Hrw_B = rwInertia * w_RW;
% satTorque = Md - Hrw_B_dot

eulerCoupling = cross( w_IB, (satInertia * w_IB + Hrw_B));
wDot = satInvInertia*(satTorque - eulerCoupling);

% Collect the state vector
%-------------------------
x = x + [wDot; qDot]*dt;

end

