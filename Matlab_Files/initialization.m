function [X, P, A, B, H, Q, R, K, Z, C, u, dt]= initialization(omega_b1, acc_b1)
% Initialization of the all the variables in Kalman Filter for time
% instance k = 1

%-------------------------------------%-----------------------------------%
% Inputs
% omega_b1 = angular velocities 
%             without noise at t = 1 (3x1)
% omega_b1 = accleration  
%             without noise at t = 1 (3x1)

% Outputs
% X = state vector                   (6x1)
% P = Prediction variance matrix     (6x6)
% A = state matrix                   (6x6)
% B = control matrix                 (6x1)
% H = measurement matrix             (6x6)
% Q = Process noise                  (6x6)
% R = Measurement noise              (6x6)
% K = Kalman Gain                    (6x6)
% Z = True measurements              (6x1)
% C = The orientation of the FFU     (3x3)
% u = control variable               (1x1)
% dt= smapling period                (1x1)

%-------------------------------------%-----------------------------------%

% Sampling Time
dt = 1; % sec

% Process Noise in body frame
Q = 1*eye(6,6);

% Measurement Noise
R = 2*eye(6,6);
R(4,1) = 1;
R(5,2) = 1;
R(6,3) = 1;
R(1,4) = 1;
R(2,5) = 1;
R(3,6) = 1;
R = 100*R;

% Control vector;
u = 9.81 * 10^-3; % m/sec2 (miliGravity)

%----------------------------------%--------------------------------------%

 % A matrix
 A = eye(6,6);
 A(1,4) = dt;
 A(2,5) = dt;
 
 % B matrix
 B = zeros(6,1);
 B(2,1) = -0.5*dt^2;
 B(5,1) = -dt;

 % Measurements
 Z = ones(6,1);
 H = eye(6,6);
 
 % Kalman Gain
 K = zeros(6,6);

 % Rotation Matrix
 % The orientation of the FFU at time instnace t = 0, after the ejection
 C = eye(3,3);

%----------------------------------%--------------------------------------%
% We will use the measurements for the instance t=1 as the inital
% states, X[1] and then start estimating from the t=2 using
% filter.

% Intial State Vector at time t = 0
X0 = zeros(6,1);
X0(4) = 1;
X0(5) = 0.5;

% Measurement at time t = 1;
[X,Ct] = measurements(X0, C, omega_b1, acc_b1, dt);
C = Ct;

% Initial Prediction Vector
    % Initial prediction variance in position;
    P = zeros(6,6);
    P(1,1) = R(1,1); % m (pos_x)
    P(2,2) = R(2,2); % m (pos_y)
    P(3,3) = R(3,3); % m (pos_z)
    
    % Initial prediction variance in velocity
    P(4,4) = R(4,4); % m/sec (vel_x)
    P(5,5) = R(5,5); % m/sec (vel_y)
    P(6,6) = R(6,6); % m/sec (vel_z)
    
    % Initial prediction covariance in velocity and position
    P(4,1) = R(1,1)/dt;
    P(5,2) = R(2,2)/dt;
    P(6,3) = R(3,3)/dt;
    P(1,4) = R(4,4)/dt;
    P(2,5) = R(5,5)/dt;
    P(3,6) = R(6,6)/dt;    
%----------------------------------%--------------------------------------%
end


    