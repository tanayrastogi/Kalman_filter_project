function [ omega_b, acc_b, acc_b_true, omega_b_true] = generateData( ~ )
%This function generates data for FFU complex motion. Initially, Gyroscope
%angular rates are generated based on non-linear motion model provided by
%Euler's equations of rotationally symmetric motion. Afterwards, these
%values are used to fill rotation matrix C, which is used to transform
%expected projectile motion a = [0; -g; 0] to body frame acceleration.
%Finally, gaussian noise is added to all datasets.
%Sub-script "b" in omega_b and acc_b denotes BODY FRAME!
%--------------------------------------------------------------------------

%Output
% omega_b      = angular velocity with noise in body frame      (3x600)
% acc_b        = acceleration with noise in body frame          (3x600)
% omega_b_true = angular velocity without noise in body frame   (3x600)
% acc_b_true   = acceleration without noise in body frame       (3x600)

%--------------------------------------------------------------------------
%Initialization
clear
clc
close all

%Define milligravity constant (m/s^2)
gravity = 9.81 * 10^(-3);
%Define expected acceleration of center of mass based on projectile motion
acc_g = [0; -gravity; 0];
%Define time vector (duration of dataset)
time_vec = linspace(1,600,600); %10 minutes
ts = 3;                         %3 Hz sampling frequency
%Define Moments of Inertia of FFU
%I1 is MoI around x and I3 around Z
I1 = 0.012;  %(kg*m*m)
I3 = 2*I1;
%Define Spinning Velocity (around z)
wz = 8;     %(rad/s)

%Define lambda ratio
lambda = (I1-I3)*wz/I1;
%Define amplitude wxy (wobble)
wxy = 4;    %(rad/s)
%Calculate angular rates according to solution of Euler's equations
wx = wxy * sin(lambda*time_vec);
wy = wxy * cos(lambda*time_vec);

%--------------------------------------------------------------------------
%Calculations

%Pre-allocate C transformation matrix
%This C matrix can be improved given initial assumptions to mitigate
%allocation bias (systematic error)!!!
C = eye(3,3);
%Pre-allocate acceleration (body frame) matrix
acc_b = zeros(3,size(time_vec,2)); %(3x120)
for t=1:size(time_vec,2)
    
    %Calculate skew symmetric Omega tensor
    omega = [wx(t); wy(t); wz];
    omega_tensor = [   0        -omega(3)     omega(2);
                     omega(3)       0        -omega(1);
                    -omega(2)    omega(1)        0   ];
    %Integrate with rectangular rule
    B = ts*omega_tensor;
    %Approximate magnitude of angular displacement in body frame by
    %rectangular integration
    sigma = sqrt(omega(1)^2+omega(2)^2+omega(3)^2)*ts;
    %Approximate C transformation using Taylor expansion up to second order
    C = C*(eye(3,3)+(sin(sigma)/sigma)*B+((1-cos(sigma))/sigma^2)*(B*B));
    %Transform acc_g to acc_b
    acc_b(:,t) = C \ acc_g;
    %C is orthogonal, thus inv(C) = C', but here we have Taylor
    %approximation so probably this does NOT apply
    %acc_b(:,t) = C' * acc_g;
end

omega_b = [wx; wy; wz*ones(1,600)];

% True accleration values
acc_b_true = acc_b;
omega_b_true = omega_b;

% Finally induce white Gaussian noise to all datasets!
acc_b(1,:) = awgn(acc_b(1,:), 17, 'measured');
acc_b(2,:) = awgn(acc_b(2,:), 17, 'measured');
acc_b(3,:) = awgn(acc_b(3,:), 17, 'measured');
omega_b(1,:) = awgn(omega_b(1,:), 17, 'measured');
omega_b(2,:) = awgn(omega_b(2,:), 17, 'measured');
omega_b(3,:) = awgn(omega_b(3,:), 17, 'measured');


% figure
% hold on;
% plot(time_vec, acc_b(1,:));
% plot(time_vec, acc_b(2,:));
% plot(time_vec, acc_b(3,:));
% title('Accelerometer measurements')
% ylabel('Acceleration (m/s^2)')
% xlabel('Time (s)')
% legend('ax', 'ay', 'az')
% 
% figure
% hold on;
% plot(time_vec, omega_b(1,:));
% plot(time_vec, omega_b(2,:));
% plot(time_vec, omega_b(3,:));
% title('Gyroscope measurements')
% ylabel('Angular rates (rad/s)')
% xlabel('Time (s)')
% legend('wx', 'wy', 'wz')

end

