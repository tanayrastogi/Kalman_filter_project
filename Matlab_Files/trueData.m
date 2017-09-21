function [ Z_true ] = trueData( X_init )
%This functions returns the ideal values of position and velocity following
% ideal projectile motion. This act as a reference to see the performance
% of the Kalman filter
%-------------------------------------%-----------------------------------%
% Input
% X_init = Initial states at time t = 1          (3x1)

% Output
% Z_true = the true states for whole time period (6x600)
%-------------------------------------%-----------------------------------%

%Define milligravity constant (m/s^2)
gravity = 9.81 * 10^(-3);

Z_true = zeros(6,600);
X = zeros(6,600);

X(:,1) = X_init;

%%Define time vector (duration of dataset)
    time_vec = linspace(1,600,600); %10 minutes

for i = 1:600
    
    Z_true(1,i) = X(1,1) + X(4,1)*i;
    Z_true(2,i) = X(2,1) + X(5,1)*i - 0.5*gravity*(i^2);
    Z_true(3,i) = X(3,1);
    Z_true(4,i) = X(4,1);
    Z_true(5,i) = X(5,1) - gravity * i;
    Z_true(6,i) = 0;
end

end

