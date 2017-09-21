function [X,P] = kf(X0, P0, Z, u, A, B, Q, R, H)
% The function runs the Kalman filter for one time instance
%-------------------------------------%-----------------------------------%
% Inputs
% X0 = the state for the previous step                    (6x1) 
% P0 = prediction error                                   (6x6) 
% u = control signal                                      (1x1)
% z = measurements from the GPS sensor and acclerometer   (6x1)

% Output
% x = updated state information after                     (6x1) 
%     integrating the measurements
% P = updated prediction error                            (6x6)
%-------------------------------------%-----------------------------------%

% Kalman Filter
% Predicton step
    Xbar = A*X0 + B*u;          % Estimated states 
    Pbar = A*P0*(A') + Q;       % Covariance of the states
    
% Update step
    % Kalman gain
    K = Pbar*(H')*(inv(H*Pbar*(H') + R));
    
    X = Xbar + K*(Z - H*Xbar);    % States updated after measurements
    P = (eye(6,6) - K*H)*Pbar;    % Covariance updated
    
end

    

