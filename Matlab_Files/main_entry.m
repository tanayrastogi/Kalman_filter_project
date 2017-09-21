clear;

% For the model
% x[k] = A*x[k-1] + B*u[k]+ w;
% z[k] = C*x[k] + v;

% Predict
% Xbar[k] = A*X[k-1] + B*u[k]
% Pbar[k] = A*P[k-1](A.') + Q
% Update
% K = Pbar[k]*(C.')(inv(C*Pbar[k]*(C.') + R))
% X[k] = Xbar[k] + K(Z[k] - H*Xbar[k])
% P[k] = (I - K*C)*Pbar[k]

% State model
% [posx_k]   [1 0 0  dt 0 0][posx_k-1]   [    0   ]      
% [posy_k]   [0 1 0  0 dt 0][posy_k-1]   [-0.5dt^2] 
% [posz_k] = [0 0 1  0  0 0][posz_k-1] + [    0   ][g]  + w_k   
% [ vx_k ]   [0 0 0  1  0 0][ vx_k-1 ]   [    0   ]      
% [ vy_k ]   [0 0 0  0  1 0][ vy_k-1 ]   [   -dt  ]
% [ vz_k ]   [0 0 0  0  0 1][ vz_k-1 ]   [    0   ]

% Measurement model
% [posx_k]   [1 0 0 0 0 0][posx_k]
% [posy_k] = [0 1 0 0 0 0][posy_k] 
% [posz_k]   [0 0 1 0 0 0][posz_k]   + v_k
% [ vx_k ]   [0 0 0 1 0 0][ vx_k ]
% [ vy_k ]   [0 0 0 0 1 0][ vy_k ]
% [ vz_k ]   [0 0 0 0 0 1][ vz_k ]


% Generate Sensor data
[omega_b,acc_b, ~, ~] = generateData();

% Generate true sensor values without noise
[~, ~, acc_b_true, omega_b_true] = generateData();

% Kalman Filter
    % Initialization of all the variables at for time k=0
      [X, P, A, B, H, Q, R, K, Z, C, u, dt] = initialization(omega_b_true(:,1), acc_b_true(:,1));
    
    % Initialization for the true values
    Z_true = trueData(X);
    final = zeros(6,600);

    % Initialize results plot
    figure(1)
    hold on;
%     prompt = 'enter';
%     dummy = input(prompt); 
 
    
for i=2:600;
  
    % Measurements from sensor
    [Z,Ct] = measurements(X, C, omega_b(:,i), acc_b(:,i), dt);
    C = Ct;
    
    % Filter    
    [X_k, P_k] = kf(X, P, Z, u, A, B, Q, R, H);
    
    % For next iteration
    X = X_k;
    P = P_k;

       
    % Plot figures
    plot(X(1), X(2), 'ob');
    plot(Z_true(1,i), Z_true(2,i), 'oy');
    title('Ballistic Projectile Motion')
    ylabel('y dir (m)')
    xlabel('x dir (m)')
    
    pause(0.01)
end


