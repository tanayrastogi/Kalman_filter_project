function [Z,C] = measurements(X,Cprev,omega_b,acc_b,dt)
% The function to calculate the true measurements from the sensor.
% GPS sensor --> position (longitude, latitide, altitude)
% Acclerometer --> Accleration (ax, ay, az)
% Gyroscope --> Anglular velocities (wx,wy,wz)

%-------------------------------------%-----------------------------------%
% Input
%   X = The previous state values before the measurements   (6x1)
%   Cprev = The previous rotation matrix before the meas    (3x3)
% Output
%   Z = position and veloctiy measurements                  (6x1)
%-------------------------------------%-----------------------------------%

% Calculate velocity from acclerometer in global frame
sprev = X(1:3);
uprev = X(4:6);
[u, s, C] = body2global(acc_b, omega_b, dt, Cprev, uprev, sprev);

% Convert GPS signal to XYZ points
% Pos = GPS_convertor(lng,lat, ht);
% Pos = [0.5;0.75;0.1];

% Return the measurements
Z = [s;u];

end
