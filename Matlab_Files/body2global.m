function [u, s, C] = body2global(acc, omega, ts, Cprev, uprev, sprev)
%This function initially converts acceleration from body frame to global
%frame based on rotation measured by gyroscope and then integrates once
%the acceleration to obtain velocity.
%-------------------------------------%-----------------------------------%
%   Inputs:  omega ->     Gyroscope angular rates              (3x1)
%            acc --->     Accelerometer measurements           (3x1)
%            ts ---->     Sampling period                      (1x1)
%            Cprev ->     Previous rotation                    (3x3)
%            uprev ->     Previous velocity                    (3x1)
            
%   Outputs: C ----->     The rotation matrix after dt         (3x1)
%            u ----->     Velocity of FFU COM in global frame  (3x1) 
%-------------------------------------%-----------------------------------%
%Define milligravity constant (m/s^2)
gravity = 9.81 * 10^(-3);

%Calculate skew symmetric Omega tensor
omega_tensor = [   0        -omega(3)     omega(2);
                 omega(3)       0        -omega(1);
                -omega(2)    omega(1)        0   ];
     
%Integrate with rectangular rule
B = ts*omega_tensor;

%Approximate magnitude of angular displacement in body frame by rectangular
%integration
sigma = abs(omega)*ts;

%Approximate C using Taylor expansion up to second order
C = Cprev*(eye(3,3)+(sin(sigma)/sigma)*B+((1-cos(sigma))/sigma.^2)*(B*B));

%Transform acceleration from body to global frame
acc_g = C*acc;

%Subtract gravitational bias from vertical component
acc_g(3) = acc_g(3) - gravity;

%Integrate acceleration by rectangular rule to find velocity
u = acc_g*ts + uprev;

%Integrate velocity by rectangular rule to find position
s = u*ts + sprev;
end

