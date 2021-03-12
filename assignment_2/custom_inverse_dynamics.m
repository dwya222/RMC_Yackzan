%% HW2: Custom Inverse Dynamics Function
% David Yackzan

% function takes in three 3x1 matrices: position(q), velocity(dq), and
% acceleration(ddq) with values for each of the 3 actuators.
% Outputs a 3x1 matrix for the required torques of the three joints

function [Tau] = custom_inverse_dynamics(q, dq, ddq)
    % create variables for the input values
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    dq1 = dq(1);
    dq2 = dq(2);
    dq3 = dq(3);
    ddq1 = ddq(1);
    ddq2 = ddq(2);
    ddq3 = ddq(3);
   
    % Create variables for our known values
    g = 9.81;
    m1 = 1;
    m2 = 2;
    m3 = 3;
    L1 = .4;
    L2 = 1;
    L3 = 1;
    Lc1 = L1/2;
    Lc2 = L2/2;
    Lc3 = L3/2;
    
    % Solve for Dq
    Dq = [(3*L2^2*cos(2*q2))/2 - (67*cos(2*q2 + 2*q3))/200 - (917*cos(2*q2))/2000 + Lc2^2*cos(2*q2) + (3*Lc3^2*cos(2*q2 + 2*q3))/2 + (3*L2^2)/2 + Lc2^2 + (3*Lc3^2)/2 + 3*L2*Lc3*cos(q3) + 3*L2*Lc3*cos(2*q2 + q3) + 2579/2000, 0, 0;...
          0, 3*L2^2 + 6*cos(q3)*L2*Lc3 + 2*Lc2^2 + 3*Lc3^2 + 2, 3*Lc3^2 + 3*L2*cos(q3)*Lc3 + 1;...
          0, 3*Lc3^2 + 3*L2*cos(q3)*Lc3 + 1, 3*Lc3^2 + 1]
      
    % Solve for C(q, dq)
    Cq_dq = [-(3/2)*L2^2*sin(2*q2)*(2*dq2) + (67/200)*sin(2*q2 + 2*q3)*(2*dq2)*(2*dq3) + (917/2000)*sin(2*q2)*(2*dq2) - Lc2^2*sin(2*q2)*(2*dq2) - (3/2)*Lc3^2*sin(2*q2 + 2*q3)*(2*dq2)*(2*dq3) - 3*L2*Lc3*sin(q3)*dq3 - 3*L2*Lc3*sin(2*q2 + q3)*(2*dq2)*(dq3), 0 0;...
            0, -Lc3*m3*L2*sin(q3)*dq3, -Lc3*m3*L2*sin(q3)*dq3;...
            0, -Lc3*m3*L2*sin(q3)*dq3, 0]
        
    % Solve for G(q)
    Gq = [0; m2*g*Lc1*cos(q2)+m3*g*L2*cos(q2); m3*g*cos(q3)];
    
    % Solve for the output 3 variable array Tau
    Tau = Dq*ddq + Cq_dq*dq + Gq;