%% SCARA RPRR Robot Simulation
% DH Parameters: [a, d, alpha, theta_offset]
clear; clc; close all;

%% Robot Parameters
% L1: Link 1 length (m)
% L2: Link 2 length (m)
% d1: Fixed base height (m)
L1 = 0.4;   
L2 = 0.3;
d1 = 0.1;

L(1) = Link([0,  d1,  L1,  0],    'revolute');
L(2) = Link([0,  0.2, 0,   pi],   'prismatic');  % d2 variable
L(3) = Link([0,  0,   L2,  0],    'revolute');
L(4) = Link([0,  0,   0,   0],    'revolute');

robot = SerialLink(L, 'name', 'SCARA_RPRR');
robot.gravity = [0 0 -9.81];

%% Forward Kinematics
theta1 = pi/4;   % Joint 1 angle (rad)
d2     = 0.15;   % Joint 2 displacement (m)
theta3 = -pi/3;  % Joint 3 angle (rad)
theta4 = 0;      % Joint 4 angle (rad)

q = [theta1, d2, theta3, theta4];
T = robot.fkine(q)   % 4x4 homogeneous transform

%% Inverse Kinematics (Analytical)
% Target position
x = 0.5; y = 0.2; z = 0.05;

% Solve theta3 (elbow)
cos_theta3 = (x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2);
theta3_sol = acos(cos_theta3);   % elbow-up
% theta3_sol = -acos(cos_theta3); % elbow-down

% Solve theta1
theta1_sol = atan2(y, x) - atan2(L2*sin(theta3_sol), L1 + L2*cos(theta3_sol));

% Solve d2 (prismatic — sets height)
d2_sol = d1 - z;

fprintf('IK Solution:\n');
fprintf('  theta1 = %.4f rad (%.2f deg)\n', theta1_sol, rad2deg(theta1_sol));
fprintf('  d2     = %.4f m\n', d2_sol);
fprintf('  theta3 = %.4f rad (%.2f deg)\n', theta3_sol, rad2deg(theta3_sol));

%% Trajectory Simulation
t = linspace(0, 2, 100);   % Time vector

q_start = [0,    0.1,  0,     0];
q_end   = [pi/3, 0.25, -pi/4, pi/6];

% Linear interpolation (jtraj for smooth motion)
Q = jtraj(q_start, q_end, t);

%% Animate
figure('Name','SCARA RPRR Simulation','Color','white');
robot.plot(Q, 'trail', 'r-', 'fps', 30, 'scale', 1.5);
title('SCARA RPRR Trajectory');

%% Jacobian & Velocity
J = robot.jacob0(q);    % Geometric Jacobian at config q
fprintf('\nJacobian:\n'); disp(J);

%% Workspace Plot
figure; hold on; grid on;
N = 500;
pts = zeros(N,3);

for i = 1:N
    q_rand = [rand*2*pi-pi, rand*0.3, rand*2*pi-pi, 0];
    T_i = robot.fkine(q_rand);
    pts(i,:) = T_i.t(1:3)';
end

scatter3(pts(:,1), pts(:,2), pts(:,3), 5, pts(:,3), 'filled');
colorbar; title('SCARA Workspace'); xlabel('X'); ylabel('Y'); zlabel('Z');
