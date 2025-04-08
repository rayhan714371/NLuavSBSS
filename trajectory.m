%% Trajectory and State Estimation Simulation for 6-DOF Linearized UAV
% This script simulates the UAV tracking a trajectory from an initial position to the origin
% and demonstrates the convergence of state estimation error to zero

% Load UAV model parameters
run('uavmodel2.m');

%% LQR Controller Design
% Define LQR cost function weights
Q_lqr = diag([100 100 100 ... % Position states (x,y,z)
              10 10 10 ...    % Attitude states (phi,theta,psi)
              1 1 1 ...       % Linear velocity states (vx,vy,vz)
              1 1 1]);        % Angular velocity states (p,q,r)

R_lqr = diag([0.1 1 1 1]);    % Thrust, Roll, Pitch, Yaw torques

% Solve for LQR gain
[K, ~, ~] = lqr(A, B, Q_lqr, R_lqr);

%% State Observer Design - Fixed complex pole placement
% Place observer poles for faster convergence than controller poles
closed_loop_poles = eig(A - B*K);
slowest_pole_re = max(real(closed_loop_poles));

% Create observer poles that are faster than controller poles
% Make sure they are properly defined as complex conjugate pairs
observer_poles = [];

% Add real poles
for i = 1:6
    observer_poles = [observer_poles; 4*slowest_pole_re*i];
end

% Add complex conjugate pairs
for i = 1:3
    real_part = 3*slowest_pole_re;
    imag_part = 0.2*i;
    observer_poles = [observer_poles; 
                     complex(real_part, imag_part);
                     complex(real_part, -imag_part)];
end

% Design observer gain L using place() function
L = place(A', C', observer_poles)';

%% Simulation Parameters
% Initial state - start from a position away from origin
x0 = zeros(12, 1);
x0(1:3) = [10; -20; -5];  % Initial position [x=10, y=-20, z=-5]
                          % The negative z is because z-axis points downward in NED frame

% Target state (origin, hover at 0m)
x_ref = zeros(12, 1);

% Equilibrium thrust for hover (to be added to LQR control)
u_eq = [m*g; 0; 0; 0];

% Initial estimate (with some error)
x0_est = x0 + [1; -1.5; 0.5; 0.05; -0.05; 0.1; 0; 0; 0; 0; 0; 0];

% Simulation time
sim_time = 30;  % 30 seconds of simulation
dt = 0.01;      % 10ms time step
tspan = 0:dt:sim_time;
steps = length(tspan);

%% Preallocate Arrays
x_true = zeros(12, steps);
x_est = zeros(12, steps);
e_est = zeros(12, steps);  % Estimation error
u_control = zeros(4, steps);
y_meas = zeros(12, steps);

% Initialize first step
x_true(:,1) = x0;
x_est(:,1) = x0_est;
e_est(:,1) = x_true(:,1) - x_est(:,1);

%% Simulation Loop
for k = 1:steps-1
    % Generate measurement with noise
    y_meas(:,k) = C * x_true(:,k) + randn(12,1) .* sqrt(diag(R));
    
    % State feedback control using estimated state
    u_control(:,k) = -K*(x_est(:,k) - x_ref) + u_eq;
    
    % Simulate true system dynamics (with nonlinear residuals and process noise)
    proc_noise = randn(12,1) .* sqrt(diag(Q));  % Process noise
    
    % State update equation (Euler integration)
    x_true(:,k+1) = x_true(:,k) + dt * (A*x_true(:,k) + B*u_control(:,k) + proc_noise);
    
    % State observer update
    x_est(:,k+1) = x_est(:,k) + dt * (A*x_est(:,k) + B*u_control(:,k) + L*(y_meas(:,k) - C*x_est(:,k)));
    
    % Calculate estimation error
    e_est(:,k+1) = x_true(:,k+1) - x_est(:,k+1);
end

%% Plot Results
% Figure 1: 3D Trajectory and Position
figure(1);
subplot(2,2,1);
plot3(x_true(1,:), x_true(2,:), -x_true(3,:), 'LineWidth', 1.5); % Negate z for plotting
hold on;
plot3(x_true(1,1), x_true(2,1), -x_true(3,1), 'ro', 'MarkerSize', 10);
plot3(0, 0, 0, 'go', 'MarkerSize', 10);
hold off;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Altitude (m)');
title('UAV 3D Trajectory');
legend('Path', 'Start', 'Target');

subplot(2,2,2);
plot(x_true(1,:), x_true(2,:), 'LineWidth', 1.5);
hold on;
plot(x_true(1,1), x_true(2,1), 'ro', 'MarkerSize', 10);
plot(0, 0, 'go', 'MarkerSize', 10);
hold off;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('UAV Path Projection (XY Plane)');

subplot(2,2,3);
plot(tspan, x_true(1,:), 'r-', tspan, x_true(2,:), 'g-', tspan, -x_true(3,:), 'b-', 'LineWidth', 1.5);grid on;
xlabel('Time (s)');
ylabel('Position (m)');
title('Position vs Time');
legend('X', 'Y', 'Z (altitude)');

subplot(2,2,4);
plot(tspan, x_true(4,:)*180/pi, 'r-', tspan, x_true(5,:)*180/pi, 'g-', ...
     tspan, x_true(6,:)*180/pi, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Attitude vs Time');
legend('Roll', 'Pitch', 'Yaw');

% Figure 2: Estimation Error
figure(2);
subplot(3,1,1);
plot(tspan, e_est(1,:), 'r-', tspan, e_est(2,:), 'g-', tspan, e_est(3,:), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Error (m)');
title('Position Estimation Error');
legend('X error', 'Y error', 'Z error');

subplot(3,1,2);
plot(tspan, e_est(4,:)*180/pi, 'r-', tspan, e_est(5,:)*180/pi, 'g-', ...
     tspan, e_est(6,:)*180/pi, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Error (deg)');
title('Attitude Estimation Error');
legend('Roll error', 'Pitch error', 'Yaw error');

subplot(3,1,3);
plot(tspan, e_est(7,:), 'r-', tspan, e_est(8,:), 'g-', tspan, e_est(9,:), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Error (m/s)');
title('Velocity Estimation Error');
legend('Vx error', 'Vy error', 'Vz error');

% Figure 3: Control Inputs
figure(3);
subplot(2,1,1);
plot(tspan, u_control(1,:), 'b-', 'LineWidth', 1.5);
hold on;
plot(tspan, ones(size(tspan))*u_eq(1), 'r--', 'LineWidth', 1);
hold off;
grid on;
xlabel('Time (s)');
ylabel('Force (N)');
title('Thrust Control Input');
legend('Total Thrust', 'Hover Thrust');

subplot(2,1,2);
plot(tspan, u_control(2,:), 'r-', tspan, u_control(3,:), 'g-', tspan, u_control(4,:), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Torque (N·m)');
title('Attitude Control Inputs');
legend('Roll Torque', 'Pitch Torque', 'Yaw Torque');

% Calculate and display convergence statistics
disp('Trajectory and Estimation Results:');

% Calculate when position converges (within 0.1m of origin)
position_error = vecnorm(x_true(1:3,:));
converged_idx = find(position_error < 0.1, 1);
if ~isempty(converged_idx)
    disp(['Position converged to within 0.1m of origin at t = ', num2str(tspan(converged_idx)), ' seconds']);
else
    disp('Position did not converge to within 0.1m of origin during simulation');
end

% Calculate when estimation error converges (norm < 0.05)
estimation_error = vecnorm(e_est);
est_converged_idx = find(estimation_error < 0.05, 1);
if ~isempty(est_converged_idx)
    disp(['Estimation error converged below 0.05 at t = ', num2str(tspan(est_converged_idx)), ' seconds']);
else
    disp('Estimation error did not converge below 0.05 during simulation');
end

% Final position and estimation errors
final_pos_error = norm(x_true(1:3,end));
final_est_error = norm(e_est(:,end));
disp(['Final position error: ', num2str(final_pos_error), ' m']);
disp(['Final estimation error: ', num2str(final_est_error)]);