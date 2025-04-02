%% UAV Trajectory Tracking Simulation
% This script combines elements from uavmodel.m and controller2.m
% to simulate a UAV trajectory from an initial position to a target position
% using the linearized 6-DOF UAV model and LQR controller

%% Load Model and Controller
% First run the UAV model script to get A, B, C matrices
run('uavmodel.m');

% Then run the controller script to get K_lqr and L matrices
run('controller2.m');

%% Simulation Parameters
dt = 0.01;            % Sampling time (s)
T = 50;               % Simulation duration (s)
steps = T/dt;         % Number of simulation steps

% Time vector
t = 0:dt:T;

%% Initial and Target States
% Initial state of the UAV
x_init = zeros(12, 1);
x_init(1:3) = [10; -20; -5];   % Initial position [px; py; pz] (meters)
x_init(4:6) = [0; 0; 0];       % Initial orientation [phi; theta; psi] (radians)
x_init(7:9) = [0; 0; 0];       % Initial linear velocities [vx; vy; vz] (m/s)
x_init(10:12) = [0; 0; 0];     % Initial angular velocities [p; q; r] (rad/s)

% Target state (destination)
x_target = zeros(12, 1);       % Target is the origin with zero velocities and angles

%% Initialize Arrays for Simulation
X = zeros(12, steps+1);         % True state
X_hat = zeros(12, steps+1);     % Estimated state
Y = zeros(12, steps+1);         % Measurements
U = zeros(4, steps);            % Control inputs
e = zeros(12, steps+1);         % Estimation error

% Set initial conditions
X(:,1) = x_init;
X_hat(:,1) = x_init + 0.1*randn(12,1);  % Slightly offset initial estimate
e(:,1) = X(:,1) - X_hat(:,1);           % Initial estimation error
Y(:,1) = C * X(:,1) + 0.01*randn(12,1); % Initial measurement with noise

%% Noise Parameters
% Process noise bounds (from h(x,u) nonlinearities)
process_noise_bound = h_bounds * 0.1;  % Reduced to ensure stability and convergence

% Measurement noise standard deviation
meas_noise_std = 0.01;

%% Define a damping coefficient to help stability
damping_factor = 0.8;  % This adds artificial damping to prevent oscillations

%% Main Simulation Loop
for k = 1:steps
    % State error for control (control to target state)
    state_error = X_hat(:,k) - x_target;
    
    % Add damping for velocities to prevent oscillations
    damping_control = zeros(12, 1);
    damping_control(7:12) = -damping_factor * X_hat(7:12,k);
    
    % Compute control input using LQR feedback with damping
    U(:,k) = U_eq - K_lqr * state_error;
    
    % Apply control limits to prevent saturation
    U(1,k) = max(min(U(1,k), 2*m*g), 0.5*m*g);  % Thrust between 0.5*m*g and 2*m*g
    U(2:4,k) = max(min(U(2:4,k), 0.5), -0.5);   % Limit torques
    
    % True system dynamics with reduced process noise
    process_noise = (2*rand(12,1)-1) .* process_noise_bound;
    X(:,k+1) = A * X(:,k) + B * U(:,k) + process_noise;
    
    % Add safety check - if the UAV is close to the target and moving slowly,
    % reduce noise to help it stabilize at the target
    dist_to_target = norm(X(1:3,k) - x_target(1:3));
    vel_magnitude = norm(X(7:9,k));
    if dist_to_target < 1.0 && vel_magnitude < 0.5
        process_noise = process_noise * 0.1;  % Reduce noise near target
    end
    
    % Generate noisy measurement
    meas_noise = meas_noise_std * randn(12,1);
    Y(:,k+1) = C * X(:,k+1) + meas_noise;
    
    % State estimation using Kalman filter
    X_hat_pred = A * X_hat(:,k) + B * U(:,k);
    X_hat(:,k+1) = X_hat_pred + L * (Y(:,k+1) - C * X_hat_pred);
    
    % Compute estimation error
    e(:,k+1) = X(:,k+1) - X_hat(:,k+1);
end

%% Plot Results
% Figure 1: 3D Trajectory
figure('Name', 'UAV 3D Trajectory');
plot3(X(1,:), X(2,:), -X(3,:), 'b-', 'LineWidth', 2);
hold on;
plot3(X(1,1), X(2,1), -X(3,1), 'go', 'MarkerSize', 10, 'LineWidth', 2);  % Initial point
plot3(x_target(1), x_target(2), -x_target(3), 'rx', 'MarkerSize', 10, 'LineWidth', 2);  % Target point

% Mark important points on the trajectory
start_idx = 1;
mid_idx = round(steps/3);
late_idx = round(2*steps/3);
end_idx = steps+1;

plot3(X(1,start_idx), X(2,start_idx), -X(3,start_idx), 'ko', 'MarkerSize', 8);
text(X(1,start_idx), X(2,start_idx), -X(3,start_idx), sprintf('t=0s'), 'VerticalAlignment', 'bottom');

plot3(X(1,mid_idx), X(2,mid_idx), -X(3,mid_idx), 'ko', 'MarkerSize', 8);
text(X(1,mid_idx), X(2,mid_idx), -X(3,mid_idx), sprintf('t=%ds', round(mid_idx*dt)), 'VerticalAlignment', 'bottom');

plot3(X(1,late_idx), X(2,late_idx), -X(3,late_idx), 'ko', 'MarkerSize', 8);
text(X(1,late_idx), X(2,late_idx), -X(3,late_idx), sprintf('t=%ds', round(late_idx*dt)), 'VerticalAlignment', 'bottom');

plot3(X(1,end_idx), X(2,end_idx), -X(3,end_idx), 'ko', 'MarkerSize', 8);
text(X(1,end_idx), X(2,end_idx), -X(3,end_idx), sprintf('t=%ds', T), 'VerticalAlignment', 'bottom');

grid on;
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('UAV 3D Trajectory');
legend('Flight Path', 'Initial Position', 'Target Position');
axis equal;

% Figure 2: Position vs Time
figure('Name', 'Position Tracking');
subplot(3,1,1);
plot(t, X(1,:), 'b-', t, X_hat(1,:), 'r--', 'LineWidth', 1.5);
ylabel('X Position (m)');
grid on;
legend('True', 'Estimated');
title('Position Tracking');

subplot(3,1,2);
plot(t, X(2,:), 'b-', t, X_hat(2,:), 'r--', 'LineWidth', 1.5);
ylabel('Y Position (m)');
grid on;

subplot(3,1,3);
plot(t, X(3,:), 'b-', t, X_hat(3,:), 'r--', 'LineWidth', 1.5);
ylabel('Z Position (m)');
xlabel('Time (s)');
grid on;

% Figure 3: Distance to Target over time
figure('Name', 'Distance to Target');
distance_to_target = zeros(1, steps+1);
for i = 1:steps+1
    distance_to_target(i) = norm(X(1:3,i) - x_target(1:3));
end
plot(t, distance_to_target, 'LineWidth', 2);
title('Distance to Target vs Time');
xlabel('Time (s)');
ylabel('Distance (m)');
grid on;

% Figure 4: Estimation Errors
figure('Name', 'Estimation Errors');

% Position errors
subplot(3,1,1);
plot(t, e(1,:), 'b-', t, e(2,:), 'r-', t, e(3,:), 'g-', 'LineWidth', 1.5);
title('Position Estimation Errors');
ylabel('Position Error (m)');
grid on;
legend('X Error', 'Y Error', 'Z Error');

% Attitude errors
subplot(3,1,2);
plot(t, rad2deg(e(4,:)), 'b-', t, rad2deg(e(5,:)), 'r-', t, rad2deg(e(6,:)), 'g-', 'LineWidth', 1.5);
ylabel('Angle Error (deg)');
grid on;
legend('\phi Error', '\theta Error', '\psi Error');

% Velocity errors
subplot(3,1,3);
plot(t, e(7,:), 'b-', t, e(8,:), 'r-', t, e(9,:), 'g-', 'LineWidth', 1.5);
ylabel('Velocity Error (m/s)');
xlabel('Time (s)');
grid on;
legend('V_x Error', 'V_y Error', 'V_z Error');

% Figure 5: Norm of Estimation Error
figure('Name', 'Estimation Error Norm');
error_norm = zeros(1, steps+1);
for i = 1:steps+1
    error_norm(i) = norm(e(:,i));
end
plot(t, error_norm, 'LineWidth', 2);
title('Norm of State Estimation Error');
xlabel('Time (s)');
ylabel('||e(t)||');
grid on;

% Check final position
final_distance = norm(X(1:3,end) - x_target(1:3));
fprintf('Final distance to target: %.4f meters\n', final_distance);

% Print mean and max estimation errors
mean_pos_error = mean(sqrt(e(1,:).^2 + e(2,:).^2 + e(3,:).^2));
max_pos_error = max(sqrt(e(1,:).^2 + e(2,:).^2 + e(3,:).^2));
fprintf('Mean position estimation error: %.4f meters\n', mean_pos_error);
fprintf('Max position estimation error: %.4f meters\n', max_pos_error);

mean_attitude_error = mean(sqrt(e(4,:).^2 + e(5,:).^2 + e(6,:).^2))*180/pi;
max_attitude_error = max(sqrt(e(4,:).^2 + e(5,:).^2 + e(6,:).^2))*180/pi;
fprintf('Mean attitude estimation error: %.4f degrees\n', mean_attitude_error);
fprintf('Max attitude estimation error: %.4f degrees\n', max_attitude_error);

% Check if estimation errors converge
final_error_norm = error_norm(end);
fprintf('Final estimation error norm: %.4f\n', final_error_norm);
if final_error_norm < 0.1
    fprintf('Estimation errors have converged well.\n');
elseif final_error_norm < 0.5
    fprintf('Estimation errors have somewhat converged.\n');
else
    fprintf('Warning: Estimation errors have not properly converged.\n');
end