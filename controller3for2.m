%% LQR Controller for 6 DOF Linearized UAV with Process and Measurement Noise
% This script builds on uavmodel.m to implement an LQR controller
% for the UAV model with process and measurement noise


%% LQR Controller Design Parameters
% State weighting matrix
Q_lqr = diag([100 100 100 50 50 10 10 10 10 1 1 1]); 
    
% Control weighting matrix
R_lqr = diag([0.1, 1, 1, 1]);  % Lower weight on thrust, higher on torques

%% LQR Gain Calculation
% Calculate the optimal gain matrix K
[K_lqr, ~, ~] = lqr(A, B, Q_lqr, R_lqr);

% Display the LQR gain matrix
disp('LQR Gain Matrix K:');
disp(K_lqr);

%% Simulation Parameters
t_final = 10;           % Simulation duration (seconds)
dt = 0.01;              % Time step (seconds)
t = 0:dt:t_final;       % Time vector
num_steps = length(t);

% Initial state (displaced from hover)
x0 = zeros(12, 1);
x0(1:3) = [0.5; -0.3; -1.5];  % Initial position offset from hover
x0(4:6) = [0.1; -0.1; 0.2];   % Initial attitude offset

% Reference state (hover at origin)
x_ref = zeros(12, 1);
x_ref(3) = -2;  % Hover at 2 meters above ground

% Equilibrium input 
u_eq = U_eq;  % From the UAV model

%% Run Simulation with LQR + Kalman Filter (LQG Control)
% Initialize arrays
x_true = zeros(12, num_steps);     % True states
x_est = zeros(12, num_steps);      % Estimated states
y_meas = zeros(12, num_steps);     % Noisy measurements
u_control = zeros(4, num_steps);   % Control inputs
x_true(:,1) = x0;                  % Set initial true state
x_est(:,1) = x0;                   % Initial state estimate (assuming we know it)

% Kalman filter covariance
P = cell(num_steps, 1);
P{1} = eye(12);  % Initial estimation error covariance

% Discretize system for Kalman filter
Ad = eye(12) + dt*A;
Bd = dt*B;
Qd = dt*Q;  % Discretized process noise covariance

% Simulation loop
for k = 1:num_steps-1
    % Current state estimate
    x_k = x_est(:,k);
    
    % LQR Control law using state estimate (deviation from reference)
    u_delta = -K_lqr * (x_k - x_ref);
    u = u_eq + u_delta;
    u_control(:,k) = u;
    
    % Apply control limits (optional)
    u(1) = max(0, min(u(1), 2*m*g));  % Thrust limits
    
    % Generate process noise
    w_k = randn(12, 1) .* sqrt(diag(Q));
    
    % True system dynamics with process noise
    x_true(:,k+1) = x_true(:,k) + dt * (A*x_true(:,k) + B*u + nonlinear_residual(x_true(:,k), u) + w_k);
    
    % Generate noisy measurement
    v_k = randn(12, 1) .* sqrt(diag(R));
    y_meas(:,k+1) = C*x_true(:,k+1) + v_k;
    
    % Kalman Filter
    % Predict step
    x_pred = Ad*x_est(:,k) + Bd*u;
    P_pred = Ad*P{k}*Ad' + Qd;
    
    % Update step
    K_kalman = P_pred*C'/(C*P_pred*C' + R);  % Kalman gain
    x_est(:,k+1) = x_pred + K_kalman*(y_meas(:,k+1) - C*x_pred);
    P{k+1} = (eye(12) - K_kalman*C)*P_pred;
end

%% Plotting Results
% Create a figure for visualizing the results
figure('Name', 'UAV LQR Control with Kalman Filter', 'Position', [100, 100, 1200, 800]);

% Position plot
subplot(3,3,1);
plot(t, x_true(1,:), 'b-', t, x_est(1,:), 'r--', t, x_ref(1)*ones(size(t)), 'k:');
legend('True', 'Estimated', 'Reference');
grid on; title('X Position'); xlabel('Time (s)'); ylabel('Position (m)');

subplot(3,3,2);
plot(t, x_true(2,:), 'b-', t, x_est(2,:), 'r--', t, x_ref(2)*ones(size(t)), 'k:');
grid on; title('Y Position'); xlabel('Time (s)'); ylabel('Position (m)');

subplot(3,3,3);
plot(t, x_true(3,:), 'b-', t, x_est(3,:), 'r--', t, x_ref(3)*ones(size(t)), 'k:');
grid on; title('Z Position'); xlabel('Time (s)'); ylabel('Position (m)');

% Attitude plot
subplot(3,3,4);
plot(t, x_true(4,:)*180/pi, 'b-', t, x_est(4,:)*180/pi, 'r--', t, x_ref(4)*180/pi*ones(size(t)), 'k:');
grid on; title('Roll Angle'); xlabel('Time (s)'); ylabel('Angle (deg)');

subplot(3,3,5);
plot(t, x_true(5,:)*180/pi, 'b-', t, x_est(5,:)*180/pi, 'r--', t, x_ref(5)*180/pi*ones(size(t)), 'k:');
grid on; title('Pitch Angle'); xlabel('Time (s)'); ylabel('Angle (deg)');

subplot(3,3,6);
plot(t, x_true(6,:)*180/pi, 'b-', t, x_est(6,:)*180/pi, 'r--', t, x_ref(6)*180/pi*ones(size(t)), 'k:');
grid on; title('Yaw Angle'); xlabel('Time (s)'); ylabel('Angle (deg)');

% Control inputs
subplot(3,3,7);
plot(t, u_control(1,:));
grid on; title('Thrust'); xlabel('Time (s)'); ylabel('Force (N)');

subplot(3,3,8);
plot(t, u_control(2,:), 'r-', t, u_control(3,:), 'g-', t, u_control(4,:), 'b-');
legend('\tau_\phi', '\tau_\theta', '\tau_\psi');
grid on; title('Control Torques'); xlabel('Time (s)'); ylabel('Torque (N·m)');

% 3D trajectory
subplot(3,3,9);
plot3(x_true(1,:), x_true(2,:), -x_true(3,:));
grid on; title('UAV Trajectory'); xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
axis equal;
view(45, 30);

% Create separate figure for estimation errors
figure('Name', 'State Estimation Errors', 'Position', [100, 100, 1200, 400]);

% Position estimation errors
subplot(1,3,1);
plot(t, x_true(1,:)-x_est(1,:), 'r-', t, x_true(2,:)-x_est(2,:), 'g-', t, x_true(3,:)-x_est(3,:), 'b-');
legend('X Error', 'Y Error', 'Z Error');
grid on; title('Position Estimation Errors'); xlabel('Time (s)'); ylabel('Error (m)');

% Attitude estimation errors
subplot(1,3,2);
plot(t, (x_true(4,:)-x_est(4,:))*180/pi, 'r-', t, (x_true(5,:)-x_est(5,:))*180/pi, 'g-', t, (x_true(6,:)-x_est(6,:))*180/pi, 'b-');
legend('Roll Error', 'Pitch Error', 'Yaw Error');
grid on; title('Attitude Estimation Errors'); xlabel('Time (s)'); ylabel('Error (deg)');

% Velocity estimation errors
subplot(1,3,3);
plot(t, x_true(7,:)-x_est(7,:), 'r-', t, x_true(8,:)-x_est(8,:), 'g-', t, x_true(9,:)-x_est(9,:), 'b-');
legend('Vx Error', 'Vy Error', 'Vz Error');
grid on; title('Velocity Estimation Errors'); xlabel('Time (s)'); ylabel('Error (m/s)');

%% Evaluate Controller Performance
% Calculate RMSE for position tracking
pos_rmse = sqrt(mean((x_true(1,:) - x_ref(1)).^2 + (x_true(2,:) - x_ref(2)).^2 + (x_true(3,:) - x_ref(3)).^2));
disp(['Position RMSE: ', num2str(pos_rmse), ' m']);

% Calculate RMSE for attitude tracking
att_rmse = sqrt(mean((x_true(4,:) - x_ref(4)).^2 + (x_true(5,:) - x_ref(5)).^2 + (x_true(6,:) - x_ref(6)).^2)) * 180/pi;
disp(['Attitude RMSE: ', num2str(att_rmse), ' deg']);

% Calculate control effort
control_effort = sum(sum(u_control.^2));
disp(['Control Effort: ', num2str(control_effort)]);

%% Create a function for trajectory tracking (optional extension)
function [x_ref, u_ref] = desired_trajectory(t)
    % Circular trajectory with hover orientation
    radius = 1.0;  % 1m radius
    omega = 0.5;   % 0.5 rad/s angular velocity
    
    % Reference state
    x_ref = zeros(12, 1);
    
    % Positions
    x_ref(1) = radius * cos(omega * t);       % x position
    x_ref(2) = radius * sin(omega * t);       % y position
    x_ref(3) = -2;                            % z position (2m above ground)
    
    % Velocities (derivatives of positions)
    x_ref(7) = -radius * omega * sin(omega * t);  % x velocity
    x_ref(8) = radius * omega * cos(omega * t);   % y velocity
    x_ref(9) = 0;                                 % z velocity
    
    % Yaw should follow the direction of motion
    x_ref(6) = atan2(x_ref(8), x_ref(7));       % yaw angle
    
    % Reference control input (feed-forward)
    u_ref = [1.5*9.81; 0; 0; 0];  % Hover thrust with feed-forward terms
end

%% Nonlinear Residual Function (copied from uavmodel.m)
function h_xu = nonlinear_residual(x, u)
    % Extract states
    phi = x(4); theta = x(5); psi = x(6);
    vx = x(7); vy = x(8); vz = x(9);
    p = x(10); q = x(11); r = x(12);
    
    % Pull parameters from base workspace
    m = evalin('base', 'm');
    g = evalin('base', 'g');
    Ixx = evalin('base', 'Ixx');
    Iyy = evalin('base', 'Iyy');
    Izz = evalin('base', 'Izz');
    kf = evalin('base', 'kf');
    
    % Initialize residual vector
    h_xu = zeros(12, 1);
    
    % Extract control inputs
    F = u(1);
    
    % Rotation-related nonlinearities (simplified for small angles)
    if abs(phi) > 0.1 || abs(theta) > 0.1
        % Compute full rotation matrix
        R_phi = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
        R_theta = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
        R_psi = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
        R_body_to_inertial = R_psi * R_theta * R_phi;
        
        % Gravity force in body frame (exact)
        F_gravity_body = R_body_to_inertial' * [0; 0; m*g];
        
        % Linearized gravity approximation
        F_gravity_lin = [m*g*theta; -m*g*phi; m*g];
        
        % Nonlinear residual for gravity
        gravity_residual = F_gravity_body - F_gravity_lin;
        h_xu(7:9) = gravity_residual / m;
    end
    
    % Nonlinearities in angular kinematics
    if abs(theta) > 0.1
        % Exact angle rate transformation
        W = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
             0, cos(phi), -sin(phi);
             0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
        
        % Linear approximation (at hover)
        W_lin = [1, 0, 0;
                 0, 1, 0;
                 0, 0, 1];
        
        % Angular velocity vector
        omega = [p; q; r];
        
        % Residual in angle rates
        angle_rate_residual = W*omega - W_lin*omega;
        h_xu(4:6) = angle_rate_residual;
    end
    
    % Nonlinearities from Coriolis effects
    coriolis = cross([p; q; r], [vx; vy; vz]);
    h_xu(7:9) = h_xu(7:9) - coriolis;
    
    % Nonlinearities in angular dynamics
    omega = [p; q; r];
    I = diag([Ixx, Iyy, Izz]);
    gyroscopic = cross(omega, I * omega);
    h_xu(10) = h_xu(10) - gyroscopic(1)/Ixx;
    h_xu(11) = h_xu(11) - gyroscopic(2)/Iyy;
    h_xu(12) = h_xu(12) - gyroscopic(3)/Izz;
    
    % Thrust nonlinearity (when not at hover)
    thrust_delta = F - evalin('base', 'U_eq(1)');
    if abs(thrust_delta) > 0.1 || abs(theta) > 0.1 || abs(phi) > 0.1
        % Nonlinear thrust effects in body frame
        thrust_nonlin = [sin(theta); -sin(phi)*cos(theta); -cos(phi)*cos(theta)] * thrust_delta / m;
        
        % Linear approximation
        thrust_lin = [0; 0; -thrust_delta/m];
        
        % Residual
        h_xu(7:9) = h_xu(7:9) + (thrust_nonlin - thrust_lin);
    end
end
