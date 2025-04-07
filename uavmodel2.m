%% 6 DOF Linearized UAV
% This script provides a linearized UAV model in the form:
% dot_x(t) = Ax(t) + Bu(t) + h(x,u) + w(t)
% y(t) = Cx(t) + v(t)
% where h(x,u) represents the nonlinear residuals after linearization
% w(t) represents process noise
% and v(t) represents measurement noise

%% UAV Physical Parameters
m = 1.5;              % Mass (kg)
g = 9.81;             % Gravity (m/s^2)
L = 0.23;             % Arm length (m)

% Moments of inertia (kg.m^2)
Ixx = 0.0211;
Iyy = 0.0219;
Izz = 0.0366;

% Rotor and aerodynamic parameters
kT = 1.091e-5;        % Thrust coefficient
kD = 1.779e-7;        % Drag coefficient
kf = 0.01;            % Friction coefficient

%% State and Control Vectors Definition
% State vector x = [px; py; pz; phi; theta; psi; vx; vy; vz; p; q; r]
% where:
% (px, py, pz) - position in inertial frame
% (phi, theta, psi) - Euler angles (roll, pitch, yaw)
% (vx, vy, vz) - linear velocities in body frame
% (p, q, r) - angular velocities in body frame

% Control input vector u = [F; tau_phi; tau_theta; tau_psi]
% where:
% F - total thrust
% tau_phi - roll torque
% tau_theta - pitch torque
% tau_psi - yaw torque

%% Linearization Point (Hover)
% Define equilibrium state (hover condition)
X_eq = zeros(12, 1);
X_eq(3) = -2;  % Hover at 2 meters above ground

% Define equilibrium input (hover thrust balances gravity)
U_eq = [m*g; 0; 0; 0];

%% Linearized State-Space Matrices
% A matrix (state matrix)
A = zeros(12, 12);

% Position derivatives
A(1, 7) = 1;  % dx/dt = vx
A(2, 8) = 1;  % dy/dt = vy
A(3, 9) = 1;  % dz/dt = vz

% Attitude kinematics at hover
A(4, 10) = 1;  % dphi/dt = p
A(5, 11) = 1;  % dtheta/dt = q
A(6, 12) = 1;  % dpsi/dt = r

% Velocity dynamics (linearized around hover)
A(7, 5) = g;    % dvx/dt affected by pitch angle (gravity projection)
A(8, 4) = -g;   % dvy/dt affected by roll angle (gravity projection)
A(7, 7) = -kf/m;  % Damping in x direction
A(8, 8) = -kf/m;  % Damping in y direction
A(9, 9) = -kf/m;  % Damping in z direction

% Angular velocity dynamics
A(10, 10) = -0.01;  % Aerodynamic damping in roll
A(11, 11) = -0.01;  % Aerodynamic damping in pitch
A(12, 12) = -0.02;  % Aerodynamic damping in yaw

% B matrix (input matrix)
B = zeros(12, 4);

% Thrust affects vertical acceleration (in body frame)
B(9, 1) = -1/m;  % Negative because Z-axis points downward in body frame

% Torques affect angular accelerations
B(10, 2) = 1/Ixx;  % Roll torque
B(11, 3) = 1/Iyy;  % Pitch torque
B(12, 4) = 1/Izz;  % Yaw torque

% C matrix (output matrix)
% Assuming all states are measurable
C = eye(12);

% D matrix (direct feedthrough)
D = zeros(12, 4);

%% Noise Characteristics
% Process noise covariance matrix Q
% Process noise affects the state dynamics directly
% Generally higher for velocity and angular states, lower for position states
pos_proc_std = 0.01;      % Position process noise (m/s^2)
angle_proc_std = 0.02;    % Angle process noise (rad/s^2)
vel_proc_std = 0.05;      % Velocity process noise (m/s^2)
angvel_proc_std = 0.1;    % Angular velocity process noise (rad/s^2)

% Create diagonal process noise covariance matrix
Q = diag([0.0001 0.0001 0.0001 0.0004 0.0004 0.0004 0.0025 0.0025 0.0025 0.01 0.01 0.01]);  

% Measurement noise covariance matrix R
pos_meas_std = 0.05;      % Position measurement noise (m)
angle_meas_std = 0.02;    % Angle measurement noise (rad)
vel_meas_std = 0.1;       % Velocity measurement noise (m/s)
angvel_meas_std = 0.05;   % Angular velocity measurement noise (rad/s)

% Create diagonal measurement noise covariance matrix
R = diag([0.0025 0.0025 0.0025 0.0004 0.0004 0.0004 0.01 0.01 0.01 0.0025 0.0025 0.0025]);

% Create input process noise covariance (optional)
input_noise_std = 0.1;    % Input noise standard deviation
Qu = input_noise_std^2 * eye(4);  % Input noise covariance

%% Create Linearized State-Space Model
sys_lin = ss(A, B, C, D);

% Display linearized model properties
disp('Linearized UAV Model:');
disp('A matrix (State Matrix):');
disp(A);
disp('B matrix (Input Matrix):');
disp(B);
disp('C matrix (Output Matrix):');
disp(C);
disp('Process Noise Covariance Matrix Q:');
disp(Q);
disp('Measurement Noise Covariance Matrix R:');
disp(R);

%% UBB Noise Parameters
% Define bounds for the nonlinear residual h(x,u)
h_bounds = zeros(12, 1);
h_bounds(1:3) = 0.05;     % Position residuals (m)
h_bounds(4:6) = 0.1;      % Angle residuals (rad)
h_bounds(7:9) = 0.2;      % Velocity residuals (m/s)
h_bounds(10:12) = 0.3;    % Angular velocity residuals (rad/s)

%% Nonlinear Residual Function h(x,u)
% This function computes the nonlinear residuals after linearization
% h(x,u) = f_nonlinear(x,u) - (Ax + Bu)
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

%% Function to generate measurement with noise
function y = measurement_with_noise(x)
    % Get the output matrix C from base workspace
    C = eye(12);  % Assuming all states are measured
    
    % Get noise covariance matrix from base workspace
    R = evalin('base', 'R');
    
    % Generate Gaussian noise for measurements
    v = randn(12, 1) .* sqrt(diag(R));
    
    % Calculate noisy measurements
    y = C*x + v;
end

%% Function to generate process noise
function w = process_noise()
    % Get process noise covariance from base workspace
    Q = evalin('base', 'Q');
    
    % Generate Gaussian process noise
    w = randn(12, 1) .* sqrt(diag(Q));
end

%% Sample simulation with measurement and process noise
function [t, x_true, y_noisy] = simulate_uav_with_noise(tspan, x0, u)
    % Get model matrices
    A = evalin('base', 'A');
    B = evalin('base', 'B');
    
    % Time step for simulation
    dt = tspan(2) - tspan(1);
    num_steps = length(tspan);
    
    % Preallocate arrays
    x_true = zeros(12, num_steps);
    y_noisy = zeros(12, num_steps);
    x_true(:,1) = x0;
    
    % Generate noisy measurements for initial state
    y_noisy(:,1) = measurement_with_noise(x0);
    
    % Simulate system with Euler integration
    for k = 1:num_steps-1
        % Generate process noise for this time step
        w = process_noise();
        
        % Update state with process noise
        x_true(:,k+1) = x_true(:,k) + dt * (A*x_true(:,k) + B*u + nonlinear_residual(x_true(:,k), u) + w);
        
        % Generate noisy measurement
        y_noisy(:,k+1) = measurement_with_noise(x_true(:,k+1));
    end
    
    t = tspan;
end

%% Kalman Filter Implementation
function [x_est, P] = kalman_filter(y_noisy, u, tspan)
    % Get model matrices
    A = evalin('base', 'A');
    B = evalin('base', 'B');
    C = evalin('base', 'C');
    Q = evalin('base', 'Q');
    R = evalin('base', 'R');
    
    % Time step for simulation
    dt = tspan(2) - tspan(1);
    num_steps = length(tspan);
    
    % Initial state estimate and covariance
    x_est = zeros(12, num_steps);
    x_est(:,1) = zeros(12, 1);  % Initial state estimate (zero or user-defined)
    P = cell(num_steps, 1);
    P{1} = eye(12);  % Initial covariance estimate (identity or user-defined)
    
    % Discrete-time system matrices (Euler approximation)
    Ad = eye(12) + dt*A;
    Bd = dt*B;
    
    % Process noise covariance (discretized)
    Qd = dt*Q;
    
    % Kalman filter iterations
    for k = 1:num_steps-1
        % Predict step
        x_pred = Ad*x_est(:,k) + Bd*u;
        P_pred = Ad*P{k}*Ad' + Qd;
        
        % Update step (measurement)
        K = P_pred*C'/(C*P_pred*C' + R);  % Kalman gain
        x_est(:,k+1) = x_pred + K*(y_noisy(:,k+1) - C*x_pred);
        P{k+1} = (eye(12) - K*C)*P_pred;
    end
end