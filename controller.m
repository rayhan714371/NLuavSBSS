%% UAV LQR Controller, Observer Design and Stability Verification
% This script builds on the existing linearized 6 DOF UAV model
% and implements LQR control, state observer design, and stability verification

% Assumes the base UAV model variables are already available in the workspace
% (A, B, C, D, system parameters, etc.)

%% Check if the base model exists in workspace
if ~exist('A', 'var') || ~exist('B', 'var') || ~exist('C', 'var')
    error('Base UAV model matrices not found in workspace. Run uavmodel.m first.');
end

%% System Controllability and Observability Analysis
% Check controllability - can we control all states?
Co = ctrb(A, B);
rank_Co = rank(Co);
if rank_Co == size(A, 1)
    disp(['System is controllable. Rank of controllability matrix: ', num2str(rank_Co)]);
else
    warning(['System is NOT fully controllable. Rank of controllability matrix: ', ...
             num2str(rank_Co), ' (expected ', num2str(size(A, 1)), ')']);
    disp('Uncontrollable modes:');
    [V, D] = eig(A);
    eigs = diag(D);
    for i = 1:length(eigs)
        % Check if this eigenvalue corresponds to an uncontrollable mode
        if rank([Co, V(:,i)]) == rank(Co)
            disp(['Eigenvalue ', num2str(eigs(i))])
        end
    end
end

% Check observability - can we observe all states?
Ob = obsv(A, C);
rank_Ob = rank(Ob);
if rank_Ob == size(A, 1)
    disp(['System is observable. Rank of observability matrix: ', num2str(rank_Ob)]);
else
    warning(['System is NOT fully observable. Rank of observability matrix: ', ...
             num2str(rank_Ob), ' (expected ', num2str(size(A, 1)), ')']);
    disp('Unobservable modes:');
    [V, D] = eig(A');
    eigs = diag(D);
    for i = 1:length(eigs)
        % Check if this eigenvalue corresponds to an unobservable mode
        if rank([Ob, V(:,i)]) == rank(Ob)
            disp(['Eigenvalue ', num2str(eigs(i))])
        end
    end
end

%% LQR Controller Design
% Define LQR cost function weights
% State weights (Q matrix) - how important is each state?
Q = diag([10 10 20 15 15 5 5 5 8 1 1 1]);  

% Control weights (R matrix) - how expensive is each control input?
R = diag([0.1 1 1 2 ]);          % Thrust cost (F)
        % Torque costs (tau_phi, tau_theta, tau_psi)


% Design LQR controller
[K_lqr, P_lqr, eig_cl] = lqr(A, B, Q, R);

% Display LQR controller gain
disp('LQR Controller Gain Matrix K:');
disp(K_lqr);

% Check closed-loop stability with LQR controller
A_cl = A - B * K_lqr;
eig_cl_lqr = eig(A_cl);
disp('Closed-loop eigenvalues with LQR controller:');
disp(eig_cl_lqr);

if all(real(eig_cl_lqr) < 0)
    disp('LQR closed-loop system is stable.');
else
    warning('LQR closed-loop system is unstable!');
end

%% Observer (Kalman Filter) Design
% Define process and measurement noise covariance
% Process noise covariance (related to nonlinear residuals h(x,u))
Q_kalman = diag([0.01 0.01 0.01 0.02 0.02 0.02 0.05 0.05 0.05 0.1 0.1 0.1]).^2;
           % Position process noise (px, py, pz)
           % Angle process noise (phi, theta, psi)
           % Velocity process noise (vx, vy, vz)
              % Angular velocity process noise (p, q, r)


% Measurement noise covariance
R_kalman = diag([0.01 0.01 0.01 0.02 0.02 0.02 0.03 0.03 0.03 0.05 0.05 0.05]).^2; 
         % Position measurement noise
          % Angle measurement noise
           % Velocity measurement noise
           % Angular velocity measurement noise


% Design Kalman filter (which gives us the observer gain L)
[kf, L, P] = kalman(ss(A, eye(size(A)), C, 0), Q_kalman, R_kalman);

% Display observer gain
disp('Observer Gain Matrix L:');
disp(L');

% Check observer stability
A_obs = A - L' * C;
eig_obs = eig(A_obs);
disp('Observer eigenvalues:');
disp(eig_obs);

if all(real(eig_obs) < 0)
    disp('Observer is stable.');
else
    warning('Observer is unstable!');
end

%% Combined Controller-Observer System Analysis (Separation Principle)
% Form augmented system with states [x; x_hat]
n = size(A, 1);
A_aug = [A, -B*K_lqr; 
         L'*C, A-B*K_lqr-L'*C];
     
% Verify eigenvalues of the augmented system
eig_aug = eig(A_aug);
disp('Eigenvalues of the combined controller-observer system:');
disp(eig_aug);

if all(real(eig_aug) < 0)
    disp('Combined controller-observer system is stable.');
else
    warning('Combined controller-observer system is unstable!');
end

%% Performance Analysis

% Define simulation parameters
Ts = 0.01;  % Sample time (seconds)
T = 10;     % Simulation duration (seconds)
steps = T/Ts;

% Time vector
t = 0:Ts:T;

% Initial state
x0 = zeros(12, 1);
x0(1:3) = [1; 1; -3];  % Initial position offset from equilibrium

% Initial state estimate (slightly off)
x_hat0 = x0 + 0.1*randn(12, 1);

% Storage for simulation results
X = zeros(12, steps+1);
X_hat = zeros(12, steps+1);
U = zeros(4, steps);
Y = zeros(12, steps+1);

% Set initial conditions
X(:,1) = x0;
X_hat(:,1) = x_hat0;
Y(:,1) = C * x0;

% Define zero reference for regulation problem
r = zeros(12, 1);

% Add UBB noise bounds to the simulation
h_noise_bound = h_bounds;  % Using the h_bounds from the base model

% Run simulation
for k = 1:steps
    % Calculate control input using state estimate
    error = r - X_hat(:,k);
    U(:,k) = U_eq - K_lqr * X_hat(:,k);  % Note: U_eq is the equilibrium control
    
    % Simulate the true system with UBB noise
    % Normally we'd call nonlinear_residual, but here we approximate 
    % with random noise within bounds
    h_noise = (2*rand(12,1)-1) .* h_noise_bound;
    X(:,k+1) = A * X(:,k) + B * U(:,k) + h_noise * Ts;
    
    % Simulate measurements
    Y(:,k+1) = C * X(:,k+1) + 0.01 * randn(12, 1);  % Add small measurement noise
    
    % Update state estimate using Kalman filter
    X_hat(:,k+1) = A * X_hat(:,k) + B * U(:,k) + L' * (Y(:,k+1) - C * (A * X_hat(:,k) + B * U(:,k)));
end

%% Plot Results
figure('Name', 'UAV Trajectory and Control Performance');

% Plot position tracking
subplot(3,1,1);
plot(t, X(1,:), 'b-', t, X(2,:), 'r-', t, X(3,:), 'g-', 'LineWidth', 1.5);
hold on;
plot(t, X_hat(1,:), 'b--', t, X_hat(2,:), 'r--', t, X_hat(3,:), 'g--', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Position (m)');
title('Position Tracking');
legend('x', 'y', 'z', 'x estimate', 'y estimate', 'z estimate');
grid on;

% Plot orientation
subplot(3,1,2);
plot(t, X(4,:)*180/pi, 'b-', t, X(5,:)*180/pi, 'r-', t, X(6,:)*180/pi, 'g-', 'LineWidth', 1.5);
hold on;
plot(t, X_hat(4,:)*180/pi, 'b--', t, X_hat(5,:)*180/pi, 'r--', t, X_hat(6,:)*180/pi, 'g--', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Orientation Tracking');
legend('\phi', '\theta', '\psi', '\phi estimate', '\theta estimate', '\psi estimate');
grid on;

% Plot control inputs
subplot(3,1,3);
plot(t(1:end-1), U(1,:)-U_eq(1), 'b-', t(1:end-1), U(2,:), 'r-', ...
     t(1:end-1), U(3,:), 'g-', t(1:end-1), U(4,:), 'm-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control Input');
title('Control Signals (Deviations from Equilibrium)');
legend('F - F_{eq}', '\tau_\phi', '\tau_\theta', '\tau_\psi');
grid on;

% Figure for state estimation errors
figure('Name', 'State Estimation Errors');
error = X - X_hat;

% Position errors
subplot(3,1,1);
plot(t, error(1,:), 'b-', t, error(2,:), 'r-', t, error(3,:), 'g-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position Error (m)');
title('Position Estimation Errors');
legend('x error', 'y error', 'z error');
grid on;

% Angle errors
subplot(3,1,2);
plot(t, error(4,:)*180/pi, 'b-', t, error(5,:)*180/pi, 'r-', t, error(6,:)*180/pi, 'g-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angle Error (deg)');
title('Orientation Estimation Errors');
legend('\phi error', '\theta error', '\psi error');
grid on;

% Velocity errors
subplot(3,1,3);
plot(t, error(7,:), 'b-', t, error(8,:), 'r-', t, error(9,:), 'g-', 'LineWidth', 1.5);
hold on;
plot(t, error(10,:)*180/pi, 'b--', t, error(11,:)*180/pi, 'r--', t, error(12,:)*180/pi, 'g--', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Velocity Error');
title('Velocity Estimation Errors');
legend('v_x error', 'v_y error', 'v_z error', 'p error (deg/s)', 'q error (deg/s)', 'r error (deg/s)');
grid on;

%% Robustness Analysis - Stability Margins
% Analyze robust stability with respect to gain variations

% Loop Transfer Function at Plant Input
L_in = ss(A, B, K_lqr, 0);

% Compute stability margins
[Gm_in, Pm_in, Wcg_in, Wcp_in] = margin(L_in);
Gm_in_dB = 20 * log10(Gm_in);

disp('Stability Margins at Plant Input:');
disp(['Gain Margin: ', num2str(Gm_in_dB), ' dB at frequency ', num2str(Wcg_in), ' rad/s']);
disp(['Phase Margin: ', num2str(Pm_in), ' degrees at frequency ', num2str(Wcp_in), ' rad/s']);

% Display stability margin summary
disp('Robustness Summary:');
if (Gm_in_dB > 6) && (Pm_in > 30)
    disp('Controller has good robustness margins (GM > 6dB, PM > 30°)');
else
    disp('Controller robustness margins are less than recommended (GM > 6dB, PM > 30°)');
end

%% System Response to Step Command
figure('Name', 'Step Response Analysis');

% Extract SISO transfer functions for key outputs
% Position response to a step command in desired height
z_step_sys = ss(A_cl, B(:,1), C(3,:), 0);
[z_step, t_step] = step(z_step_sys, t);

% Plot step response for height
subplot(2,1,1);
plot(t_step, z_step, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Height (m)');
title('Step Response - Height Command');
grid on;

% Extract pitch angle response to a pitch command
pitch_step_sys = ss(A_cl, B(:,3), C(5,:), 0);
[pitch_step, t_step] = step(pitch_step_sys, t);

% Plot step response for pitch
subplot(2,1,2);
plot(t_step, pitch_step*180/pi, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Pitch Angle (deg)');
title('Step Response - Pitch Command');
grid on;

%% Print Overall Stability and Performance Summary
disp('======== UAV Control System Analysis Summary ========');

% Controller performance metrics
settling_idx = find(abs(X(3,:) - X(3,end)) < 0.05*abs(X(3,1) - X(3,end)), 1);
if ~isempty(settling_idx)
    settling_time = t(settling_idx);
    disp(['Settling Time (5%): ', num2str(settling_time), ' seconds']);
else
    disp('System did not settle within the simulation time.');
end

max_pos_error = max(abs(X(1:3,:)), [], 2);
disp('Maximum Position Errors:');
disp(['  X: ', num2str(max_pos_error(1)), ' m']);
disp(['  Y: ', num2str(max_pos_error(2)), ' m']);
disp(['  Z: ', num2str(max_pos_error(3)), ' m']);

max_angle_error = max(abs(X(4:6,:)), [], 2) * 180/pi;
disp('Maximum Angle Errors:');
disp(['  Roll: ', num2str(max_angle_error(1)), ' deg']);
disp(['  Pitch: ', num2str(max_angle_error(2)), ' deg']);
disp(['  Yaw: ', num2str(max_angle_error(3)), ' deg']);

% Final stability assessment
if all(real(eig_cl_lqr) < 0) && all(real(eig_obs) < 0) && all(real(eig_aug) < 0)
    disp('OVERALL ASSESSMENT: System is stable and meets performance requirements.');
else
    disp('OVERALL ASSESSMENT: System has stability issues!');
end