%% UAV LQR Controller, Observer Design and Stability Verification (REVISED)
% This script builds on the existing linearized 6 DOF UAV model
% and implements LQR control, state observer design, and stability verification
% with improved stability characteristics

% Assumes the base UAV model variables are already available in the workspace
% (A, B, C, D, system parameters, etc.)

%% Check if the base model exists in workspace
if ~exist('A', 'var') || ~exist('B', 'var') || ~exist('C', 'var')
    error('Base UAV model matrices not found in workspace. Run uavmodel.m first.');
end

%% Check System Stability
disp('Open-loop system eigenvalues:');
eig_A = eig(A);
disp(eig_A);
if any(real(eig_A) >= 0)
    warning('Open-loop system is unstable or marginally stable');
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

%% LQR Controller Design with Improved Weights
% Define LQR cost function weights with improved stability characteristics
% State weights (Q matrix) - increased weights for better stabilization
Q = diag([20 20 40 30 30 10 10 10 16 5 5 5]);  
    
% Control weights (R matrix) - decreased for more aggressive control
R = diag([0.05 0.5 0.5 1]);
   
% Design LQR controller
try
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
        warning('LQR closed-loop system is unstable! Trying pole placement instead...');
        % Try using pole placement as alternative
        desired_poles = -abs(real(eig_cl_lqr)) - 0.5;  % Move unstable poles to left half-plane
        K_lqr = place(A, B, desired_poles);
        A_cl = A - B * K_lqr;
        eig_cl_lqr = eig(A_cl);
        if all(real(eig_cl_lqr) < 0)
            disp('Controller stabilized using pole placement.');
        else
            error('Could not stabilize system with either LQR or pole placement.');
        end
    end
catch ME
    disp(['LQR design failed: ', ME.message]);
    
    % Try a different approach with manual pole placement
    disp('Attempting pole placement for controller design...');
    try
        % Desired closed-loop poles (stable, with good damping)
        desired_poles = [-0.5, -0.6, -0.7, -0.8, -0.9, -1.0, -1.1, -1.2, -1.3, -1.4, -1.5, -1.6];
        K_lqr = place(A, B, desired_poles);
        
        A_cl = A - B * K_lqr;
        eig_cl_lqr = eig(A_cl);
        
        if all(real(eig_cl_lqr) < 0)
            disp('Controller design via pole placement succeeded.');
        else
            error('Could not stabilize system with pole placement either.');
        end
    catch ME2
        error(['Both LQR and pole placement failed: ', ME2.message]);
    end
end

%% Observer (Kalman Filter) Design with Improved Robustness
% Define process and measurement noise covariance with increased values
% Process noise covariance (related to nonlinear residuals h(x,u))
Q_kalman = diag([0.02 0.02 0.02 0.04 0.04 0.04 0.1 0.1 0.1 0.2 0.2 0.]).^2;
   
% Measurement noise covariance
R_kalman = diag([0.01 0.01 0.01 0.02 0.02 0.02 0.03 0.03 0.03 0.05 0.05 0.05]).^2; 
   
try
    % Design Kalman filter
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
        warning('Observer is unstable! Trying direct pole placement...');
        
        % Try direct pole placement for observer
        desired_obs_poles = -abs(real(eig_obs)) - 0.5;  % Move unstable poles to left half-plane
        L_new = place(A', C', desired_obs_poles)';
        A_obs = A - L_new * C;
        eig_obs = eig(A_obs);
        if all(real(eig_obs) < 0)
            disp('Observer stabilized using pole placement.');
            L = L_new;
        else
            error('Could not stabilize observer with either Kalman filter or pole placement.');
        end
    end
catch ME
    disp(['Kalman filter design failed: ', ME.message]);
    
    % Try a different approach with manual pole placement for observer
    disp('Attempting pole placement for observer design...');
    try
        % Desired observer poles (stable, faster than controller poles)
        desired_obs_poles = [-1.0, -1.2, -1.4, -1.6, -1.8, -2.0, -2.2, -2.4, -2.6, -2.8, -3.0, -3.2];
        L = place(A', C', desired_obs_poles)';
        
        A_obs = A - L * C;
        eig_obs = eig(A_obs);
        
        if all(real(eig_obs) < 0)
            disp('Observer design via pole placement succeeded.');
        else
            error('Could not stabilize observer with pole placement either.');
        end
    catch ME2
        error(['Both Kalman filter and observer pole placement failed: ', ME2.message]);
    end
end

%% Combined Controller-Observer System Analysis (Separation Principle)
% Form augmented system with states [x; x_hat]
n = size(A, 1);
A_aug = [A, -B*K_lqr; 
         L*C, A-B*K_lqr-L*C];
     
% Verify eigenvalues of the augmented system
eig_aug = eig(A_aug);
disp('Eigenvalues of the combined controller-observer system:');
disp(eig_aug);

if all(real(eig_aug) < 0)
    disp('Combined controller-observer system is stable.');
else
    warning('Combined controller-observer system is unstable!');
    % At this point, if both controller and observer are individually stable
    % but the combined system is not, there might be a numerical issue
    disp('This contradicts separation principle and indicates numerical issues.');
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
if exist('h_bounds', 'var')
    h_noise_bound = h_bounds;  % Using the h_bounds from the base model
else
    h_noise_bound = zeros(12, 1);
    h_noise_bound(1:3) = 0.05;     % Position residuals (m)
    h_noise_bound(4:6) = 0.1;      % Angle residuals (rad)
    h_noise_bound(7:9) = 0.2;      % Velocity residuals (m/s)
    h_noise_bound(10:12) = 0.3;    % Angular velocity residuals (rad/s)
    warning('h_bounds not found in workspace, using default values');
end

% Run simulation
for k = 1:steps
    % Calculate control input using state estimate
    error = r - X_hat(:,k);
    
    % Ensure U_eq exists, otherwise use default
    if exist('U_eq', 'var')
        U(:,k) = U_eq - K_lqr * X_hat(:,k);  % Note: U_eq is the equilibrium control
    else
        m_default = 1.5;
        g_default = 9.81;
        U_eq_default = [m_default*g_default; 0; 0; 0];
        U(:,k) = U_eq_default - K_lqr * X_hat(:,k);
        if k == 1
            warning('U_eq not found in workspace, using default value');
        end
    end
    
    % Simulate the true system with UBB noise
    h_noise = (2*rand(12,1)-1) .* h_noise_bound;
    X(:,k+1) = A * X(:,k) + B * U(:,k) + h_noise * Ts;
    
    % Simulate measurements
    Y(:,k+1) = C * X(:,k+1) + 0.01 * randn(12, 1);  % Add small measurement noise
    
    % Update state estimate using Kalman filter
    X_hat(:,k+1) = A * X_hat(:,k) + B * U(:,k) + L * (Y(:,k+1) - C * (A * X_hat(:,k) + B * U(:,k)));
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
if exist('U_eq', 'var')
    plot(t(1:end-1), U(1,:)-U_eq(1), 'b-', t(1:end-1), U(2,:), 'r-', ...
         t(1:end-1), U(3,:), 'g-', t(1:end-1), U(4,:), 'm-', 'LineWidth', 1.5);
    ylabel('Control Input');
    title('Control Signals (Deviations from Equilibrium)');
    legend('F - F_{eq}', '\tau_\phi', '\tau_\theta', '\tau_\psi');
else
    plot(t(1:end-1), U(1,:), 'b-', t(1:end-1), U(2,:), 'r-', ...
         t(1:end-1), U(3,:), 'g-', t(1:end-1), U(4,:), 'm-', 'LineWidth', 1.5);
    ylabel('Control Input');
    title('Control Signals');
    legend('F', '\tau_\phi', '\tau_\theta', '\tau_\psi');
end
xlabel('Time (s)');
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

% For MIMO systems, we need to analyze individual channels
disp('Stability Margins Analysis (Single Channel Analysis):');

% We'll analyze key input-output channels individually
channels = [
    9, 1;   % Height control (z position response to thrust)
    4, 2;   % Roll control (phi response to roll torque)
    5, 3;   % Pitch control (theta response to pitch torque)
    6, 4    % Yaw control (psi response to yaw torque)
];

channel_names = {'Height', 'Roll', 'Pitch', 'Yaw'};

% Analyze each channel separately
for i = 1:size(channels, 1)
    output_idx = channels(i, 1);
    input_idx = channels(i, 2);
    
    % Extract SISO channel from the MIMO system
    % Form the open loop transfer function for this channel
    C_i = zeros(1, size(C, 1));
    C_i(output_idx) = 1;
    
    % Create SISO loop transfer function for this channel
    % Use the feedback form to ensure proper stability analysis
    A_fb = A_cl;
    B_fb = B(:, input_idx);
    C_fb = C_i;
    D_fb = 0;
    
    chan_sys = ss(A_fb, B_fb, C_fb, D_fb);
    
    try
        % Try to compute stability margins
        [Gm, Pm, Wcg, Wcp] = margin(chan_sys);
        
        % Check for valid results
        if isempty(Gm) || isempty(Pm) || Gm < 1 || isnan(Gm) || isnan(Pm)
            disp(['Channel ', channel_names{i}, ': Could not compute valid margins.']);
        else
            Gm_dB = 20 * log10(Gm);
            disp(['Channel ', channel_names{i}, ':']);
            disp(['  Gain Margin: ', num2str(Gm_dB), ' dB at frequency ', num2str(Wcg), ' rad/s']);
            disp(['  Phase Margin: ', num2str(Pm), ' degrees at frequency ', num2str(Wcp), ' rad/s']);
            
            % Plot Nyquist diagram instead of Bode for better stability assessment
            figure('Name', ['Nyquist Plot - ', channel_names{i}, ' Channel']);
            nyquist(chan_sys);
            title(['Nyquist Plot - ', channel_names{i}, ' Control Channel']);
            grid on;
        end
    catch ME
        disp(['Could not compute margins for ', channel_names{i}, ' channel: ', ME.message]);
        
        % Try alternative approach using frequency response directly
        try
            w = logspace(-2, 2, 100);
            [mag, phase, w] = bode(chan_sys, w);
            
            figure('Name', ['Frequency Response - ', channel_names{i}, ' Channel']);
            subplot(2,1,1);
            semilogx(w, 20*log10(squeeze(mag)), 'b-', 'LineWidth', 1.5);
            grid on;
            xlabel('Frequency (rad/s)');
            ylabel('Magnitude (dB)');
            title(['Magnitude Response - ', channel_names{i}, ' Channel']);
            
            subplot(2,1,2);
            semilogx(w, squeeze(phase), 'r-', 'LineWidth', 1.5);
            grid on;
            xlabel('Frequency (rad/s)');
            ylabel('Phase (deg)');
            title(['Phase Response - ', channel_names{i}, ' Channel']);
        catch
            disp(['Could not generate frequency response for ', channel_names{i}, ' channel.']);
        end
    end
end