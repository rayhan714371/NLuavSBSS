%% LQR Controller Design for 6-DOF Linearized UAV Model
% This script checks controllability and observability of the linearized UAV model
% and designs an LQR controller to stabilize the system

% Run the UAV model script first to ensure we have all parameters
% (assuming uavmodel2.m is in the same directory)
run('uavmodel2.m');

%% Check Controllability and Observability
disp('Checking Controllability and Observability:');

% Check controllability
Cm = ctrb(A, B);
n = length(A);
rank_Cm = rank(Cm);

if rank_Cm == n
    disp(['The system is controllable. Rank of controllability matrix = ', num2str(rank_Cm)]);
else
    disp(['The system is NOT fully controllable. Rank = ', num2str(rank_Cm), ' < ', num2str(n)]);
    
    % Find uncontrollable modes
    [V, D] = eig(A);
    for i = 1:n
        mode = V(:, i);
        if rank([Cm mode]) == rank(Cm)
            disp(['Uncontrollable mode at eigenvalue λ = ', num2str(D(i,i))]);
        end
    end
end

% Check observability
Om = obsv(A, C);
rank_Om = rank(Om);

if rank_Om == n
    disp(['The system is observable. Rank of observability matrix = ', num2str(rank_Om)]);
else
    disp(['The system is NOT fully observable. Rank = ', num2str(rank_Om), ' < ', num2str(n)]);
    
    % Find unobservable modes
    [V, D] = eig(A');
    for i = 1:n
        mode = V(:, i);
        if rank([Om'; mode]) == rank(Om')
            disp(['Unobservable mode at eigenvalue λ = ', num2str(D(i,i))]);
        end
    end
end

%% Design LQR Controller
% Define LQR cost function weights
% State weights (prioritize position and attitude)
Q = diag([100 100 100 ... % Position states (x,y,z)
          10 10 10 ...    % Attitude states (phi,theta,psi)
          1 1 1 ...       % Linear velocity states (vx,vy,vz)
          1 1 1]);        % Angular velocity states (p,q,r)

% Control weights
R = diag([0.1 1 1 1]);    % Thrust, Roll, Pitch, Yaw torques

% Solve the Riccati equation to get optimal gain matrix K
[K, S, e] = lqr(A, B, Q, R);

disp('LQR Controller Design:');
disp('Optimal gain matrix K:');
disp(K);
disp('Closed-loop eigenvalues:');
disp(e);

%% Simulation Parameters
% Initial state (deviated from hover)
x0 = zeros(12, 1);
x0(1:3) = [0.5; 0.5; -2.2];  % Initial position slightly off from hover point
x0(4:6) = [0.1; -0.1; 0.2];  % Initial attitude disturbance

% Simulation time
tspan = 0:0.01:5;  % 5 seconds with 0.01s step size

%% Simulate Closed-Loop System Response
% Preallocate arrays
x = zeros(12, length(tspan));
u = zeros(4, length(tspan));
x(:,1) = x0;

% Reference state (hover at 2m)
x_ref = X_eq;

% Closed-loop simulation
for k = 1:length(tspan)-1
    % State error
    e = x(:,k) - x_ref;
    
    % LQR control law: u = -K*e + u_eq
    u(:,k) = -K*e + U_eq;
    
    % Simple forward Euler integration of the linear model
    % (Ignoring nonlinear residuals and noise for this test)
    x(:,k+1) = x(:,k) + 0.01 * (A*x(:,k) + B*u(:,k));
end

%% Plot Results
figure(1);
% Position plots
subplot(3,1,1);
plot(tspan, x(1,:), 'r', tspan, x(2,:), 'g', tspan, x(3,:), 'b');
legend('x', 'y', 'z');
title('Position');
grid on;

% Attitude plots
subplot(3,1,2);
plot(tspan, x(4,:), 'r', tspan, x(5,:), 'g', tspan, x(6,:), 'b');
legend('\phi', '\theta', '\psi');
title('Attitude');
grid on;

% Control inputs plots
subplot(3,1,3);
plot(tspan, u(1,:)-U_eq(1), 'r', tspan, u(2,:), 'g', tspan, u(3,:), 'b', tspan, u(4,:), 'm');
legend('Thrust', '\tau_\phi', '\tau_\theta', '\tau_\psi');
title('Control Inputs (deviation from hover)');
grid on;

%% Display System Stability Assessment
disp('System Stability Assessment:');
disp('Open-loop eigenvalues:');
disp(eig(A));
disp('Closed-loop eigenvalues:');
disp(eig(A - B*K));