%% ======================================================================
% Proactive Attack Detection Scheme for Cyber-Physical Systems
% ----------------------------------------------------------------------
% Main Theme of the paper:
% Hybrid defense strategy combining nonlinear transformation (NT) and moving
% target defense (MTD) to detect stealthy FDI attacks in CPS. Ensures no
% performance loss during attack-free operation.
% ======================================================================

%% A. System Model
% Discrete-time CPS with unknown but bounded noise:
%   x_k+1 = A * x_k + ω_k          (State dynamics)
%   y_k = C * x_k + v_k            (Sensor measurements)
%   ||ω_k||_2 ≤ δ, ||v_k||_2 ≤ ε   (Noise bounds)
%
%% B. State Estimator:
%   x_hat_k+1 = A * x_hat_k + L * (y_bar_k - C * x_hat_k)
%   where L = observer gain matrix.
% suppose initial value of the estimate, x_hat_k=0; we define the state 
% estimate error error as, e_k = x_k - x_hat_k. Combining system model and MTD
% formula, we obtain the dynamics of etimate error as:
% e_k+1=x_k+1-x_hat_k+1
%      =A * x_k + ω_k - (A * x_hat_k + L * (y_bar_k - C * x_hat_k))
%      =A * x_k + ω_k - A * x_hat_k -L(y_k+f^-1(Γ_k^-1 * ay_k)-C * x_hat_k) 
% {ay_k is the attack model y is on top of a while k is downn. 
%  ay_k=g(yM_k)=g(Γ_k*y_prime_k)} 
% e_k+1= (A-LC)e_k + ω_k - Lv_k - Lf^-1(Γ_k^-1 * ay_k)   

% We all know about that the estimator is stable if and only if
%the matrix (A−LC) is stable. In this work, the estimator can
%be stabilized by choosing appropriate estimator gain.The objectives of this paper can be formulated as follows:
% 1) How to design the proactive detection mechanism based on
% nonlinear transformation and moving target such that it can
% operate well for the system with UBB noises?
% 2) How to design the proactive detection strategy to improve
% the attack detection rate?
%% Proactive Defense Components
% ------------------------------------------
%% A. Nonlinear Transformation (NT): f(·) is placed after the sensor measures
%   y_prime_k = f(y_k)
%   - f(y_k): Bijective, monotonic function 
%   - Is invertible 
%
%% B. Moving Target Defense (MTD):
%   yM_k = Γ_k * y_prime_k   {M is on top of y, k is down}
%   - Γ_k: Time-varying diagonal matrix, e.g.,
%       Γ_k = diag([ϑ1_k 0 ... 0; 0 ϑ2_k ... 0; ...; 0 0 ...ϑny_k]);
%   - ϑi_k, i = 1,2,...,ny , are parameters to be designed..
%
% 3. Reverse Transformations (for attack-free performance):
%   y_bar_k = f_inverse(Γ_k \ yM_k)

%% C. Attack Detection Logic
% ------------------------------------------
% Residual Calculation:The residual of estimation error is defined as follows
%   r_a_k = y_bar_k - C * x_hat_k   {a on top of r, k down}
%
% Threshold Detector:
%   flag = 0 if norm(r_a_k) ≤ r_threshold(r_bar)  (No attack)
%   flag = 1 if norm(r_a_k) > r_threshold(r_bar)  (Attack detected)
%    where, r_bar=r1_bar,r2_bar,...rq_bar] is the threshold of the detector.

%% Algorithm 1: The Proactive Detection Strategy-Operations
% 1. Input: x_0, y_k, C
% 2. For k = 1 : N
% Calculate: y_prime_k, yM_k according to Nonlinear transformation and Moving target formulas
% y_bar_k can be obtained by the f(·) and MT remover 
% x(hat)_k according to state observer formula
% r_a_k = y_bar_k - C * x_hat_k
% If ∥r_a_k∥ ≤ r_threshold
% flag = 0(Attack-free)
% else
% flag = 1(Attacked)
% End If
% End For
% 3.Output: flag

%%The attack detection scheme is below:
%y_k from plant goes through f(.) and becomes y_prime_k. y_prime_k goes 
%through MTD and becomes yM_k, yM_k attacked and becomes ya_k, ya_k goes
%through MT remover(MTD inverse) and becomes y(tilda)_k, then f(.) remover
%makes it y_bar_k which goes to estimator to realize attak or not. 

%% D. System performance in absence of attack
% In the above detection scheme, resulting system cannot be interfered
% by the proposed framework when the system is attack-free.From above,
% y_bar_k=f^-1(y_tilda_k)= f^-1(Γ_k^-1(yM_k + ay_k)
%                        = f^-1(Γ_k^-1((Γ_k*y_prime_k) + ay_k))
%                        = f^-1(Γ_k^-1((Γ_k*f(y_k)) + ay_k))
% If the system is attack-free, i.e., ay_k≡ 0; it is obvious that
% y_bar_k ≡ y_k, which completes the proof.

%% E. Detection of FDI attacks: 
% The attack detection rate can be improved by proactive defense 
% consisting of moving target and non-linear transformation in the
% following two cases: (1) f(x) is a monotonic increasing function and 
% choose a larger ϑi_k ,i = 1,2,...,ny . (2) f(x) is a monotonic decreasing
% function and choose a smaller ϑi_k , i = 1,2,...,ny.
% From the residual of estimation error formula and e_k=x_k-x_hat_k, 
%  r_a_k = y_bar_k - C * x_hat_k
%        = y_k + f^-1(Γ_k^-1 * ay_k) - C * x_hat_k
%        = Ce_k + v_k + f^-1(Γ_k^-1 * ay_k)
% Then we can derive, 
%            ∥r_a_k∥ ≤ ∥C∥ ∥e_k∥ + ∥v_k∥ + ∥f^-1(Γ_k^-1 * ay_k)∥
%                   ≤ ∥C∥ ∥e_k∥ + ε + ∥f^-1(Γ_k^-1 * ay_k)∥
% To increase the estimation residues r_a_k under FDI attacks, the vaule 
% of ∥f^-1(Γ_k^-1 * ay_k)∥ should be made as large as possible. If f(x)
% is a monotonic increasing function, then f^-1 is monotonically
% decreasing.To increase ∥f^-1(Γ_k^-1 * ay_k)∥, should make Γ_k^-1 smaller,
% larger ϑi_k should be chosen. Consequently the attack detection rate 
% should be improved on account of ∥r_a_k∥ >> ∥r_r,smaller_k∥ [r,smaller on top, k down]
% where r_r,smaller_k is the value when a smaller ϑ_r_k is selcted. 




