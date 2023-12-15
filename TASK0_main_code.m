%% ADVANCED CONTROL AND INFORMATICS HOMEWORK
% Barnabás Piri, HH7AOB
clear
close
clc

%% Numerical data
M = 7.87; % mass of the cart [kg]
m = 1.55; % mass of the pendulum [kg]
l = 3.11; % length of the pendulum [m]
b = 0.2; % viscous damping acting on the cart [N*s/m] 
g = 9.81; % gravitational acceleration [m/s^2]
J_s = 1/3*m*l^2; % inertia of the pendulum around its endpoint [kg*m^2]

%% Nonlinear model in Simulink

close all

sim("TASK1_nonlinear_model.slx")

figure(1)
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 16, 12]);
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [16 12]);
hold on
xlabel('Time [s]', fontsize=12)
ylabel('x [m]', fontsize=12)
title('Nonlinear system step response', fontsize=12)
grid('on')
plot(phi_org.Time, phi_org.Data, LineWidth=1.5)
plot(x_org.Time, x_org.Data, LineWidth=1.5)
legend('\phi', 'x', Location='northwest', fontsize=12)
saveas(gcf, 'images/task1_nonlinear_model.pdf', 'pdf');

%% State space model from the analytically linearized equations of motion
disp('----------- TASK 1. State space model from analytical linearization -----------')

% The matrices from the derived equations of motion
M_m = [J_s+m*l^2, m*l; m*l, M+m]; % mass matrix (coeff. matrix of [ddphi, ddx])
C_m = [0, 0; 0, b]; % damping matrix (coeff. matrix of [dphi, dx])
K_m = [-m*g*l, 0; 0, 0]; % stiffness matrix (coeff. matrix of [phi, x])
M_inv = inv(M); % inverse of the mass matrix
F = [0; 1]; % excitation force vector

% Rewrite the 2 second order DE. system to 4 first order DE. system to obtain state space model matrices
[A_analytic, B_analytic] = TASK0_ss_AB(M_m, C_m, K_m, F)

%% Linearizing the nonlinear model in simulink
disp('----------- TASK 2. Linearizing the nonlinear model in simulink -----------')

model = 'TASK2_linearization'; % name of the nonlinear model
io = getlinio(model); % finding linearization annotations set in Simulink
op = operpoint(model); % create the operating point to linearize around
% the default point for the states are [0; 0; 0; 0] which is good for the operating point
sys = linearize(model, io, op); % linearization of the model around op.
sys = xperm(sys, [3 1 4 2]);

% linearized state space matrices
A_lin = sys.A
B_lin = sys.B
C_lin = sys.C;
D_lin = sys.D;

%% Finding the poles, damping and time constants of the linearized system
disp('----------- TASK 3. Poles, damping and time constants of the linearized system -----------')

[omega_n, damping_factors, poles] = damp(sys)
time_constants = 1./(omega_n.*damping_factors)
figure(2);
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 10, 10]);
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [10 10]);
pzmap(sys)
title('Pole-zero map for the linearized model', fontsize=12);
saveas(gcf, 'images/task3_pzmap.jpg', 'jpg');

%% Showing that the system is observable and controllable
disp('----------- TASK 4. Showing that the system is observable and controllable -----------')

ctrbMatrix = ctrb(sys.A, sys.B); % controllability matrix
rankCtrb = rank(ctrbMatrix); % rank of the controllabililty matrix
isControllable = (rankCtrb == size(sys.A, 1)); % controllability condition

obsvMatrix = obsv(sys.A, sys.C); % observability matrix 
rankObsv = rank(obsvMatrix); % rank of the observability matrix  
isObservable = (rankObsv == size(sys.A, 1)); % observability condition

disp(' ')
disp(['The system is controllable: ', num2str(isControllable)]);
disp(['The system is observable: ', num2str(isObservable)]);
disp(' ')

%% State feedback controller
disp('----------- TASK 5. State feedback controller -----------')

close all

p = [-1; -1.1; -1.2; -1.3]; % initial poles
K = place(A_lin, B_lin, p); % feedback matrix

% initial conditions
x0 = 1.1;
dx0 = 0.4;
phi0 = 0.1;
dphi0 = 0.3;

% pole increasing
p1 = p + 0.5;
K1 = place(A_lin, B_lin, p1);
p2 = p + 0.75;
K2 = place(A_lin, B_lin, p2);
p3 = p + 1.0;
K3 = place(A_lin, B_lin, p3);

% pole decreasing
p4 = p - 0.5;
K4 = place(A_lin, B_lin, p4);
p5 = p - 1.5;
K5 = place(A_lin, B_lin, p5);
p6 = p - 2.25;
K6 = place(A_lin, B_lin, p6);

% simulation with initial poles
sim("TASK5_initial_poles.slx")

% simulation with increasing poles
sim("TASK5_increasing_poles.slx")

% simulation with decreasing poles
sim("TASK5_decreasing_poles.slx")

% plot the result of increasing poles
figure(3)
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 16, 12]);
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [16 12]);
hold on
xlabel('Time [s]', fontsize=12)
ylabel('x [m]', fontsize=12)
title('Pole increasing', fontsize=12)
grid('on')
plot(x_init.Time, x_init.Data, LineWidth=1.5)
plot(x_1.Time, x_1.Data, LineWidth=1.5)
plot(x_2.Time, x_2.Data, LineWidth=1.5)
plot(x_3.Time, x_3.Data, LineWidth=1.5)
legend('p_{initial}', 'p_1', 'p_2', 'p_3', Location='northwest', fontsize=12)
saveas(gcf, 'images/task5_increasing_x.pdf', 'pdf');

figure(4)
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 16, 12]);
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [16 12]);
hold on
xlabel('Time [s]', fontsize=12)
ylabel('\phi [rad]', fontsize=12)
title('Pole increasing', fontsize=12)
grid('on')
plot(phi_init.Time, phi_init.Data, LineWidth=1.5)
plot(phi_1.Time, phi_1.Data, LineWidth=1.5)
plot(phi_2.Time, phi_2.Data, LineWidth=1.5)
plot(phi_3.Time, phi_3.Data, LineWidth=1.5)
legend('p_{initial}', 'p_1', 'p_2', 'p_3', Location='northeast', fontsize=12)
saveas(gcf, 'images/task5_increasing_phi.pdf', 'pdf');

figure(5)
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 16, 12]);
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [16 12]);
hold on
xlabel('Time [s]', fontsize=12)
ylabel('x [m]', fontsize=12)
title('Pole increasing, effect on control signal', fontsize=12)
grid('on')
plot(u_init.Time, u_init.Data, LineWidth=1.5)
plot(u_1.Time, u_1.Data, LineWidth=1.5)
plot(u_2.Time, u_2.Data, LineWidth=1.5)
plot(u_3.Time, u_3.Data, LineWidth=1.5)
legend('p_{initial}', 'p_1', 'p_2', 'p_3', Location='northwest', fontsize=12)
saveas(gcf, 'images/task5_increasing_u.pdf', 'pdf');

% plot the result of decreasing poles
figure(6)
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 16, 12]);
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [16 12]);
hold on
xlabel('Time [s]', fontsize=12)
ylabel('x [m]', fontsize=12)
title('Pole decreasing', fontsize=12)
grid('on')
plot(x_init.Time, x_init.Data, LineWidth=1.5)
plot(x_4.Time, x_4.Data, LineWidth=1.5)
plot(x_5.Time, x_5.Data, LineWidth=1.5)
plot(x_6.Time, x_6.Data, LineWidth=1.5)
legend('p_{initial}', 'p_4', 'p_5', 'p_6', Location='northeast', fontsize=12)
saveas(gcf, 'images/task5_decreasing_x.pdf', 'pdf');

figure(7)
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 16, 12]);
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [16 12]);
hold on
xlabel('Time [s]', fontsize=12)
ylabel('\phi [rad]', fontsize=12)
title('Pole decreasing', fontsize=12)
grid('on')
plot(phi_init.Time, phi_init.Data, LineWidth=1.5)
plot(phi_4.Time, phi_4.Data, LineWidth=1.5)
plot(phi_5.Time, phi_5.Data, LineWidth=1.5)
plot(phi_6.Time, phi_6.Data, LineWidth=1.5)
legend('p_{initial}', 'p_4', 'p_5', 'p_6', Location='northeast', fontsize=12)
saveas(gcf, 'images/task5_decreasing_phi.pdf', 'pdf');

figure(8)
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 16, 12]);
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [16 12]);
hold on
xlabel('Time [s]', fontsize=12)
ylabel('x [m]', fontsize=12)
title('Pole decreasing, effect on control signal', fontsize=12)
grid('on')
plot(u_init.Time, u_init.Data, LineWidth=1.5)
plot(u_4.Time, u_4.Data, LineWidth=1.5)
plot(u_5.Time, u_5.Data, LineWidth=1.5)
plot(u_6.Time, u_6.Data, LineWidth=1.5)
legend('p_{initial}', 'p_4', 'p_5', 'p_6', Location='northwest', fontsize=12)
saveas(gcf, 'images/task5_decreasing_u.pdf', 'pdf');

%% LQR state feedback controller
disp('----------- TASK 6. LQR state feedback controller -----------')

close all

Q = [100, 0 , 0  ,  0;
      0 , 1 , 0  ,  0;
      0 , 0 , 10 ,  0; 
      0 , 0 , 0  , 100]; % initial Q matrix

R = 1; % initial R matrix

[K, S, P] = lqr(sys, Q, R); % LQR control

% initial conditions
x0 = 1.1;
dx0 = 0.4;
phi0 = 0.1;
dphi0 = 0.3;

% different weighting schemes
Q1 = [50, 0 , 0  ,  0;
       0 , 1 , 0  ,  0;
       0 , 0 , 5 ,  0; 
       0 , 0 , 0  , 50];

[K1, S1, P1] = lqr(sys, Q1, R);

Q2 = [125, 0 , 0  ,  0;
       0 , 75 , 0  ,  0;
       0 , 0 , 75 ,  0; 
       0 , 0 , 0  , 125];

[K2, S2, P2] = lqr(sys, Q2, R);

Q3 = [ 1 , 0 , 0 , 0;
       0 , 1 , 0 , 0;
       0 , 0 , 1 , 0; 
       0 , 0 , 0 , 1];

[K3, S3, P3] = lqr(sys, Q3, R);

% weigths for R
R1 = 1.5;
R2 = 3;
R3 = 6;

[KR1, S, P] = lqr(sys, Q, R1);
[KR2, S, P] = lqr(sys, Q, R2);
[KR3, S, P] = lqr(sys, Q, R3);

% simulation with initial Q
sim("TASK6_lqr_initial_q.slx")

% simulation with Q1 - Q3
sim("TASK6_lqr_different_q.slx")

% simulation with R1 - R3
sim("TASK6_lqr_different_r.slx")

% plot
figure(9)
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 16, 12]);
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [16 12]);
hold on
xlabel('Time [s]', fontsize=12)
ylabel('x [m]', fontsize=12)
title('LQR feedback control', fontsize=12)
grid('on')
plot(x_qinit.Time, x_qinit.Data, LineWidth=1.5)
plot(x_q1.Time, x_q1.Data, LineWidth=1.5)
plot(x_q2.Time, x_q2.Data, LineWidth=1.5)
plot(x_q3.Time, x_q3.Data, LineWidth=1.5)
legend('$\mathbf{Q}_{\mathrm{initial}}$', '$\mathbf{Q}_1$', ...
       '$\mathbf{Q}_2$', '$\mathbf{Q}_3$', ...
        Location='northeast', Interpreter='latex', fontsize=12)
saveas(gcf, 'images/task6_lqr_x.pdf', 'pdf');

figure(10)
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 16, 12]);
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [16 12]);
hold on
xlabel('Time [s]', fontsize=12)
ylabel('\phi [rad]', fontsize=12)
title('LQR feedback control', fontsize=12)
grid('on')
plot(phi_qinit.Time, phi_qinit.Data, LineWidth=1.5)
plot(phi_q1.Time, phi_q1.Data, LineWidth=1.5)
plot(phi_q2.Time, phi_q2.Data, LineWidth=1.5)
plot(phi_q3.Time, phi_q3.Data, LineWidth=1.5)
legend('$\mathbf{Q}_{\mathrm{initial}}$', '$\mathbf{Q}_1$', ...
       '$\mathbf{Q}_2$', '$\mathbf{Q}_3$', ...
        Location='northeast', Interpreter='latex', fontsize=12)
saveas(gcf, 'images/task6_lqr_phi.pdf', 'pdf');

figure(11)
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 16, 12]);
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [16 12]);
hold on
xlabel('Time [s]', fontsize=12)
ylabel('x [m]', fontsize=12)
title('LQR feedback control', fontsize=12)
grid('on')
plot(x_qinit.Time, x_qinit.Data, LineWidth=1.5)
plot(x_r1.Time, x_r1.Data, LineWidth=1.5)
plot(x_r2.Time, x_r2.Data, LineWidth=1.5)
plot(x_r3.Time, x_r3.Data, LineWidth=1.5)
legend('$R_{\mathrm{initial}}$', '$R_1$', ...
       '$R_2$', '$R_3$', ...
        Location='northeast', Interpreter='latex', fontsize=12)
saveas(gcf, 'images/task6_lqr_r_x.pdf', 'pdf');

figure(12)
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 16, 12]);
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [16 12]);
hold on
xlabel('Time [s]', fontsize=12)
ylabel('\phi [rad]', fontsize=12)
title('LQR feedback control', fontsize=12)
grid('on')
plot(phi_qinit.Time, phi_qinit.Data, LineWidth=1.5)
plot(phi_r1.Time, phi_r1.Data, LineWidth=1.5)
plot(phi_r2.Time, phi_r2.Data, LineWidth=1.5)
plot(phi_r3.Time, phi_r3.Data, LineWidth=1.5)
legend('$R_{\mathrm{initial}}$', '$R_1$', ...
       '$R_2$', '$R_3$', ...
        Location='northeast', Interpreter='latex', fontsize=12)
saveas(gcf, 'images/task6_lqr_r_phi.pdf', 'pdf');

%% Full state observer based on the measurement of x
disp('----------- TASK 7. Full state observer based on the measurement of x -----------')

close all

sigma_wd = [ 1 , 0  , 0 , 0  ;
             0 , 10 , 0 , 0  ;
             0 , 0  , 1 , 0  ; 
             0 , 0  , 0 , 10 ];

sigma_wn = 0.1;

% different covariances
sigma_wd2 = [ 1 , 0  , 0 , 0  ;
             0 , 1 , 0 , 0  ;
             0 , 0  , 1 , 0  ; 
             0 , 0  , 0 , 1 ];

sigma_wd3 = [ 10 , 0  , 0 , 0  ;
             0 , 1 , 0 , 0  ;
             0 , 0  , 10 , 0  ; 
             0 , 0  , 0 , 1 ];

% initial conditions
x0 = -0.01;
dx0 = 0.01;
phi0 = 0.05;
dphi0 = 0.1;

% noise settings
noise_variance = 0.0001;
noise_mean = 0;
noise_seed = 10000;
noise_sample_time = 0.001;

% Kalman filter state space model
L = lqe(A_lin, eye(4), C_lin(2,:), sigma_wd, sigma_wn);
KF = ss(A_lin-L*C_lin(2,:), [B_lin L], eye(4), 0*[B_lin L]);

% simulation using the default covariance
sim("TASK7_kalman_filter.slx")

% plot the results 
figure(13)
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 16, 12]);
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [16 12]);
hold on
xlabel('Time [s]', fontsize=12)
ylabel('\phi [rad]', fontsize=12)
title('\phi measurement for \sigma_{wd}', fontsize=12)
grid('on')
plot(phi_kf.Time, phi_kf.Data, LineWidth=2.0)
plot(phi_kf_filtered.Time, phi_kf_filtered.Data, LineWidth=2.0)
legend('\phi', '\phi_{filtered}', Location='southwest', fontsize=12)
saveas(gcf, 'images/task7_phi_cov1.pdf', 'pdf');

figure(14)
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 16, 12]);
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [16 12]);
hold on
xlabel('Time [s]', fontsize=12)
ylabel('x [m]', fontsize=12)
title('x measurement for \sigma_{wd}', fontsize=12)
grid('on')
plot(x_kf.Time, x_kf.Data, LineWidth=2.0)
plot(x_kf_filtered.Time, x_kf_filtered.Data, LineWidth=2.0)
legend('x', 'x_{filtered}', Location='northeast', fontsize=12)
saveas(gcf, 'images/task7_x_cov1.pdf', 'pdf');

% second covariance matrix 
figure(15)
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 16, 12]);
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [16 12]);
hold on
xlabel('Time [s]', fontsize=12)
ylabel('\phi [rad]', fontsize=12)
title('\phi measurement for \sigma_{wd2}', fontsize=12)
grid('on')
plot(phi_kf2.Time, phi_kf2.Data, LineWidth=2.0)
plot(phi_kf_filtered2.Time, phi_kf_filtered2.Data, LineWidth=2.0)
legend('\phi', '\phi_{filtered}', Location='southwest', fontsize=12)
saveas(gcf, 'images/task7_phi_cov2.pdf', 'pdf');

figure(16)
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 16, 12]);
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [16 12]);
hold on
xlabel('Time [s]', fontsize=12)
ylabel('x [m]', fontsize=12)
title('x measurement for \sigma_{wd2}', fontsize=12)
grid('on')
plot(x_kf2.Time, x_kf2.Data, LineWidth=2.0)
plot(x_kf_filtered2.Time, x_kf_filtered2.Data, LineWidth=2.0)
legend('x', 'x_{filtered}', Location='northeast', fontsize=12)
saveas(gcf, 'images/task7_x_cov2.pdf', 'pdf');

% third covariance matrix 
figure(17)
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 16, 12]);
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [16 12]);
hold on
xlabel('Time [s]', fontsize=12)
ylabel('\phi [rad]', fontsize=12)
title('\phi measurement for \sigma_{wd3}', fontsize=12)
grid('on')
plot(phi_kf3.Time, phi_kf3.Data, LineWidth=2.0)
plot(phi_kf_filtered3.Time, phi_kf_filtered3.Data, LineWidth=2.0)
legend('\phi', '\phi_{filtered}', Location='southwest', fontsize=12)
saveas(gcf, 'images/task7_phi_cov3.pdf', 'pdf');

figure(18)
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 16, 12]);
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [16 12]);
hold on
xlabel('Time [s]', fontsize=12)
ylabel('x [m]', fontsize=12)
title('x measurement for \sigma_{wd3}', fontsize=12)
grid('on')
plot(x_kf3.Time, x_kf3.Data, LineWidth=2.0)
plot(x_kf_filtered3.Time, x_kf_filtered3.Data, LineWidth=2.0)
legend('x', 'x_{filtered}', Location='northeast', fontsize=12)
saveas(gcf, 'images/task7_x_cov3.pdf', 'pdf');

%% Trajectory tracking
disp('----------- TASK 8. Trajectory tracking of the LQR system -----------')

close all

N=[0; 0; 1; 0];

sim("TASK8_trajectory_tracking.slx")

figure(19)
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 16, 12]);
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [16 12]);
hold on
xlabel('Time [s]', fontsize=12)
ylabel('x [m]', fontsize=12)
title('Trajectory tracking for LQR control', fontsize=12)
grid('on')
xlim([0, 74.9])
ylim([-0.2, 1.6])
plot(x_defined_traj.Time, x_defined_traj.Data, LineWidth=2.0)
plot(x_traj2.Time, x_traj2.Data, LineWidth=2.0)
plot(x_traj3.Time, x_traj3.Data, LineWidth=2.0)
legend('Defined trajectory', 'Actual trajectory with K', 'Actual trajectory with K3', Location='northeast', fontsize=12)
saveas(gcf, 'images/task8_trajectory.pdf', 'pdf');
