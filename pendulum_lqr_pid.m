%% 1. System Parameters (Same as your previous plot)
M = 0.5; m = 0.2; b = 0.1; I = 0.006; g = 9.81; l = 0.3;
d = I*(M+m) + M*m*l^2; 

A = [0      1              0           0;
     0 -(I+m*l^2)*b/d  (m^2*g*l^2)/d   0;
     0      0              0           1;
     0  (m*l*b)/d    m*g*l*(M+m)/d     0];
B = [0; (I+m*l^2)/d; 0; m*l/d];
C = eye(4); D = zeros(4,1);
t = 0:0.01:5; 
x0 = [0; 0; 0.15; 0]; % Initial 8.6 degree lean

%% 2. PID Setup (Feedback on Angle and Angular Velocity)
Kp = 100; Kd = 20;
K_pid = [0, 0, Kp, Kd]; 
sys_pid = ss(A - B*K_pid, B, C, D);
[y_pid, t_pid] = initial(sys_pid, x0, t);
u_pid = -y_pid * K_pid';

%% 3. LQR Setup (Optimal Multi-State Feedback)
Q = [500 0 0 0; 0 1 0 0; 0 0 100 0; 0 0 0 1];
R = 0.01;
K_lqr = lqr(A, B, Q, R);
sys_lqr = ss(A - B*K_lqr, B, C, D);
[y_lqr, t_lqr] = initial(sys_lqr, x0, t);
u_lqr = -y_lqr * K_lqr';

%% 4. The Comparative Plot (Styled like your screenshot)
figure('Color', [0.15 0.15 0.15]); 

% Subplot 1: Pendulum Angle
subplot(3,1,1);
plot(t_pid, y_pid(:,3), 'r--', 'LineWidth', 1.5); hold on;
plot(t_lqr, y_lqr(:,3), 'c', 'LineWidth', 2);
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w');
title('Pendulum Angle \theta(t)', 'Color', 'w');
ylabel('Angle (rad)'); legend('PID', 'LQR', 'TextColor', 'w'); grid on;

% Subplot 2: Cart Position
subplot(3,1,2);
plot(t_pid, y_pid(:,1), 'r--', 'LineWidth', 1.5); hold on;
plot(t_lqr, y_lqr(:,1), 'c', 'LineWidth', 2);
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w');
title('Cart Position x(t)', 'Color', 'w');
ylabel('Pos (m)'); legend('PID (Drift)', 'LQR (Fixed)', 'TextColor', 'w'); grid on;

% Subplot 3: Control Input
subplot(3,1,3);
plot(t_pid, u_pid, 'r--', 'LineWidth', 1.5); hold on;
plot(t_lqr, u_lqr, 'm', 'LineWidth', 2);
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w');
title('Control Input u(t) (Force Applied)', 'Color', 'w');
ylabel('Force (N)'); xlabel('Time (s)'); legend('PID Force', 'LQR Force', 'TextColor', 'w'); grid on;
