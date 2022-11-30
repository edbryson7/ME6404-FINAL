%% ME 6404 FINAL PROJECT CODE
clear;
clc;
close all;

%% Plant Properties
L = 1.730; % m  Length from trolley to hook; we can try three lengths for the cable 
wn=sqrt(9.81/L); % rad/s
z1 = Damping_Ratio_Log_Decrement(0.079-0.007,0.068-0.007); % function to calcuate the damping ratio

%% S Domain TF
A=[0 1;-(wn^2) -2*z1*wn];B=[0; 1]; C=[0 -1]; % State Space Representation: theta/V
D=[0];
[num,den]=ss2tf(A,B,C,D);
s = tf('s');
z = tf('z');

G_pos = tf(num,den) + 1/s;
G_vel = s*G_pos; % Transfer Function: Ydot/V

ts = 0.051;
TF_z = c2d(G_vel, ts);

%% Controller
C_z = inv(TF_z)*inv(z); % Model Inversion

%% Smooth Trajectory Generation
moves_y = [-.055 .570 .825 .400];
yd = moves_y(4)
tf = yd/0.15+3

% Smooth curves from 
[traj_y,vel_y,accel_y,t_y]=smooth_traj(0,yd,tf,.051);
vel_y = traj_future(vel_y, 1);

%% Controller Testing
r_y = lsim(C_z, vel_y, t_y);
r_y = actuator_limit(r_y, -0.2, 0.2);

y_vel = lsim(G_vel, r_y, t_y);
y_pos = lsim(G_pos, r_y, t_y);

RMSE = rms(y_pos - traj_y')
(y_pos(end) - traj_y(end))/traj_y(end)
%% Plotting
figure() % Plot of Positions Overlayed
plot(t_y, traj_y);
hold on
plot(t_y, y_pos)
% legend(["y_d", "y_o_u_t"])
title('Simulated ZPETC')
xlabel('time (s)')
ylabel('m')

figure()

subplot (3,2,1) % desired trajectory
plot(t_y, traj_y);
title('Desired Trajectory y_d')
xlabel('time (s)')
ylabel('m')

subplot (3,2,3) % desired velocity
plot(t_y, vel_y);
title('Desired Velocity v_d')
xlabel('time (s)')
ylabel('m/s')

subplot(3,2,5) % controller input
plot(t_y, r_y)
title('Controller Input r_k')
xlabel('time (s)')
ylabel('m/s')

subplot(3,2,2)
plot(t_y, y_pos);
title('System Trajectory y_p')
xlabel('time (s)')
ylabel('m')

subplot(3,2,4)
plot(t_y, y_vel);
title('System velocity v_p')
xlabel('time (s)')
ylabel('m/s')

subplot(3,2,6)
plot(t_y, y_pos-traj_y')
title('Tracking Error')
xlabel('time (s)')
ylabel('m')



