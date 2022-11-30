%% ME 6404 FINAL PROJECT CODE
clear;
clc;
clear;
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

%% Obstacle Course Trajectories
speeds = [0.1 0.2]/2;
moves = [
    0 -55;
    570 0;
    0 825;
    400 0]/1000;
total_dist = sum(abs(moves));

start_pos = [-220, -400]/1000;

% Time alloted based on distance/speed for each axis
times = max((abs(moves)./speeds)')';

traj = [];
vel = [];
accel = [];
t = [];

prev = start_pos;
prev_t = 0;

for i = 1:4
    time = times(i)+1;
    move = moves(i,:)+prev;
    [traj_x,vel_x,accel_x,t_x]=smooth_traj(prev(1),move(1),time,ts);
    [traj_y,vel_y,accel_y,]=smooth_traj(prev(2),move(2),time,ts);
    
    traj = [traj; [traj_x traj_y]];
    vel = [vel; [vel_x, vel_y]];
    accel = [accel; [accel_x, accel_y]];
    prev = move;

    t = [t; t_x+prev_t+ts];
    prev_t = t(end);
    
end

vel = traj_future(vel, 1);

% figure()
% scatter(traj(:,2), traj(:,1))
% 
% figure()
% hold on
% plot(t,vel(:,1))
% plot(t,vel(:,2))
% legend('v_x', 'v_y')
% 
% figure()
% hold on
% plot(t,accel(:,1))
% plot(t,accel(:,2))
% legend('a_x', 'a_y')


%% Controller Testing

vel_y = vel(:,2);
t_y = t;
traj_y = traj(:,2);

r_y = lsim(C_z, vel_y, t_y);
r_y = actuator_limit(r_y, -0.2, 0.2);

y_vel = lsim(G_vel, r_y, t_y);
y_pos = lsim(G_pos, r_y, t_y)+start_pos(2);

RMSE = rms(y_pos - traj_y)
overshoot=(y_pos(end) - traj_y(end))/traj_y(end)
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
plot(t_y, y_pos-traj_y)
title('Tracking Error')
xlabel('time (s)')
ylabel('m')



