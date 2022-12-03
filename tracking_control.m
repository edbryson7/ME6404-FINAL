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
% Scaling the "max" speeds to slow down the trajectories
speeds = [0.1 0.2]./[1.2 1.2];                              % ADJUST THIS LINE TO CHANGE TIMINGS
moves = [
    0 -55;
    570 0;
    0 825;
    400 0]/1000;
total_dist = sum(abs(moves));

start_pos = [-220, -400]/1000;

% Time alloted based on distance/speed for each axis
times = max((abs(moves)./speeds)')'+1;                      % ADJUST THIS LINE TO CHANGE TIMINGS
times(1)=times(1)+0.5;



plan_traj = [];
plan_vel = [];
plan_accel = [];
t = [];

prev = start_pos;
prev_t = 0;

for i = 1:4 % Iterating through each move and generating the smooth path, then adding them together
    time = times(i)+1;
    move = moves(i,:)+prev;
    [traj_x,vel_x,accel_x,t_x]=smooth_traj(prev(1),move(1),time,ts);
    [traj_y,vel_y,accel_y,]=smooth_traj(prev(2),move(2),time,ts);
    
    plan_traj = [plan_traj; [traj_x traj_y]];
    plan_vel = [plan_vel; [vel_x, vel_y]];
    plan_accel = [plan_accel; [accel_x, accel_y]];
    prev = move;

    t = [t; t_x+prev_t+ts];
    prev_t = t(end);
    
end
t = t-ts;

% delay = 9;
% t = 0:ts:delay*2*pi;
% a = 1;
% b = 2;
% delta = pi/2;
% plan_traj = 0.5*[cos(a*t/delay+delta)',sin(b*t/delay)'];
% plan_vel = diff(plan_traj)/ts;
% plan_vel = [plan_vel; plan_vel(end,:)];
% plan_accel =diff(plan_vel)/ts;
% plan_accel = [plan_accel; plan_accel(end,:)];
% start_pos = [0 0];

s1.traj = plan_traj; s1.time = t;
save('eight_trajectory.mat', '-struct', 's1');

% plot(plan_traj(:,2), plan_traj(:,1))

plan_vel = traj_future(plan_vel, 1); % Shifting values 1 move from the future

figure()
hold on
plot(t,plan_vel(:,1))
plot(t,plan_vel(:,2))
legend('v_x', 'v_y')

figure()
hold on
plot(t,plan_accel(:,1))
plot(t,plan_accel(:,2))
legend('a_x', 'a_y')

fprintf("Total time: %.1f\n", t(end))



%% Controller Testing
% Simulate x
r_x = lsim(C_z, plan_vel(:,1), t);
r_x = actuator_limit(r_x, 0.1); % ACTUATOR SATURATION

x_vel = lsim(G_vel, r_x, t);
x_pos = lsim(G_pos, r_x, t)+start_pos(1);

% Simulate y
r_y = lsim(C_z, plan_vel(:,2), t);
r_y = actuator_limit(r_y, 0.2); % ACTUATOR SATURATION

y_vel = lsim(G_vel, r_y, t);
y_pos = lsim(G_pos, r_y, t)+start_pos(2);

sim_vel = [x_vel y_vel];
sim_pos = [x_pos y_pos];

sim_inp = [r_x r_y];

RMSE_x = rms(x_pos - plan_traj(:,1))
RMSE_y = rms(y_pos - plan_traj(:,2))

crane_commands = round(sim_inp./[.1 .2]*100); % Generating commands for the crane
writematrix(crane_commands, 'planned_trajectory.csv');

%% Plotting
figure()
hold on
plot(plan_traj(:,2), plan_traj(:,1))
plot(sim_pos(:,2), sim_pos(:,1))

legend('Planned Trajectory', 'Simulated Path', 'Location', 'northwest')

sim_plotting('Y', t, sim_vel(:,2), sim_pos(:,2), sim_inp(:,2), plan_vel(:,2), plan_traj(:,2));
sim_plotting('X', t, sim_vel(:,1), sim_pos(:,1), sim_inp(:,1), plan_vel(:,1), plan_traj(:,1));
