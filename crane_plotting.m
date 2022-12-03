clc; close all; clear;

%% Plotting Model Inversion
start = [-.220,-.400];
smooth_traj = load('smooth_trajectory.mat');
fast_traj = load('trajectory.mat');
figure_eight_traj = load('eight_trajectory.mat');

plotCraneData('../Data/MI/MI_8figure.csv', 'Figure 8', 1320, figure_eight_traj)
% plotCraneData('../Data/MI/MI_1150.csv', '1.15m', 409, smooth_traj)
% plotCraneData('../Data/MI/MI_1730.csv', '1.73m', 566, smooth_traj)
% plotCraneData('../Data/MI/MI_1730_fast.csv', 'Fast Trajectory 1.73m', 362, plan_traj)
% plotCraneData('../Data/MI/MI_1730_fast_no_obstacles.csv', 'Fast Trajectory no obstacles 1.73m', 357, plan_traj)
% plotCraneData('../Data/MI/MI_1850.csv', '1.85m', 337, smooth_traj) %

%% Plotting Optimal Control
% plotCraneData('../Data/Opt/Optimal_1150.csv', '1.15m', 271, plan_traj)
% plotCraneData('../Data/Opt/Optimal_1500.csv', '1.50m', 320, plan_traj)
% plotCraneData('../Data/Opt/Optimal_1730.csv', '1.73m', 397, plan_traj)
% plotCraneData('../Data/Opt/Optimal_1730_no_obst.csv', 'No Obst 1.73m', 351, plan_traj)
% plotCraneData('../Data/Opt/Optimal_1850.csv', '1.85m', 336, plan_traj)




%% Plotting Input Shaping
modified_traj = load('trajectory.mat').traj;
% plotCraneData('../Data/ZV/ZV_1150.csv', '1.15m', 379, modified_traj)
% plotCraneData('../Data/ZV/ZV_1500.csv', '1.50m', 514, modified_traj)
% plotCraneData('../Data/ZV/ZV_1730.csv', '1.73m', 510, modified_traj)
% plotCraneData('../Data/ZV/ZV_1850.csv', '1.85m', 514, modified_traj)


%%
function plotCraneData(filename, name, dataEnd, plan_traj)  
    data=readtable(filename);
    data(dataEnd:end,:)=[];

%     plan_traj= load('trajectory.mat').traj;

    xstart = plan_traj.traj(1,1);
    ystart = plan_traj.traj(1,2);
    data.XCranePosition_mm_=data.XCranePosition_mm_-data.XCranePosition_mm_(1)+xstart*1000;
    data.YCranePosition_mm_ = data.YCranePosition_mm_-data.YCranePosition_mm_(1)+ystart*1000;
    
    figure()
    subplot(3,1,1)
    plot(data.TimeY_dir_sec_,data.YActualVelocity_mm_sec_/1000);
    title(sprintf("%s Y Cart Velocity", name))
    xlabel('time (s)')
    ylabel('m/s')
    
    subplot (3,1,2)
    plot(data.TimeY_dir_sec_,(data.YCranePosition_mm_+data.YPayloadDeflection_mm_-data.YPayloadDeflection_mm_(1))/1000);
    title(sprintf('%s Payload Y Position', name))
    xlabel('time (s)')
    ylabel('m')
    
    subplot(3,1,3)
    plot(data.TimeY_dir_sec_,data.YPayloadDeflection_mm_/1000);
    title(sprintf('%s Payload Y Deflection', name))
    xlabel('time (s)')
    ylabel('m')
    

    figure()
    subplot(3,1,1)
    plot(data.TimeY_dir_sec_,data.XActualVelocity_mm_sec_/1000);
    title(sprintf('%s Payload X Velocity', name))
    xlabel('time (s)')
    ylabel('m/s')
    
    subplot (3,1,2)
    plot(data.TimeY_dir_sec_,(data.XCranePosition_mm_+data.XPayloadDeflection_mm_-data.XPayloadDeflection_mm_(1))/1000);
    title(sprintf('%s Payload X Position', name))
    xlabel('time (s)')
    ylabel('m')
    
    subplot(3,1,3)
    plot(data.TimeY_dir_sec_,data.XPayloadDeflection_mm_/1000);
    title(sprintf('%s Payload X Deflection', name))
    xlabel('time (s)')
    ylabel('m')
    

    figure()
    hold on
    plot(plan_traj.traj(:,2), plan_traj.traj(:,1))
    plot((data.YCranePosition_mm_+data.YPayloadDeflection_mm_-data.YPayloadDeflection_mm_(1))/1000,(data.XCranePosition_mm_+data.XPayloadDeflection_mm_-data.XPayloadDeflection_mm_(1))/1000);
    title(sprintf('%s Payload Path', name))
    xlabel('Y (m)')
    ylabel('X (m)')
    legend("Planned Trajectory", "Payload Path",'Location','northwest')

end
