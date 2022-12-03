clc; close all; clear;

plotCraneData('Data/GT_BridgeCrane_Data_700.csv', 'Bridge Crane .7m', 291)
% plotCraneData('Data/GT_BridgeCrane_Data_1150.csv', 'Bridge Crane 1.15m', 696)
% plotCraneData('Data/GT_BridgeCrane_Data_1730.csv', 'Bridge Crane 1.73m', 696)

function plotCraneData(filename, name, dataEnd)  
    data=readtable(filename);
    data(dataEnd:end,:)=[];

    plan_traj= load('trajectory.mat').traj;

    xstart = plan_traj(1,1);
    ystart = plan_traj(1,2);
    data.XCranePosition_mm_=data.XCranePosition_mm_-data.XCranePosition_mm_(1)+xstart*1000;
    data.YCranePosition_mm_ = data.YCranePosition_mm_-data.YCranePosition_mm_(1)+ystart*1000;
    
    figure()
    subplot(3,1,1)
    plot(data.TimeY_dir_sec_,data.YActualVelocity_mm_sec_/1000);
    title(sprintf("%s Y Velocity", name))
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
    plot(plan_traj(:,2), plan_traj(:,1))
    plot((data.YCranePosition_mm_+data.YPayloadDeflection_mm_-data.YPayloadDeflection_mm_(1))/1000,(data.XCranePosition_mm_+data.XPayloadDeflection_mm_-data.XPayloadDeflection_mm_(1))/1000);
    title(sprintf('Bridge Crane %s Payload Path', name))
    xlabel('Y (m)')
    ylabel('X (m)')
    legend("Planned Trajectory", "Payload Path",'Location','northwest')

end
