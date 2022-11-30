clc;close all;clear;

nopause=readtable('GT_BridgeCrane_Data.csv');
    
nopause(872:end,:)=[];

headers=nopause.Properties.VariableNames';


figure()
subplot(3,1,1)
plot(nopause.TimeY_dir_sec_,nopause.YActualVelocity_mm_sec_/1000);
title('Y Bridge Crane Velocity')
xlabel('time (s)')
ylabel('m/s')

subplot (3,1,2)
plot(nopause.TimeY_dir_sec_,((nopause.YCranePosition_mm_-nopause.YCranePosition_mm_(1))+nopause.YPayloadDeflection_mm_)/1000);
title('Y Bridge Crane Payload Position')
xlabel('time (s)')
ylabel('m')

subplot(3,1,3)
plot(nopause.TimeY_dir_sec_,nopause.YPayloadDeflection_mm_/1000);
title('Y Bridge Crane Payload Deflection')
xlabel('time (s)')
ylabel('m')


figure()
subplot(3,1,1)
plot(nopause.TimeY_dir_sec_,nopause.XActualVelocity_mm_sec_/1000);
title('X Bridge Crane Y Velocity')
xlabel('time (s)')
ylabel('m/s')

subplot (3,1,2)
plot(nopause.TimeY_dir_sec_,((nopause.XCranePosition_mm_-nopause.XCranePosition_mm_(1))+nopause.XPayloadDeflection_mm_)/1000);
title('X Bridge Crane Payload Position')
xlabel('time (s)')
ylabel('m')

subplot(3,1,3)
plot(nopause.TimeY_dir_sec_,nopause.XPayloadDeflection_mm_/1000);
title('X Bridge Crane Payload Deflection')
xlabel('time (s)')
ylabel('m')

figure()
plot(((nopause.YCranePosition_mm_-nopause.YCranePosition_mm_(1))+nopause.YPayloadDeflection_mm_)/1000,((nopause.XCranePosition_mm_-nopause.XCranePosition_mm_(1))+nopause.XPayloadDeflection_mm_)/1000);
title('Bridge Crane Payload Trajectory')
xlabel('time (s)')
ylabel('m')

