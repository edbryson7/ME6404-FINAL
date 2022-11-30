function [traj,vel,accel,t] = smooth_traj(y0, yf, tf, ts)
    yd0 = 0;
    ydf = 0;
    
    t = 0:ts:tf;

    a0 = y0;
    a1 = 0;
    a2 = 3/(tf^2) * (yf - y0);
    a3 = -2/(tf^3)*(yf-y0);
    
    traj = a0 + a1*t + a2*t.^2 + a3*t.^3;
%     traj = linspace(y0,yf, length(t));
    vel = diff(traj)/ts;
    vel = [vel vel(end)];
    accel =diff(vel)/ts;
    accel = [accel accel(end)];
end