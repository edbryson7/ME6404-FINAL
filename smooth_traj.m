function [traj,vel,accel,t] = smooth_traj(y0, yf, tf, ts)
    yd0 = 0;
    ydf = 0;
    
    t = 0:ts:tf;
    t = t';

    a0 = y0;
    a1 = 0;
    a2 = 3/(tf^2) * (yf - y0);
    a3 = -2/(tf^3)*(yf-y0);
    
    traj = a0 + a1*t + a2*t.^2 + a3*t.^3;

    % UNCOMMENT THIS LINE FOR LINEAR TRAJECTORIES
%     traj = linspace(y0,yf, length(t));

    vel = diff(traj)/ts;
    vel = [vel; vel(end)];
    accel =diff(vel)/ts;
    accel = [accel; accel(end)];



    return % TAKE OUT THIS RETURN STATEMENT TO GET THE EVEN MORE SMOOTH TRAJECTORIES

    ydd0 = 0;
    yddf = 0;
    
    a0 = y0; a1 = yd0; a2 = ydd0/2;
    
    a3 = 1/(2*tf^3) * ( 20*yf - 20*y0 - (8*ydf + 12*yd0)*tf - (3*ydd0 - yddf)*tf^2 );
    a4 = 1/(2*tf^4) * ( 30*y0 - 30*yf + (14*ydf + 16*yd0)*tf + (3*ydd0 - 2*yddf)*tf^2 );
    a5 = 1/(2*tf^5) * ( 12*yf - 12*y0 - (6*ydf + 6*yd0)*tf - (ydd0 - yddf)*tf^2 );
    
    ydes = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;

    % UNCOMMENT THIS LINE FOR LINEAR TRAJECTORIES
%     traj = linspace(y0,yf, length(t));

    vel = diff(traj)/ts;
    vel = [vel; vel(end)];
    accel =diff(vel)/ts;
    accel = [accel; accel(end)];



    
end