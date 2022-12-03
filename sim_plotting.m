function sim_plotting(axis, t, sim_vel, sim_pos, sim_inp, plan_vel, plan_traj)

    figure() % Plot of Positions Overlayed
    plot(t, plan_traj);
    hold on
    plot(t, sim_pos)
    legend(sprintf("%s_d",axis), sprintf("%s_o_u_t",axis), 'Location','northwest')
    title(sprintf('Simulated %s Axis Tracking',axis))
    xlabel('time (s)')
    ylabel('m')
    
    figure()
    
    subplot (3,2,1) % desired trajectory
    plot(t, plan_traj);
    title(sprintf('Desired Trajectory %s_d',axis))
    xlabel('time (s)')
    ylabel('m')
    
    subplot (3,2,3) % desired velocity
    plot(t, plan_vel);
    title('Desired Velocity v_d')
    xlabel('time (s)')
    ylabel('m/s')
    
    subplot(3,2,5) % controller input
    plot(t, sim_inp)
    title('Controller Input r_k')
    xlabel('time (s)')
    ylabel('m/s')
    
    subplot(3,2,2)
    plot(t, sim_pos);
    title(sprintf('System Trajectory %s_p',axis))
    xlabel('time (s)')
    ylabel('m')
    
    subplot(3,2,4)
    plot(t, sim_vel);
    title('System velocity v_p')
    xlabel('time (s)')
    ylabel('m/s')
    
    subplot(3,2,6)
    plot(t, sim_pos-plan_traj)
    title('Tracking Error')
    xlabel('time (s)')
    ylabel('m')

end