function yd = traj_future(yd, s)
    yd = [yd(s+1:length(yd)), ones(1, s)*yd(length(yd))]; 
end