function yd = traj_future(yd, s)
    yd = [yd(s+1:length(yd),:); ones(s, 1)*yd(length(yd),:)]; 
end