function u = actuator_limit(u, lim)
    u(u<-lim)=-lim;
    u(u>lim)=lim;
end