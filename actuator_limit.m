function u = actuator_limit(u, min, max)
    u(u<min)=min;
    u(u>max)=max;
end