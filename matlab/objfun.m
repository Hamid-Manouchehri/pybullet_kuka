function cost_val = objfun(grip_location)

    simulator = @py.main_kuka_sim.pybullet_simulator;
    tilt_angle = simulator(grip_location);

    cost_val = abs(tilt_angle);
   
end