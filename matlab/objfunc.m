function [cost_val] = objfunc(grip_location)

    simulator = @py.main_kuka_sim.python_simulator;
    tilt_angle = simulator(grip_location);

    cost_val = abs(tilt_angle);
    fprintf(['grip location: ' num2str(grip_location) ' cost value: ' num2str(cost_val) '\n\n'])
   
end