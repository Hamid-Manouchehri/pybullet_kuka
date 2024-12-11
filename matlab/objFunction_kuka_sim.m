function f = objfunc(x)
    simulator = @py.sim_py.simulate
    tilt_angle = simulator(x)
    f = abs(tilt_angle)
end