clc; clear; close all;

% Define the objective function
fun = @objfun;                             
x0 = 0.5;                                 % Initial guess
A = []; b = [];                           % Linear Inequality Constraint: A*X<b
Aeq = []; beq = [];                       % Linear Equality Constraint: Aeq*X=beq
lb = 0.4; ub = 0.67;                      % Lower and Upper Bound of variables
nonlcon = [];                             % Nonlinear Constraint function

global iteration_data;
iteration_data = []; % Clear global variable

% Set fmincon options
options = optimoptions('fmincon', 'Algorithm', 'sqp', ...
                       'OutputFcn', @outfun, ...
                       'Display', 'iter', ...
                       'StepTolerance', 1e-8, ...
                       'MaxFunctionEvaluations', 5e5);

% Run fmincon
[x, fval, exitflag, output, lambda, grad, hessian] = fmincon(fun, x0, A, b, Aeq, beq, lb, ub, nonlcon, options);

% Plot cost (objective function value) vs. gripping location
figure;
plot(iteration_data(:,1), iteration_data(:,2), '-o', 'LineWidth', 1.5);
xlabel('Gripping Location (x)');
ylabel('Cost (abs(\Theta))');
title('Cost vs. Gripping Location');
fontsize(gca, 18, 'points')
grid on;

% Objective Function
function cost = objfun(x)
    cost = abs(x - 0.55); % Replace this with your actual cost function
end

% Output Function to capture iteration data
function stop = outfun(x, optimValues, state)
    global iteration_data;
    stop = false;

    switch state
        case 'init'
            iteration_data = []; % Initialize storage
        case 'iter'
            % Store current gripping location and cost
            grip_location = x;
            cost_val = optimValues.fval;
            iteration_data = [iteration_data; grip_location, cost_val];
        case 'done'
            % Final actions after optimization (if needed)
    end
end
