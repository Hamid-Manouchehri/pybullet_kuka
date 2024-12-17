clc; clear; close all;

% Define the objective function
fun = @objfun;                             
x0 = 0.59;                                % Initial guess
A = []; b = [];                           % Linear Inequality Constraint: A*X<b
Aeq = []; beq = [];                       % Linear Equality Constraint: Aeq*X=beq
lb = 0.4; ub = .7;                      % Lower and Upper Bound of variables
nonlcon = [];                             % Nonlinear Constraint function

global iteration_data;
iteration_data = []; % Clear global variable

% Set fmincon options
options = optimoptions('fmincon', 'Algorithm', 'interior-point', ...
                       'OutputFcn', @outfun, ...
                       'Display', 'iter', ...
                       'StepTolerance', 1e-4, ...
                       'MaxFunctionEvaluations', 5e5, ...
                       'FiniteDifferenceStepSize', 1e-5, ...
                       'PlotFcn', 'optimplotfval');

% Run fmincon
[x_opt, fval] = fmincon(fun, x0, A, b, Aeq, beq, lb, ub, nonlcon, options);

ax = gca; % Get current axes
ax.FontSize = 20; % Change font size of axes
title(ax.Title.String, 'FontSize', 16);
xlabel(ax.XLabel.String, 'FontSize', 16);
ylabel(ax.YLabel.String, 'FontSize', 16);
axis padded;

disp("optimum point:");
disp(x_opt);    

% Plot cost (objective function value) vs. gripping location
figure;
plot(iteration_data(:,1), iteration_data(:,2), '-o', 'LineWidth', 1.5);
xlabel('Gripping Location (x (m))');
ylabel('Cost (|\Theta| (rad))');
title('Cost vs. Gripping Location');
fontsize(gca, 18, 'points')
axis padded;
grid on;
hold on;
% Highlight the last point
plot(iteration_data(end,1), iteration_data(end,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

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
