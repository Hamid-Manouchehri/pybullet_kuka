clc; clear; close all

fun = @objfunc;                              % objective function
x0 = 0.41;                                 % initial guess
A =[]; b = [];                            % Linear Inequality Constraint: A*X<b
Aeq = []; beq = [];                         % Linear Equality Constraint: Aeq*X=beq
lb = 0.40;
ub = 0.67;                     % Lower and Upper Bound of variables
nonlcon = [];                           % Nonlinear Constraint function
options = optimoptions('fmincon','Algorithm','sqp','Display','iter');
[x,fval] = fmincon(fun,x0,[], [], [], [], lb, ub, nonlcon, options);
