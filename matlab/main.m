clc; clear; close all

fun = @objfun;                              % objective function
x0 = [0.4];                                 % initial guess
A =[]; b = [];                              % Linear Inequality Constraint: A*X<b
Aeq = []; beq = [];                         % Linear Equality Constraint: Aeq*X=beq
lb = [0.4]; ub = [.63];                     % Lower and Upper Bound of variables
nonlcon = [];                               % Nonlinear Constraint function
options = optimoptions('fmincon','Algorithm','sqp','Display','iter','StepTolerance',1e-15,'MaxFunctionEvaluations',5e+5);

[x_opt,fval,exitflag,output,lambda,grad,hesian] = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);
    