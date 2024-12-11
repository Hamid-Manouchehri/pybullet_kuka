fun = @objfun;                              % objective function
x0 = [0.41];                                 % initial guess
A =[]; b = [];                            % Linear Inequality Constraint: A*X<b
Aeq = []; beq = [];                         % Linear Equality Constraint: Aeq*X=beq
lb = [0.40];
ub = [0.67];                     % Lower and Upper Bound of variables
nonlcon = @mycon;                           % Nonlinear Constraint function
options = optimoptions('fmincon','Algorithm','sqp','Display','iter');
sentence = 'This is a messaged passed by fmincon to ';
[x,fval,exitflag,output,lambda,grad,hesian] = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options,sentence);