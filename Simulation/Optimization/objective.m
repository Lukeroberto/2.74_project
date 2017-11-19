function f = objective(x,z0, ctrl, tf)
% Inputs:
% x - an array of decision variables.
% z0 - the initial state
% p - simulation parameters
% 
% Outputs:
% f - scalar value of the function (to be minimized) evaluated for the
%     provided values of the decision variables.
%
% Note: fmincon() requires a handle to an objective function that accepts 
% exactly one input, the decision variables 'x', and returns exactly one 
% output, the objective function value 'f'.  It is convenient for this 
% assignment to write an objective function which also accepts z0 and p 
% (because they will be needed to evaluate the objective function).  
% However, fmincon() will only pass in x; z0 and p will have to be
% provided using an anonymous function, just as we use anonymous
% functions with ode45().
    p = parameters(x(1), x(2), x(3));
    tf = x(4);
    ctrl.tf = x(5);
    ctrl.T = x(6:end);
    
    [t_1, z_1, u_1, ind_1, sols_1] = hybrid_simulation(z0,ctrl,p,[0 tf]);
    h_c = COM_jumping_leg(z_1,p);
    f = -max(h_c(2,end));    % negative of COM height

    % alternate objective functions:
%     f = x(1);   % final time
%     f = z_1(5,end);    % minimize T^2 integral
   
end