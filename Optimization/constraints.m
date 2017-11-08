function [cineq ceq] = constraints(x,z0, ctrl, tf)
% Inputs:
% x - an array of decision variables.
% z0 - the initial state
% p - simulation parameters
% 
% Outputs:
% cineq - an array of values of nonlinear inequality constraint functions.  
%         The constraints are satisfied when these values are less than zero.
% ceq   - an array of values of nonlinear equality constraint functions.
%         The constraints are satisfied when these values are equal to zero.
%
% Note: fmincon() requires a handle to an constraint function that accepts 
% exactly one input, the decision variables 'x', and returns exactly two 
% outputs, the values of the inequality constraint functions 'cineq' and
% the values of the equality constraint functions 'ceq'. It is convenient 
% in this case to write an objective function which also accepts z0 and p 
% (because they will be needed to evaluate the objective function).  
% However, fmincon() will only pass in x; z0 and p will have to be
% provided using an anonymous function, just as we use anonymous
% functions with ode45().
    p =  parameters(x(1), x(2), x(3));
    tf = x(4);
    ctrl.tf = x(5);
    ctrl.T = x(6:end);
    
    [t_1, z_1, u_1, ind_1, sols_1] = hybrid_simulation(z0,ctrl,p,[0 tf]);

    h_c = COM_jumping_leg(z_1(:,end),p); %find COM coordinates at end of sim
    apex = h_c(2); %y coord at end of sim
    velo = h_c(4); %y velocity at end of sim
    
    
    cineq = [-min(acos(((p(2)/p(1))*cos(z_1(2,:)))))]; 
%     cineq = [-min(acos(((p(1)/p(2))*cos(z_1(2,:))))), max(z_1(2,:))-pi/2]; %prevent falling thru floor and hyperextension                                       
    
%     ceq = [];
%     ceq = [ctrl.tf-t_1(ind_1(1))]; %enforce time criteria

    ceq = [ctrl.tf-t_1(ind_1(1)), velo]; %enforce time and apex criteria
                                                           
% simply comment out any alternate constraints when not in use
    
end