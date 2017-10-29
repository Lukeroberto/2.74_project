function [tout, zout, uout, indices, sols] = hybrid_simulation(z0,ctrl,p,tspan)
%Inputs:
% z0 - the initial state
% ctrl- control structure
% p - simulation parameters
% tspan - [t0 tf] where t0 is start time, tf is end time
% 
% Outputs:
% tout - vector of all time points
% zout - vector of all state trajectories
% uout - vector of all control trajectories
% indicies - vector of indices indicating when each phases ended
%   for instance, tout( indices(1) ) provides the time at which
%   the first phase (stance) ended
% sols - vector of solution structures
%

    t0 = tspan(1); tend = tspan(end);   % set initial and final times

    %% Setup tolerance Options
    inttol = 1e-6;  % set integration tolerances
    iphase = 1;     % phase number is currently phase 1
    sols = [];      % initialize array of solution structures

    while(t0 < tend)

        % now include the number of the phase in the call to the event
        % function (1== stance, 2 == flight)
        opts = odeset('Events', @(t,z) event_conditions(t,z,ctrl,p,iphase), 'abstol',inttol,'reltol',inttol);
        f = @(t,z) dynamics_continuous(t, z, ctrl,p, iphase);    % we also include the number of the phase in the call to the EoM function
        sol = ode45(f, [t0 tend], z0, opts);    % integate until an event happens or time runs out
        sol.iphase = iphase;                    % store the phase number in the solution structure
        t0 = sol.x(end);                        % reset the integration initial time

        if isfield(sol,'ie') && ~isempty(sol.ie)
            z0 = sol.ye(:,end);                         % run the discrete dynamics function
            if any(sol.ie == 1)                         % if takeoff occured during integration
                iphase = 2;                             % phase is now phase 2
            elseif any(sol.ie == 2)                     % if touchdown occured during integration
                iphase = 1;                             % phase is now phase 1
            end
        else
            sol.ie = []; sol.xe = []; sol.ye = [];      % leave this just in case no event occured
        end
        sols = [sols; sol];                             % append solution structure to array
    end

    % assemble the results from each solution structure in the array
    tout = []; zout = []; uout = []; indices = [];
    for ii = 1:length(sols)
        sol = sols(ii);                                     % get the next solution structure
        iphase = sol.iphase;                                % get the phase number
        sol.x(1) = sol.x(1)+1e-6;                           % the time starts just a little after the last time ended (required for animation)
        tout = [tout sol.x];                                % append time points
        zout = [zout sol.y];                                % append states
        uout = [uout control_laws(sol.x,sol.y,ctrl,p,iphase)];   % append controls
        indices = [indices length(tout)];                   % append indices at which phases end
    end

end

%% Continuous dynamics
function [dz, Fc] = dynamics_continuous(t,z,ctrl,p,iphase)

    u = control_laws(t,z,ctrl,p,iphase);  % get controls at this instant
   
    % Contact model
    if iphase == 2  % in flight phase
       Fc = 0;
    else            % in stance phase
       C =  C_jumping_leg(z,p);
       if C < 0   
        dC= dC_jumping_leg(z,p);
        Fc = -500 * C - 3*dC;
        if Fc < 0
            Fc = 0;
        end
       else
        Fc = 0;
       end
    end
    
    A = A_jumping_leg(z,p);                 % get full A matrix
    b = b_jumping_leg(z,u,Fc,p);               % get full b vector
    
    x = A\b;                % solve system for accelerations (and possibly forces)

    dz(1:3,1) = z(4:6); % assign velocities to time derivative of state vector
    dz(4:6,1) = x(1:3);   % assign accelerations to time derivative of state vector

%     dz(7) = u^2;
    dz(7) = 1;              % change to integrate torque squared
end

%% Control
function u = control_laws(t,z,ctrl,p,iphase)

    if iphase == 1
        % Linearily interpolate torque in stance
        ctrl.t = linspace(0,ctrl.tf,length(ctrl.T));
        u = interp1(ctrl.t,ctrl.T,t,'linear','extrap');
    else
        
        % PD Control in flight
        th = z(2,:);            % leg angle
        dth = z(5,:);           % leg angular velocity

        thd = pi/4;             % desired leg angle
        k = 5;                  % stiffness (N/rad)
        b = .5;                 % damping (N/(rad/s))

        u = -k*(th-thd) - b*dth;% apply PD control
    end

end

%% Events
function [value,isterminal,direction] = event_conditions(t,z,ctrl,p,iphase)

    if iphase == 1                      % in stance phase
        [x, Fc] = dynamics_continuous(t,z,ctrl,p,iphase);
        value(1) = Fc;              
        isterminal(1) = 1;              % terminate integration when ground reaction force is zero
        direction(1) = -1;              % if it's decreasing
    else
        value(2) = z(1);                % value() is the foot height
        isterminal(2) = 1;              % terminate integration when foot height is zero
        direction(2) = -1;              % if it's decreasing
    end
end
