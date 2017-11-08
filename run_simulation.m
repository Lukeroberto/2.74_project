clear

% We can organize our code by filing things in different folders.  These
% folders need to be added to the Matlab path so that it can run the files
% inside them even when they are not the current folder listed at the top
% of the Matlab window.  For more information about the current folder, see
% http://www.mathworks.com/help/matlab/matlab_env/understanding-file-locations-in-matlab.html
% For more information about the Matlab path, see
% http://www.mathworks.com/help/matlab/matlab_env/what-is-the-matlab-search-path.html
setpath                                     % add AutoDerived, Modeling, and Visualization folders to Matlab path

                           % get parameters from file
z0 = [0; pi/6; 0; 0; 0; 0; 0];                    % set initial state
% Note: 5th state is the integral of torque squared over time
% An equation has been added to dynamics_continuous and dynamics_discrete
% to integrate this new state.

% set guess
tf = 1;                          % simulation final time
ctrl.tf = .5;                    % control time points
ctrl.T = [.5 .5 .5];             % control values limited by motor selection currently .77 Nm pololu

kappa = .5;
l_ratio = .9;
m_ratio = 1;

% % setup and solve nonlin.16/.9ear programming problem
problem.objective = @(x) objective(x,z0,ctrl, tf);     % create anonymous function that returns objective
problem.nonlcon = @(x) constraints(x,z0,ctrl, tf);     % create anonymous function that returns nonlinear constraints
problem.x0 = [kappa l_ratio m_ratio tf ctrl.tf ctrl.T];       % initial guess for decision variables
problem.lb = [.1 0.5 0.1 0.1 .1 -1.5*ones(size(ctrl.T))];     % lower bound on decision variables
problem.ub = [100  1  10  2   2  1.5*ones(size(ctrl.T))];     % upper bound on decision variables
problem.Aineq = []; problem.bineq = [];         % no linear inequality constraints ADD POWER CONSTRAINT, SPECIFY AS POLYNOMIAL, DERIVATIVE OF TORQUE
problem.Aeq = []; problem.beq = [];             % no linear equality constraints
problem.options = optimset('Display','iter');   % set options
problem.solver = 'fmincon';                     % required
x = fmincon(problem);                           % solve nonlinear programming problem

% Note that once you've solved the optimization problem, you'll need to 
% re-define tf, tfc, and ctrl here to reflect your solution.
kappa = x(1);
l_ratio = x(2);
m_ratio = x(3); 
tf = x(4);
ctrl.tf = x(5);
ctrl.T = x(6:end);
p = parameters(kappa, l_ratio, m_ratio);
[t, z, u, indices] = hybrid_simulation(z0,ctrl,p,[0 tf]); % run simulation

% Plot COM for your submissions
figure(1)
height = COM_jumping_leg(z,p);
plot(t, height(2,:))
xlabel('Time [s]')
ylabel('Height of COM [m]')

% Run the animation
figure(2)                          % get the coordinates of the points to animate
speed = .25;                                 % set animation speed
clf                                         % clear fig
animate_simple(t,z,p,speed)                 % run animation

figure(3)
subplot(321)
plot(t,z(2,:))
xlabel('Time [s]')
ylabel('Theta 1 [rad]')
subplot(323)
plot(t,z(3,:))
xlabel('Time [s]')
ylabel('Theta 2 [rad]')
subplot(325)
plot(t,z(3,:)-z(2,:))
xlabel('Time [s]')
ylabel('delta Theta [rad]')
subplot(322)
plot(t,z(5,:))
xlabel('Time [s]')
ylabel('Theta 1 [rad/s]')
subplot(324)
plot(t,z(6,:))
xlabel('Time [s]')
ylabel('Theta 2 [rad/s]')
subplot(326)
ctrl.t = linspace(0,ctrl.tf,length(ctrl.T));
u = interp1(ctrl.t,ctrl.T,ctrl.t,'linear','extrap');
plot(ctrl.t,u)
xlabel('Time [s]')
ylabel('Torque [Nm]')

