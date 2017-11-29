%% Run Simulation Script 
% Setup and solve nonlinear programming problem for jumping leg. Saves the 
% torque profile to a mat file.

% Clear workspace
clear

% Add AutoDerived, Modeling, and Visualization folders to Matlab path
setpath           

% Initial States
z0 = [0; pi/6; 0; 0; 0; 0; 0];

% set guess
tf = .5;                         % simulation final time
ctrl.tf = .2;                    % control time points
ctrl.T = [1.4 1.4 1.4 1.4];      % control values limited by motor selection currently .77 Nm pololu

% Parameter Seeds
kappa = .5;
l_ratio = .9;
m_ratio = 1;

%% Setup and solve nonlinear programming problem
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

%% Solution
kappa = x(1);
l_ratio = x(2);
m_ratio = x(3); 
tf = x(4);
ctrl.tf = x(5);
ctrl.T = x(6:end);
p = parameters(kappa, l_ratio, m_ratio);
[t, z, u, indices] = hybrid_simulation(z0,ctrl,p,[0 tf]); % run simulation

%%  Plot COM for your submissions
figure(1)
height = COM_jumping_leg(z,p);
plot(t, height(2,:))
xlabel('Time [s]')
ylabel('Height of COM [m]')

%% Run the animation
figure(2)                                   % get the coordinates of the points to animate
speed = .25;                                % set animation speed
clf                                         % clear fig
animate_simple(t,z,p,speed)                 % run animation

%% Plotting of Important variables
figure(3)

% Theta 1 Profile
subplot(321)

plot(t,z(2,:))
xlabel('Time [s]')
ylabel('Theta 1 [rad]')

% Theta 2 Profile
subplot(323)

plot(t,z(3,:))
xlabel('Time [s]')
ylabel('Theta 2 [rad]')

% Delta Theta Profile
subplot(325)
plot(t,z(3,:)-z(2,:))
xlabel('Time [s]')
ylabel('delta Theta [rad]')

% Theta 1 Profile
subplot(322)

plot(t,z(5,:))
xlabel('Time [s]')
ylabel('Theta 1 [rad/s]')

% Theta 2 Profile
subplot(324)

plot(t,z(6,:))
xlabel('Time [s]')
ylabel('Theta 2 [rad/s]')

% Torque Profile
subplot(326)
plot(t(1:indices(1)), u(1:indices(1)));
xlabel('Time [s]')
ylabel('Torque [Nm]')


%% Power

figure(4)
motor_speed = z(6,:);
power = motor_speed(1:indices(1)).*u(1:indices(1));
plot(t(1:indices(1)),power)
ylabel('Power [W]')
xlabel('Time [s]')


%% Save the calculated torque profile to matlab
n = 1;
filename = "../Trajectories/Torque_profile_" + n; 
save(filename, "u");