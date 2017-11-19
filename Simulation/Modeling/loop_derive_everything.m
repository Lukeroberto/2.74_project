function loop_derive_everything() 
name = 'loop_jumping_leg';

% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
syms t y dy ddy th1 th2 dth1 dth2 ddth1 ddth2 tau Fy l1 l2 c1 c2 m1 m2 mh I1 I2 g kappa nu Is real
%additional state from the spring

% Group them for later use.
q   = [y; th1; th2];      % generalized coordinates
dq  = [dy; dth1; dth2];    % first time derivatives
ddq = [ddy; ddth1; ddth2];  % second time derivatives
u   = tau;          % control forces and moments
Fc   = Fy;           % constraint forces and moments
p   = [l1; l2; c1; c2; m1; m2; mh; I1; I2; g; kappa; nu; Is];  % loop_parameters

%%% Calculate important vectors and their time derivatives.

% Define fundamental unit vectors.  The first element should be the
% horizontal (+x cartesian) component, the second should be the vertical (+y
% cartesian) component, and the third should be right-handed orthogonal.
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = cross(ihat,jhat);

% Define other unit vectors for use in defining other vectors.
er1hat =  cos(th1)*ihat + sin(th1) * jhat;
er2hat = -cos(th1)*ihat + sin(th1) * jhat;

% A handy anonymous function for taking first and second time derivatives
% of vectors using the chain rule.  See Lecture 6 for more information. 
ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; 

% Define vectors to key points.
rf = y*jhat;
rcm1 = rf+c1*er1hat;
rk = rf+l1*er1hat;
rcm2 = rk + c2*er2hat;
rh = rk + l2*er2hat;
keypoints = [rh rk rf];

% Take time derivatives of vectors as required for kinetic energy terms.
drcm1 = ddt(rcm1);
drcm2 = ddt(rcm2);
drh   = ddt(rh);

%%% Calculate Kinetic Energy, Potential Energy, and Generalized Forces

% F2Q calculates the contribution of a force to all generalized forces
% for forces, F is the force vector and r is the position vector of the 
% point of force application
F2Q = @(F,r) simplify(jacobian(r,q)'*(F)); 

% M2Q calculates the contribution of a moment to all generalized forces
% M is the moment vector and w is the angular velocity vector of the
% body on which the moment acts
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M)); 

% Define kinetic energies. See Lecture 6 formula for kinetic energy
% of a rigid body.
T1 = (1/2)*m1*dot(drcm1, drcm1) + (1/2)* I1 * dth1^2;
T2 = (1/2)*m2*dot(drcm2, drcm2) + (1/2)* I2 * (-dth1)^2;
Th = (1/2)*mh*dot(drh, drh);
Ts = (1/2)*Is*(dth2)^2;

% Define potential energies. See Lecture 6 formulas for gravitational 
% potential energy of rigid bodies and elastic potential energies of
% energy storage elements.
V1 = m1*g*dot(rcm1, jhat);
V2 = m2*g*dot(rcm2, jhat);
Vh = mh*g*dot(rh, jhat);
Vs = (1/2)*kappa*(th2-th1)^2;

% Define contributions to generalized forces.  See Lecture 6 formulas for
% contributions to generalized forces.
QF = F2Q(Fy*jhat,rf);
Qtau_1 = M2Q(-tau*khat, -dth2*khat);
Qdamp = M2Q(-nu*(dth2-dth1)*khat, -dth1*khat);

% Sum kinetic energy terms, potential energy terms, and generalized force
% contributions.
T = T1 + T2 + Th + Ts;
V = V1 + V2 + Vh + Vs;
Q = QF + Qtau_1 + Qdamp;

% Calculate rcm, the location of the center of mass
rcm = (m1*rcm1 + m2*rcm2 + mh*rh)/(m1+m2+mh);

% Assemble C, the set of constraints
C = y;  % When y = 0, the constraint is satisfied because foot is on the ground
dC= ddt(C);

%% All the work is done!  Just turn the crank...
%%% Derive Energy Function and Equations of Motion
E = T+V;                                         % total system energy
L = T-V;                                         % the Lagrangian
eom = ddt(jacobian(L,dq)') - jacobian(L,q)' - Q;  % form the dynamics equations

% size(eom)

%%% Rearrange Equations of Motion. 
A = jacobian(eom,ddq);
b = A*ddq - eom;

%%% Write functions to evaluate dynamics, etc...
z = sym(zeros(length([q;dq]),1)); % initialize the state vector
z(1:3,1) = q;  
z(4:6,1) = dq;

% Write functions to a separate folder because we don't usually have to see them
directory = '../AutoDerived/';
% Write a function to evaluate the energy of the system given the current state and parameters
matlabFunction(E,'file',[directory 'energy_' name],'vars',{z p});
% Write a function to evaluate the A matrix of the system given the current state and parameters
matlabFunction(A,'file',[directory 'A_' name],'vars',{z p});
% Write a function to evaluate the b vector of the system given the current state, current control, and parameters
matlabFunction(b,'file',[directory 'b_' name],'vars',{z u Fc p});

matlabFunction(keypoints,'file',[directory 'keypoints_' name],'vars',{z p});

matlabFunction(C,'file',[directory 'C_' name],'vars',{z u p});
matlabFunction(dC,'file',[directory 'dC_' name],'vars',{z u p});

% Write a function to evaluate the X and Y coordinates and speeds of the center of mass given the current state and parameters
drcm = ddt(rcm);             % Calculate center of mass velocity vector
COM = [rcm(1:2); drcm(1:2)]; % Concatenate x and y coordinates and speeds of center of mass in array
matlabFunction(COM,'file',[directory 'COM_' name],'vars',{z p});
