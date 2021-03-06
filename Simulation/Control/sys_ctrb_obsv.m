%% Observability and Controllability of Linearized Nonlinear System

% Define symbolic parameters
syms t y dy ddy th1 th2 dth1 dth2 ddth1 ddth2 tau Fy l c1 c2 m1 m2 mh I1 I2 g kappa nu Is real

% Generate inputs to A and b functions
q   = [y; th1; th2];      % generalized coordinates
dq  = [dy; dth1; dth2];    % first time derivatives
ddq = [ddy; ddth1; ddth2];  % second time derivatives
u   = tau;          % control forces and moments
Fc   = Fy;           % constraint forces and moments
p   = [l; c1; c2; m1; m2; mh; I1; I2; g; kappa; nu; Is];  % parameters

z(1:3,1) = q;  
z(4:6,1) = dq;

A = A_jumping_leg(z,p);
b = b_jumping_leg(z,u,Fc,p);

x_ddot = A\b;

z_dot(1:3,1) = z(4:6);
z_dot(4:6,1) = x_ddot(1:3);


A_sys = jacobian(z_dot, z);
B_sys = jacobian(z_dot, u);

n = length(B_sys);

C = B_sys;
for i = 1:n-1
    C = [C (A_sys^(i-1))*B_sys];
end
size(C)
rank(C)

% p = parameters();

