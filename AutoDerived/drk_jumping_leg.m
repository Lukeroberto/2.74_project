function drk = drk_jumping_leg(in1,tau,in3)
%DRK_JUMPING_LEG
%    DRK = DRK_JUMPING_LEG(IN1,TAU,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    06-Nov-2017 23:36:08

dth1 = in1(5,:);
dy = in1(4,:);
l1 = in3(1,:);
l2 = in3(2,:);
th1 = in1(2,:);
t2 = sin(th1);
t3 = l2.^2;
t4 = cos(th1);
drk = [-dth1.*l2.*t2;dy+(dth1.*t2.*t3.*t4.*1.0./sqrt(-1.0./l1.^2.*t3.*t4.^2+1.0))./l1;0.0];