function COM = COM_jumping_leg(in1,in2)
%COM_JUMPING_LEG
%    COM = COM_JUMPING_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    06-Nov-2017 23:36:08

c1 = in2(3,:);
c2 = in2(4,:);
dth1 = in1(5,:);
dy = in1(4,:);
l1 = in2(1,:);
l2 = in2(2,:);
m1 = in2(5,:);
m2 = in2(6,:);
mh = in2(7,:);
th1 = in1(2,:);
y = in1(1,:);
t2 = cos(th1);
t3 = m1+m2+mh;
t4 = 1.0./t3;
t5 = sin(th1);
t6 = 1.0./l1.^2;
t7 = l2.^2;
t8 = t2.^2;
t12 = t6.*t7.*t8;
t9 = -t12+1.0;
t10 = sqrt(t9);
t11 = l1.*t10;
t13 = c2.*t5;
t14 = l2.*t5;
t15 = 1.0./l1;
t16 = c2.*t2;
t17 = 1.0./sqrt(t9);
t18 = t2.*t5.*t7.*t15.*t17;
COM = [-t4.*(m2.*(t16-l2.*t2)-c1.*l2.*m1.*t2.*t15);t4.*(m2.*(t11+t13+y)+mh.*(t11+t14+y)+m1.*(y+c1.*t10));dth1.*t4.*(m2.*(t13-t14)-c1.*l2.*m1.*t5.*t15);dy+dth1.*t4.*(m2.*(t16+t18)+mh.*(t18+l2.*t2)+c1.*m1.*t2.*t5.*t6.*t7.*t17)];
