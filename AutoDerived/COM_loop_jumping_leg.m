function COM = COM_loop_jumping_leg(in1,in2)
%COM_LOOP_JUMPING_LEG
%    COM = COM_LOOP_JUMPING_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    31-Oct-2017 15:28:30

c1 = in2(3);
c2 = in2(4);
dth1 = in1(5);
dy = in1(4);
l1 = in2(1);
l2 = in2(2);
m1 = in2(5);
m2 = in2(6);
mh = in2(7);
th1 = in1(2);
y = in1(1);
t2 = cos(th1);
t3 = m1+m2+mh;
t4 = 1.0./t3;
t5 = sin(th1);
t6 = l1.*t5;
t7 = c2.*t5;
t8 = l2.*t5;
t9 = c2.*t2;
t10 = l1.*t2;
t11 = c1.*m1.*t2;
COM = [t4.*(t11-m2.*(t9-l1.*t2)+mh.*(t10-l2.*t2));t4.*(m2.*(t6+t7+y)+mh.*(t6+t8+y)+m1.*(y+c1.*t5));-dth1.*t4.*(m2.*(t6-t7)+mh.*(t6-t8)+c1.*m1.*t5);dy+dth1.*t4.*(t11+m2.*(t9+t10)+mh.*(t10+l2.*t2))];
