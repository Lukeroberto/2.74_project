function A = A_jumping_leg(in1,in2)
%A_JUMPING_LEG
%    A = A_JUMPING_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    06-Nov-2017 23:36:05

I1 = in2(8,:);
I2 = in2(9,:);
Is = in2(13,:);
c1 = in2(3,:);
c2 = in2(4,:);
l1 = in2(1,:);
l2 = in2(2,:);
m1 = in2(5,:);
m2 = in2(6,:);
mh = in2(7,:);
th1 = in1(2,:);
t2 = l2.^2;
t3 = cos(th1);
t4 = l2.*t3;
t5 = 1.0./l1;
t6 = sin(th1);
t7 = 1.0./l1.^2;
t8 = t3.^2;
t10 = t2.*t7.*t8;
t9 = -t10+1.0;
t11 = sqrt(t9);
t12 = conj(t11);
t13 = 1.0./t12;
t14 = t2.*t3.*t5.*t6.*t13;
t15 = 1.0./sqrt(t9);
t16 = t2.*t3.*t5.*t6.*t15;
t17 = t4+t14;
t18 = mh.*t17.*(1.0./2.0);
t19 = t4+t16;
t20 = mh.*t19.*(1.0./2.0);
t21 = c2.*t3.*2.0;
t22 = t14+t16+t21;
t23 = m2.*t22.*(1.0./2.0);
t24 = c1.*t2.*t3.*t6.*t7.*t15;
t25 = c1.*t2.*t3.*t6.*t7.*t13;
t26 = t24+t25;
t27 = m1.*t26.*(1.0./2.0);
t28 = t18+t20+t23+t27;
t29 = c2.*t6-l2.*t6;
t30 = c2.*t3;
t31 = c1.^2;
t32 = t6.^2;
A = reshape([m1+m2+mh,t28,0.0,t28,I2+m2.*(t29.^2.*2.0+(t14+t30).*(t16+t30).*2.0).*(1.0./2.0)+m1.*(t2.*t7.*t31.*t32.*2.0+1.0./l1.^4.*t2.^2.*t8.*t13.*t15.*t31.*t32.*2.0).*(1.0./2.0)+mh.*t17.*t19-(I1.*t2.*t7.*t32)./(t10-1.0),0.0,0.0,0.0,Is],[3,3]);
