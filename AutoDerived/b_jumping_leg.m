function b = b_jumping_leg(in1,tau,Fy,Fk,in5)
%B_JUMPING_LEG
%    B = B_JUMPING_LEG(IN1,TAU,FY,FK,IN5)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    06-Nov-2017 23:36:07

I1 = in5(8,:);
c1 = in5(3,:);
c2 = in5(4,:);
dth1 = in1(5,:);
dth2 = in1(6,:);
dy = in1(4,:);
g = in5(10,:);
kappa = in5(11,:);
l1 = in5(1,:);
l2 = in5(2,:);
m1 = in5(5,:);
m2 = in5(6,:);
mh = in5(7,:);
nu = in5(12,:);
th1 = in1(2,:);
th2 = in1(3,:);
t2 = cos(th1);
t3 = l2.^2;
t4 = t2.^2;
t5 = 1.0./l1;
t6 = sin(th1);
t7 = 1.0./l1.^2;
t11 = t3.*t4.*t7;
t8 = -t11+1.0;
t9 = 1.0./sqrt(t8);
t10 = t6.^2;
t12 = c2.*t6;
t13 = sqrt(t8);
t14 = conj(t13);
t15 = 1.0./t14;
t16 = 1.0./l1.^3;
t17 = t3.^2;
t18 = 1.0./t8.^(3.0./2.0);
t19 = 1.0./l1.^4;
t20 = 1.0./t14.^3;
t21 = t3.*t5.*t10.*t15;
t22 = t4.*t10.*t16.*t17.*t20;
t23 = l2.*t6;
t24 = t3.*t5.*t9.*t10;
t25 = t4.*t10.*t16.*t17.*t18;
t26 = c1.*dth1.*t3.*t7.*t10.*t15;
t27 = c1.*dth1.*t3.*t7.*t9.*t10;
t28 = c1.*dth1.*t4.*t10.*t17.*t18.*t19;
t29 = c1.*dth1.*t2.*t3.*t6.*t7.*t15;
t30 = dy+t29;
t31 = c1.*dth1.*t2.*t3.*t6.*t7.*t9;
t32 = dy+t31;
t33 = c1.^2;
t34 = t27+t28-c1.*dth1.*t3.*t4.*t7.*t9;
t35 = c1.*dth1.*t4.*t10.*t17.*t19.*t20;
t41 = t3.*t4.*t5.*t15;
t36 = t12+t21+t22-t41;
t37 = c2.*t2;
t44 = t3.*t4.*t5.*t9;
t38 = t12+t24+t25-t44;
t39 = t2.*t3.*t5.*t6.*t9;
t40 = t37+t39;
t42 = t2.*t3.*t5.*t6.*t15;
t43 = t37+t42;
t45 = l2.*t2;
t46 = t42+t45;
t47 = t23+t24+t25-t44;
t48 = t39+t45;
t49 = t21+t22+t23-t41;
t50 = t11-1.0;
t51 = dth1.*t43;
t52 = dy+t51;
t53 = dth1.^2;
t54 = t12-t23;
t55 = dth1.*t40;
t56 = dy+t55;
t57 = t8.^(3.0./2.0);
t58 = conj(t57);
t59 = 1.0./t58;
t60 = dth1.*t48;
t61 = dy+t60;
t62 = t4.*t10.*t16.*t17.*t59;
t63 = dth1.*t46;
t64 = dy+t63;
t65 = 1.0./t50.^2;
t66 = 1.0./t50;
t67 = th1.*2.0;
t68 = t67-th2.*2.0;
b = [Fy-g.*m1-g.*m2-g.*mh+dth1.*(m2.*(dth1.*t36+dth1.*t38).*(1.0./2.0)+m1.*(t26+t27+t28+t35-c1.*dth1.*t3.*t4.*t7.*t9-c1.*dth1.*t3.*t4.*t7.*t15).*(1.0./2.0)+dth1.*mh.*(t21+t22+t23-t3.*t4.*t5.*t15).*(1.0./2.0)+dth1.*mh.*(t23+t24+t25-t3.*t4.*t5.*t9).*(1.0./2.0));dth1.*(m1.*(-c1.*t3.*t4.*t7.*t9.*t30+c1.*t3.*t7.*t9.*t10.*t30-c1.*t3.*t4.*t7.*t15.*t32+c1.*t3.*t7.*t10.*t15.*t32-dth1.*t2.*t3.*t6.*t7.*t33.*4.0+c1.*t2.*t3.*t6.*t7.*t9.*(t26+t35-c1.*dth1.*t3.*t4.*t7.*t15)+c1.*t2.*t3.*t6.*t7.*t15.*t34+c1.*t4.*t10.*t17.*t18.*t19.*t30+c1.*t4.*t10.*t17.*t19.*t20.*t32).*(1.0./2.0)+m2.*(t38.*t52+t36.*t56-dth1.*t54.*(t37-l2.*t2).*4.0+dth1.*t36.*t40+dth1.*t38.*t43).*(1.0./2.0)+mh.*t49.*t61.*(1.0./2.0)+mh.*t47.*t64.*(1.0./2.0)+dth1.*mh.*t46.*t47.*(1.0./2.0)+dth1.*mh.*t48.*t49.*(1.0./2.0)+I1.*dth1.*t2.*t3.*t6.*t7.*t66.*2.0+I1.*dth1.*t2.*t6.*t10.*t17.*t19.*t65.*2.0)-kappa.*t68.*(1.0./2.0)-nu.*(dth1-dth2)-m2.*(dth1.*t38.*t52+dth1.*t56.*(t12+t21-t41+t62)-t53.*t54.*(t37-t45).*2.0).*(1.0./2.0)-m1.*(t32.*(t26-c1.*dth1.*t3.*t4.*t7.*t15+c1.*dth1.*t4.*t10.*t17.*t19.*t59)+t30.*t34-t2.*t3.*t6.*t7.*t33.*t53.*2.0).*(1.0./2.0)-Fk.*l2.*t6-g.*m2.*t40-g.*mh.*t48-dth1.*mh.*t47.*t64.*(1.0./2.0)-dth1.*mh.*t61.*(t21+t23-t41+t62).*(1.0./2.0)-I1.*t2.*t3.*t6.*t7.*t53.*t66-I1.*t2.*t6.*t10.*t17.*t19.*t53.*t65-c1.*g.*m1.*t2.*t3.*t6.*t7.*t9;tau+kappa.*t68.*(1.0./2.0)];
