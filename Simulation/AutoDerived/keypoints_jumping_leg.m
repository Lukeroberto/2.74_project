function keypoints = keypoints_jumping_leg(in1,in2)
%KEYPOINTS_JUMPING_LEG
%    KEYPOINTS = KEYPOINTS_JUMPING_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    06-Nov-2017 23:36:07

l1 = in2(1,:);
l2 = in2(2,:);
th1 = in1(2,:);
y = in1(1,:);
t2 = cos(th1);
t3 = 1.0./l1.^2;
t4 = l2.^2;
t5 = t2.^2;
t6 = -t3.*t4.*t5+1.0;
t7 = sqrt(t6);
t8 = l1.*t7;
keypoints = reshape([0.0,t8+y+l2.*sin(th1),0.0,l2.*t2,t8+y,0.0,0.0,y,0.0],[3,3]);