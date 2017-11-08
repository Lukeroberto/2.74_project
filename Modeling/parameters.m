function p = parameters(kappa, l_ratio, m_ratio) 
%l_ratio = l2/l1
%m_ration = mh/m_legs
l_tot = .20; 
m_tot = .5; 
l1 = l_tot/(1+l_ratio);
l2 = l_ratio*l1;
mh = m_tot/(1+1/m_ratio);
m_legs = m_tot-mh;
m2 = m_legs*l1/l_tot;
m1 = m_legs-m2;
I1 = m1*l1^2/12; %parallel axis thm for bar
I2 = m2*l2^2/12;
c1 = l1/2;
c2 = l2/2;
g = 9.81;
nu = .1;
Is = .01;
p = [l1; l2; c1; c2; m1; m2; mh; I1; I2; g; kappa; nu; Is];
end