function p = parameters() 
 l = .25;
 m1  = .4*l;
 m2 = .4*l;
 mh = .1;
 I1 = m1*l^2/12;
 I2 = m2*l^2/12;
 c1 = l/2;
 c2 = l/2;
 g = 9.81;
 kappa = .5;
 nu = 0.2;
 Is = .001;
 p = [l; c1; c2; m1; m2; mh; I1; I2; g; kappa; nu; Is];
end