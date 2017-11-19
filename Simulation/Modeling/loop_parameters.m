function p = loop_parameters() 
 ratios = [.1:.1:1 2:1:10]; %1/10 to 10/1 for l2/l1, length of 19
 space = size(ratios,2);
 num_stiff = 2;
 p = ones(space^2*num_stiff,13);
 cnt = 1;
 for i=1:19
     for j=1:19
         for k = 1:1:num_stiff
            l_tot = .5;
            l1 = l_tot/(1+ratios(i));
            l2 = ratios(i)*l1;
            m1  = .4*l1; %const density
            m2 = .4*l2; %const density
            mh = ratios(j)*(m1+m2);
            I1 = m1*l1^2/12; %paralled axis thm for bar
            I2 = m2*l2^2/12;
            c1 = l1/2;
            c2 = l2/2;
            g = 9.81;
            %kappa = .1;
            nu = 0;
            Is = .001;
            p(cnt,:) = [l1; l2; c1; c2; m1; m2; mh; I1; I2; g; k; nu; Is];
            cnt = cnt + 1;
         end
     end
 end
end