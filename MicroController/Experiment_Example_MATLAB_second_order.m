function output_data = Experiment_Example_MATLAB()
    
    % Setup matlab figure
    figure(1);  clf;          
    h1 = plot([0],[0]);
    h1.XData = []; h1.YData = [];
    ylabel('Position (Rad)');

    % Load trajectory information
    n = 1;
    filename = "../Trajectories/Torque_profile_" + n; 
    load(filename, "u");
    
    function my_callback(new_data)
        t         = new_data(:,1);   % time
        v_out     = new_data(:,2); % position
        torque   = new_data(:,3); % velocity
        theta_2   = new_data(:,4); % voltage
 
        N = length(t);

        h1.XData(end+1:end+N) = t;   % Update subplot 1
        h1.YData(end+1:end+N) = torque;
%         h2.XData(end+1:end+N) = t;   % Update subplot 2
%         h2.YData(end+1:end+N) = i;
%         h3.XData(end+1:end+N) = t;
%         h3.YData(end+1:end+N) = (v-R.*i)./vel;
%         h4.XData(end+1:end+N) = t;
%         h4.YData(end+1:end+N) = i;
    end

    frdm_ip  = '192.168.1.100';     % FRDM board ip
    frdm_port= 11223;               % FRDM board port
    params.callback = @my_callback; % callback function
    params.timeout  = 2;            % end of experiment timeout
     
    time_loop     = 2;
    R_motor       = 2.3;
    k_b           = 0.25;
    b             = -0.06;
    optional      = 0;

    input = [time_loop Torque R_motor k_b optional];    % input sent to FRDM board
    output_size = 4;    % number of outputs expected

    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);

end
