function output_data = Experiment_Example_MATLAB()
    
    % Setup matlab figure
    figure; clf;           
    subplot(221); 
    h1 = plot([0],[0]);
    h1.XData = []; h1.YData = [];
    ylabel('Power (W)');
    
    subplot(222);
    h2 = plot([0], [0]);
    h2.XData = []; h2.YData = [];
    ylabel('Torque Output (Nm)');

    subplot(223);
    h3 = plot([0], [0]);
    h3.XData = []; h3.YData = [];
    ylabel('Theta 2 (rad)');

    subplot(224);
    h4 = plot([0], [0]);
    h4.XData = []; h4.YData = [];
    ylabel('Theta 2 dot (rad/s)');

    
    % Load trajectory information
    n = 1;
    filename = "../Trajectories/Torque_profile_" + n; 
    u = load(filename, "u");
    %u = [-.5 0.5 0.5 0.5];
    u = 2.5*[1 1 1 1];
    ctrl_tf = 0.7;
    
    % Function Callback
    function my_callback(new_data)
        t             = new_data(:,1); 
        v_out         = new_data(:,2); 
        desired_torque= new_data(:,3);
        current_sense = new_data(:,4);
        theta_2       = new_data(:,5); 
        theta_2_dot   = new_data(:,6);
 
        N = length(t);
        
        h1.XData(end+1:end+N) = t;   % Update subplot 1
        h1.YData(end+1:end+N) = current_sense.*v_out;
        
        h2.XData(end+1:end+N) = t;   % Update subplot 2
        h2.YData(end+1:end+N) = current_sense*k_b;
            
        h3.XData(end+1:end+N) = t;   % Update subplot 3
        h3.YData(end+1:end+N) = theta_2;
        
        h4.XData(end+1:end+N) = t;   % Update subplot 4
        h4.YData(end+1:end+N) = theta_2_dot;
    
    end

    frdm_ip  = '192.168.1.100';     % FRDM board ip
    frdm_port= 11223;               % FRDM board port
    params.callback = @my_callback; % callback function
    params.timeout  = 4;            % end of experiment timeout
     
    profile_size  = 100;
    R_motor       = 2.3;
    k_b           = 0.25;

    input = [R_motor k_b profile_size ctrl_tf pi/7 100 0.1 1 12 0 u];
    output_size = 6;    % number of outputs expected
    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);
end
