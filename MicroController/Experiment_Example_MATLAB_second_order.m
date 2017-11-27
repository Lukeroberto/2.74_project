function output_data = Experiment_Example_MATLAB()
    
    % Setup matlab figure
    figure(1); clf;           
    subplot(211); 
    h1 = plot([0],[0]);
    h1.XData = []; h1.YData = [];
    ylabel('Torque (Nm)');
    
    subplot(212);
    h2 = plot([0], [0]);
    h2.XData = []; h2.YData = [];
    ylabel('Current (A)');

    
    % Load trajectory information
    n = 1;
    filename = "../Trajectories/Torque_profile_" + n; 
    u = load(filename, "u");
    u = [-.1 0.1 0.1 0.1];
    
    function my_callback(new_data)
        t             = new_data(:,1); 
        v_out         = new_data(:,2); 
        desired_torque= new_data(:,3);
        current_sense = new_data(:,4);
        theta_2       = new_data(:,5); 
 
        N = length(t);
        
        h1.XData(end+1:end+N) = t;   % Update subplot 1
        h1.YData(end+1:end+N) = desired_torque;
        
        h2.XData(end+1:end+N) = t;   % Update subplot 2
        h2.YData(end+1:end+N) = current_sense;
%         h3.XData(end+1:end+N) = t;
%         h3.YData(end+1:end+N) = (v-R.*i)./vel;
%         h4.XData(end+1:end+N) = t;
%         h4.YData(end+1:end+N) = i;
    end

    frdm_ip  = '192.168.1.100';     % FRDM board ip
    frdm_port= 11223;               % FRDM board port
    params.callback = @my_callback; % callback function
    params.timeout  = 2;            % end of experiment timeout
     
    profile_size  = 500;
    R_motor       = 2.3;
    k_b           = 0.25;
    optional      = 0;

    input = [R_motor k_b profile_size u];    % input sent to FRDM board
    output_size = 5;    % number of outputs expected
    
    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);

end
