function output_data = Experiment_Example_MATLAB()
    figure(1);  clf;       % Create an empty figure to update later
%     subplot(221)
    h1 = plot([0],[0]);
    h1.XData = []; h1.YData = [];
    ylabel('Position (Rad)');

%     subplot(222)
%     h2 = plot([0],[0]);
%     h2.XData = []; h2.YData = [];
%     ylabel('Current (amps)');

%     subplot(223)
%     h3 = plot([0], [0]);
%     h3.XData = []; h3.YData = [];
%     ylabel('Back EMF (V*s)');
%     axis([0 5 -2 2])
% 
%     subplot(224)
%     h4 = plot([0], [0]);
%     h4.XData = []; h4.YData = [];
%     ylabel('Current [Amps]');

    % This function will get called any time there is new data from
    % the FRDM board. Data comes in blocks, rather than one at a time.
    function my_callback(new_data)
        R = 2.3; % ohms
        t = new_data(:,1);   % time
        pos = new_data(:,2); % position
        vel = new_data(:,3); % velocity
        v   = new_data(:,4); % voltage
        i   = new_data(:,5); % current
        N = length(pos);

        h1.XData(end+1:end+N) = t;   % Update subplot 1
        h1.YData(end+1:end+N) = pos;
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
     
    K_p       = 1;
    time_loop = 2;
    Res       = 2.3;
    k_b       = 0.25;
    K         = 1;
    b         = -0.06;
    pos_d     = pi;
    optional  = 0;

    input = [K_p time_loop Res k_b K b pos_d optional ];    % input sent to FRDM board
    output_size = 5;    % number of outputs expected

    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);

end
