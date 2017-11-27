function output = RunExperiment(dest_ip,port,input,output_size,params)
    % RunExperiment
    % [output] = RunExperiment(dest_ip, port, input, output_size,params)
    %
    %  Sends input to the FRDM board and processes output data 
    %
    %  INPUT PARAMETERS
    %    dest_ip:    (string) ip address of FRDM board
    %    port:       (int) port used by server on FRDM board
    %    input:      (float vector) input variables for the experiment 
    %                which are sent to the FRDM board
    %    ouput_size: (int) number of floats to be sent back by the FRDM
    %                board at each control step
    %    params:     (struct) (optional) contains arguments below
    %                params.callback - callback function 
    %                params.timeout - timeout to end experiment (seconds)
    %
    %  OUTPUT PARAMETERS
    %    output:     (float vector) matrix of all data recieved from FRDM
    %                board during the experiment
    
    if nargin == 4
        params = struct();
    end
    if ~isfield(params,'callback')
        params.callback = 0;
    end
    
    if ~isfield(params,'timeout')
        params.timeout = 2;
    end
    
    
    done = 0;
    
    data             = []; % output data
    % boolean to store if there is a callback
    has_callback     = strcmp(class(params.callback) , 'function_handle');
    
    % event handler for packet reception
    function [] = recv(obj,es)
        if es.Data.DatagramLength == 1
            done = 1;
            return
        end
        % All packets must contains floats, which are of size 4
        % if the packet isn't divisible by 4, there is an error
        if mod(es.Data.DatagramLength,4*output_size) > 0
            fprintf(1,'Error bad data size %d expected mod %d\n',es.Data.DatagramLength,4*output_size);
            fread(obj, es.Data.DatagramLength);
            return
        end
        
        d = fread(obj, es.Data.DatagramLength);
        d = typecast(uint8(d),'single');
        %length(d)
        
        num_data = length(d)/output_size;
        
        d = reshape(d,output_size,num_data)';       
        data(end+1 : end+num_data,:) = d;
        
        if (has_callback)
            params.callback(d);
        end
    end

    % create port and bind handler
    u = udp(dest_ip,port);
    u.InputBufferSize = 1000000;
    u.DatagramReceivedFcn = @recv;
    u.DatagramTerminateMode = 'on';
    
    fopen(u);
    
    % Send input data to mbed
    fwrite(u,typecast(single(input),'uint8'))

    tic
    
    % Wait for experiment to finish
    k = 0;
    pd = 0;
    pk = 0;
    while done == 0
       k = k+1;
       
       if mod(k,20) == 0
           fprintf(' (%d bytes recieved, %d available)\n',u.ValuesReceived,u.BytesAvailable);
       else
           fprintf('.');
       end
       
       d = size(data,1);
       if d > pd
           pk = k;
           pd = d;
       elseif (k-pk) > params.timeout*10
           break
       end
       
       pause(.1);
    end
    fprintf('\nExperiment finished\n');
    toc


    fclose(u)
    delete(u)
    output = data;
end