clear all
[~,hostname] = system('hostname');
hostname = string(strtrim(hostname));
% address = "192.168.0.77"               % replace with IP-address of computer
address = "localhost"
% address = "172.20.10.5"
fprintf('Server-hostname: ')
fprintf(hostname)

server = tcpserver(address,5000,"ConnectionChangedFcn",@connectionFcn);

function connectionFcn(src, ~)
    if src.Connected
        disp("Client connection accepted by server.")
    
        % set control-num to true
        init_data = [1,0,0,0,0,0,0,0];
    
        % send innit_data to client
        disp("writing to client")
        write(src,init_data,"uint8")
        disp("writing done")
    end
    pause;
    % waitforbuttonpress
    data = [1,1,0,0,0,0,0,0]
    write(src, data, "uint8")
    pause;
    data = [1,2,1,1,0,0,0,0]
    write(src, data, "uint8")
    pause;
    data = [1,3,1,1,0,0,0,0]
    write(src, data, "uint8")
    pause;
    data = [1,4,1,1,0,0,0,0]
    write(src, data, "uint8")
    pause;
    data = [1,5,1,1,0,0,0,0]
    write(src, data, "uint8")
    
end
