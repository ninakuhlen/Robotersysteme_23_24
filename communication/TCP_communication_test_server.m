%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TCP/IP communication Test
%%%%%%%%%%%%%%%%%%%%%%%%%%%

[~,hostname] = system('hostname');
hostname = string(strtrim(hostname));
address = '127.0.0.1'               % replace with IP-address of computer

fprintf('Server-hostname: ')
fprintf(hostname)

server = tcpserver(address,5000,"ConnectionChangedFcn",@connectionFcn)
configureCallback(server,"byte",1,@readDataFcn);
innit_data = 0;

function connectionFcn(src, ~)

    if src.Connected
        disp("Client connected")
        
        % Send control bit
        innit_data = 1;
        write(src, innit_data, "uint8");
    end

end

function readDataFcn(src, ~)

    rx_data = read(src,1,"uint8");
    
    if rx_data == 1
        disp("Data received correctly")
    else
        disp("Data received incorrectly, check connection")
        disp(rx_data)
    end
    
    tx_data = 0;
    write(src, tx_data, "uint8");

end