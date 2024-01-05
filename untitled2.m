clear all
[~,hostname] = system('hostname');
hostname = string(strtrim(hostname));
address = "192.168.0.77"               % replace with IP-address of computer

fprintf('Server-hostname: ')
fprintf(hostname)

server = tcpserver(address,5000,"ConnectionChangedFcn",@connectionFcn)
configureCallback(server,"byte",8,@readDataFcn);

init_data = uint8(zeros(1,8));




function connectionFcn(src, ~)
    if src.Connected
        disp("Client connection accepted by server.")
    
        % set control-Bit to true
        init_data = [1,0,0,0,0,0,0,0];
    
        % send innit_data to client
        disp("writing to client")
        write(src,init_data,"uint8")
        disp("writing done")
    end
    pause;
    % waitforbuttonpress
    data = [1,1,0,0,0,0,0,0];
    write(src, data, "uint8")
    pause;
    data = [1,2,1,1,0,0,0,0];
    write(src, data, "uint8")
    pause;
    data = [1,3,1,1,0,0,0,0];
    write(src, data, "uint8")
    pause;
    data = [1,4,1,1,0,0,0,0];
    write(src, data, "uint8")
end

% function readDataFcn(src, ~)
% disp("Data was received from the client.")
% 
% rx_data = read(src, 8, "uint8")
% 
% if rx_data(1) == 0 % Client throws an error
%     throw(rxDataError)
% end
% 
% switch rx_data(2)
%     case 0 % connec
%         % toin established, start finding object
%         disp("Connection correctly established.")
% 
%         % start object tracking
%         % send:
%         %   - status = 1
%         tx_data = [1,1,0,0,0,0,0,0]
% 
%         % send tx_data to client
%         write(src,tx_data(:),"uint8");
%     case 1 % compare if Master/Slave; start gripping
%         if rx_data(4) == 1
%             % compare master/slave index
%             if msBit ~= rx_data(3) & (rx_data(3)==0 | rx_data(3)==1)
%                 disp("Correct objects found")
%             else
%                 throw(objDetectError)
%             end
% 
%             %%%%%% start gripping object %%%%%%
% 
%             % send:
%             %   - master/slave (msBit)
%             %   - status = 2
%             tx_data = [1,2,msBit,0,0,0,0,0];
% 
%             % send tx_data to client
%             write(src,tx_data(:),"uint8");
%         else
%             throw(timingError)
%         end
%     case 2 % wait untill object is in position; start pouring
%         if rx_data(4) == 1
%             % send:
%             %   - status = 3
%             tx_data = [1,3,msBit,0,0,0,0,0];
%             write(src, tx_data(:), "uint8");
% 
%             %%%%%% start pouring %%%%%%
%         else
%             throw(timingError)
%         end
%     case 3 % wait untill pouring is finished; start moving to end position
%         if rx_data(4) == 1
%             %%%%%% start moving to end position %%%%%%
% 
%             % send:
%             %   - status =4
%             tx_data = [1,4,msBit,0,0,0,0,0];
%             write(src, tx_data(:), "uint8");
%         else
%             throw(timingError)
%         end
%     case 4 % wait untill object is dropped off
%         if rx_data(4) == 1
%             %%%%%% shutdown %%%%%%
%         else
%             throw(timingError)
%         end
%     otherwise
%         throw(statusError)
% end
% end
