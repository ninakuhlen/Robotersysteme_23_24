%% TCP Client 8 bit array integer

% client = tcpclient('192.168.0.77',5000,"Timeout",5)
% 
% configureCallback(client, "byte", 1, @recvDatafcn)
% pause(10)


% write(client, data, "uint8")
% recvData = read(client,1, "uint8")


function recvDatafcn(src, ~)
    global recvData; 
    recvData = read(src, 1, "uint8");
%     if recvData(1) == 0
%         disp("Controllbit is correct")
%     elseif recvData(1) == 1
%         disp("Controllbit is incorrect, something went wrong")
%     end
    % disp(recvData)
    % data = uint8(1);
    % write(src, data, "uint8")
    pause(1)
end

