%% TCP Client 8 bit array integer
%function recvData()
client = tcpclient('192.168.0.89',5000,"Timeout",5)
pause(1)
configureCallback(client, "byte", 1, @recvDatafcn)


data = uint8(1)
write(client, data, "uint8")
% recvData = read(client,1, "uint8")
%end

function recvDatafcn(src, ~)
    recvData = read(client, 1, "uint8");
    if recvData(1) == 0
        disp("Controllbit is correct")
    elseif recvData(1) == 1
        disp("Controllbit is incorrect, something went wrong")
    end
    disp(recvData)
    write(client, data, "uint8")
    pause(1)
end