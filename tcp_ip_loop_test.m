%% TCP Client 8 bit array integer
%function recvData()
%client = tcpclient('192.168.0.135',5000,"Timeout",20)
client = tcpclient('localhost',5000,"Timeout",20)

pause(5)
configureCallback(client, "byte", 8, @recvDatafcn)


com_data = uint8(zeros(1,8))
msBit = 0;

recvData = read(client, 8, "uint8")
if recvData(1) == 1
    com_data = [1,0,0,0,0,0,0,0]
    disp("send control bit")
    send_com_data(client, com_data) 
end

function recvDatafcn(src, ~)
    disp("Data recieved from server:")
    recvData = read(src, 8, "uint8");
    disp(recvData)

    if recvData(1) == 1
        disp("Controllbit is correct")
        switch recvData(2)
        case 0
            % Server is ready and started object detection
            disp("start object detection")
            com_data = [1,1,0,0,0,0,0,0]
            send_com_data(src, com_data)
        case 1
            % Server found object and started gripping
            disp("server started gripping")
            com_data = [1,1,1,1,0,0,0,0]
            send_com_data(src, com_data)
        case 2
            % Server moved to pouring position
            disp("Server ready to pour")
            com_data = [1,2,1,1,0,0,0,0]
            send_com_data(src, com_data)
        case 3
            % Server finished pouring and moving back to drop off
            disp("Server finisched pouring and moves back to drop off" + ...
                "position")
            com_data = [1,3,1,1,0,0,0,0]
            send_com_data(src, com_data)
        case 4
            % Server released object
            disp("Server released object")
            com_data = [1,4,1,1,0,0,0,0]
            send_com_data(src, com_data)
        otherwise
            disp("Server send wrong status")
        end

    elseif recvData(1) == 0
        disp("Controllbit is incorrect, something went wrong")
    end
end

function send_com_data(client, com_data)
    write(client, com_data(:), "uint8")
end

