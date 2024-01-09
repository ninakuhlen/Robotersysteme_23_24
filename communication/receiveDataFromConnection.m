function receiveDataFromConnection(src, ~)
global processState

recvData = read(src, 8, "uint8");

if recvData(1) == 1
    disp("Controllnum is correct")
    processState.state = recvData(2);
    if processState.state == 1
            % Server is ready and started object detection
            disp("start object detection")
            data = [1,1,0,0,0,0,0,0];
            % sendDataToConnection(src, data);
            return;          
    elseif processState.state == 2
            % Server found object and started gripping
            disp("server started gripping and approaching filling position")
            data = [1,2,1,1,0,0,0,0];
            % sendDataToConnection(src, data);
            return;
    elseif processState.state == 3
            % Server moved to pouring position
            disp("Server ready to pour")
            data = [1,3,1,1,0,0,0,0];
            % sendDataToConnection(src, data);
            return;
    elseif processState.state == 4
            % Server finished pouring and moving back to drop off
            disp("Server finished pouring and moves back to drop off" + ...
                "position")
            data = [1,4,1,1,0,0,0,0];
            % sendDataToConnection(src, data);
            return;
    else
            processState.state = -1;
            disp("Server send wrong status")
            return;
    end

elseif recvData(1) == 0
    disp("Controllbit is incorrect, something went wrong")
end
end


