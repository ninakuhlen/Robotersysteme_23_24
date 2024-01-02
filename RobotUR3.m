classdef RobotUR3 < handle
    properties (Constant)
        % device ip and port
        DEVICEIP = '192.168.1.1';
        DEVICEPORT = 30003;

        % master ip and port
        MASTERIP = '192.168.0.89';
        MASTERPORT = 5000;

        % gripper ip and port
        GRIPPERIP = "";
        GRIPPERPORT = 0;

    end
    properties
        % robot movement parameters
        a = 0.3; % acceleration
        v = 0.5; % velocity
        t = 0; % time
        r = 0.001; % blend radius

        % socket connections
        deviceSocket
        masterSocket
        gripperSocket

        % hard coded home position
        % TODO: change from Kerze to actual home position!
        HOME = deg2rad([0.0, -90.0, 0.0, -90.0, -90, 0.0]);

        % inverse kinematic to calculate angle configurations
        inverseKinematic
    end
    methods
        function object = RobotUR3()

            % create an instance of an InverseKinematic object
            object.inverseKinematic = InverseKinematic();

            % initialize socket connections
            object.connectToDevice();
            object.connectToMaster();
            object.connectToGripper();
        end % RobotUR3

        function moveJ(self, q)

            % construct the command line for UR3
            commandString = ['movej([' num2str(q(1)) ', ' num2str(q(2)) ', ' num2str(q(3)) ', ' num2str(q(4))...
                ', ' num2str(q(5)) ', ' num2str(q(6)) '], ' num2str(self.a) ', ' num2str(self.v) ', ' num2str(self.t)...
                ', ' num2str(self.r) ')'];

            % send command to UR3
            fprintf(self.deviceSocket,commandString);

            self.checkMoving();
        end % moveJ

        function moveL(self, q)

            % construct the command line for UR3
            commandString = ['movej([' num2str(q(1)) ', ' num2str(q(2)) ', ' num2str(q(3)) ', ' num2str(q(4))...
                ', ' num2str(q(5)) ', ' num2str(q(6)) '], ' num2str(self.a) ', ' num2str(self.v) ', ' num2str(self.t)...
                ', ' num2str(self.r) ')'];

            % send command to UR3
            fprintf(self.deviceSocket,commandString);

            self.checkMoving();
        end % moveL

        function openGripper(self)
            commandString = 'set_tool_digital_out(1,False)';

            % send command to gripper
            fprintf(self.gripperSocket,commandString);

            % wait for gripper to move
            pause(1)
        end % openGripper

        function closeGripper(self)
            commandString = 'set_tool_digital_out(1,True)';

            % send command to gripper
            fprintf(self.gripperSocket,commandString);

            % wait for gripper to move
            pause(1)
        end % closeGripper

    end
    methods (Access = private)
        function connectToDevice(self)
            % create tcp/ip connection to robot
            self.deviceSocket = tcpclient(self.DEVICEIP, self.DEVICEPORT);

            % close prior socket connections
            fclose(self.deviceSocket);

            % try to connect to robot
            fprintf(1, 'Establishing connection to UR3...\n');

            pause(2);

            try
                fopen(self.deviceSocket);
            catch
                error('Connection to UR3 failed.');
            end

            pause(2);

            fprintf(1, 'Connected to UR3.\n');
            disp('Setup successful.')
        end % connectToDevice

        function connectToMaster(self)
            % create tcp/ip connection to master
            self.masterSocket = tcpclient(self.MASTERIP, self.MASTERPORT, "Timeout", 50);

            configureCallback(self.masterSocket, "byte", 8, @self.receiveDataFromMaster)

            pause(5);

            % test connection to master
            recvData = read(client, 8, "uint8");
            if recvData(1) == 1
                disp("control bit received, connection to master successful")
                com_data = [1,0,0,0,0,0,0,0];
                send_com_data(client, com_data)
                disp("send control bit")
                disp(com_data)
            end
        end % connectToMaster

        function connectToGripper(self)
            % create tcp/ip connection to gripper
            self.gripperSocket = tcpclient(self.GRIPPERIP, self.GRIPPERPORT);

            % close prior socket connections
            fclose(self.gripperSocket);

            % try to connect to robot
            fprintf(1, 'Establishing connection to gripper...\n');

            pause(2);

            try
                fopen(self.gripperSocket);
            catch
                error('Connection to gripper failed.');
            end

            pause(2);

            fprintf(1, 'Connected to gripper.\n');
            disp('Gripper setup successful.')
        end % connectToGripper

        function checkMoving(self)

            % set target velocities to zero
            targetVelocities = zeros(1,48);

            % initialize current velocities with ones, to ensure entering the while
            % loop
            currentVelocities = ones(1,48);

            % compare the current velocities to the target velocities
            while ~isequal(targetVelocities,currentVelocities)
                data = read(self.deviceSocket,500,"int8");

                % extract the position TARGET JOINT VELOCITIES (bytes 61 to 108)
                % https://www.universal-robots.com/articles/ur/interface-communication/remote-control-via-tcpip/#
                % document: Client_Interfaces_1.1.3
                % sheet: RealTime 3.2 -> 3.4
                % row: 6
                currentVelocities = data(61:108);
                pause(0.05);
            end
        end % checkMoving

        function recvDatafcn(self, src, ~)
            global processState;
            processState = StateClass;
            global testCounter;
            disp("Data recieved from server:")
            recvData = read(src, 8, "uint8");
            disp(recvData)

            if recvData(1) == 1
                disp("Controllbit is correct")
                processState.state = recvData(2);
                switch processState.state
                    case 1
                        testCounter = 1
                        src.Counter = testCounter
                        % Server is ready and started object detection
                        disp("start object detection")
                        com_data = [1,1,0,0,0,0,0,0];
                        %send_com_data(src, com_data)
                        disp(processState.state)
                        % return
                    case 2
                        testCounter = 2
                        % Server found object and started gripping
                        disp("server started gripping and approaching filling position")
                        com_data = [1,2,1,1,0,0,0,0];
                        send_com_data(src, com_data)
                    case 3
                        testCounter = 3
                        % Server moved to pouring position
                        disp("Server ready to pour")
                        com_data = [1,3,1,1,0,0,0,0];
                        %send_com_data(src, com_data)
                    case 4
                        testCounter = 4
                        % Server finished pouring and moving back to drop off
                        disp("Server finished pouring and moves back to drop off" + ...
                            "position")
                        com_data = [1,4,1,1,0,0,0,0];
                        %send_com_data(src, com_data)

                        %         case 4
                        %             state = 4;
                        %             % Server released object
                        %             disp("Server released object")
                        %             com_data = [1,4,1,1,0,0,0,0];
                        %             send_com_data(src, com_data)

                    otherwise
                        processState.state = -1;
                        disp("Server send wrong status")
                end

            elseif recvData(1) == 0
                disp("Controllbit is incorrect, something went wrong")
            end


            function send_com_data(client, com_data)
                write(client, com_data(:), "uint8")
            end


        end % receiveDataFromMaster
    end
end