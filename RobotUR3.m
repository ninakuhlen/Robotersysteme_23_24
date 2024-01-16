classdef RobotUR3 < handle
    properties (Constant)
        % device ip and port
        DEVICEIP = '192.168.1.1';
        DEVICEPORT = 30003;

        % master ip and port
        % MASTERIP = '192.168.0.89';
        MASTERIP = 'localhost';
        MASTERPORT = 5000;

        % gripper ip and port
        GRIPPERIP = "";
        GRIPPERPORT = 0;

    end
    properties
        % robot movement parameters
        a = 0.3; % acceleration
        v = 0.5; % velocity
        % v = 0.25;
        t = 0; % time
        r = 0.001; % blend radius

        % socket connections
        deviceSocket
        masterSocket
        gripperSocket

        % hard coded home position
        HOME = deg2rad([180.0, -90.0, 0.0, -90.0, 90, 0.0]);

        state
        communicationData
    end
    methods
        function object = RobotUR3()

            % initialize socket connections
            object.connectToDevice();
            object.connectToMaster();
            % object.connectToGripper();

            object.moveJ(object.HOME);
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
            commandString = ['movel([' num2str(q(1)) ', ' num2str(q(2)) ', ' num2str(q(3)) ', ' num2str(q(4))...
                ', ' num2str(q(5)) ', ' num2str(q(6)) '], ' num2str(self.a) ', ' num2str(self.v) ', ' num2str(self.t)...
                ', ' num2str(self.r) ')'];

            % send command to UR3
            fprintf(self.deviceSocket,commandString);

            self.checkMoving();
        end % moveL

        function openGripper(self)
            % commandString = 'set_tool_digital_out(1,False)';

            % send command to gripper
            % fprintf(self.gripperSocket,commandString);
            fprintf(self.deviceSocket,'(4)');

            % wait for gripper to move
            pause(1)
        end % openGripper

        function closeGripper(self)
            % commandString = 'set_tool_digital_out(1,True)';

            % send command to gripper
            fprintf(self.deviceSocket,'(3)');

            % wait for gripper to move
            pause(1)
        end % closeGripper
        function set.state(obj,val)
            obj.state = val;
        end

        function answerMaster(self)
            write(self.masterSocket, self.communicationData, "uint8");
        end
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

            disp('Establishing connection to master...\n');

            % establish connection to master via tcp/ip
            % ip of master robot
            % self.masterSocket = tcpclient(self.MASTERIP,self.MASTERPORT,"ConnectTimeout",30, "Timeout", 1)
            % local host
            self.masterSocket = tcpclient("127.0.0.1",5000,"ConnectTimeout",30, "Timeout", 1);
            % ip of test pc
            % self.masterSocket = tcpclient('192.168.0.77',5000,"ConnectTimeout",30, "Timeout", 1)

            configureCallback(self.masterSocket, "byte", 1, @self.receiveDataFromConnection);
            % configureCallback(self.masterSocket, "terminator", @self.receiveDataFromConnection);


            % test connection to master
            self.communicationData = read(self.masterSocket, 8, "uint8");
            if self.communicationData(1) == 1
                disp("control bit received, connection to master successful")
                com_data = [1,0,0,0,0,0,0,0];
                sendComData(self.masterSocket, com_data)
                disp("send control bit")
                disp(com_data)
            end

            disp('Master connection setup successful.');
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
            disp('Gripper setup successful.');
        end % connectToGripper

        function checkMoving(self)

            % set target velocities to zero
            targetVelocities = zeros(1,48);

            % initialize current velocities with ones, to ensure entering the while
            % loop
            currentVelocities = ones(1,48);

            % compare the current velocities to the target velocities
            while ~isequal(targetVelocities,currentVelocities)
                socket = tcpclient(self.DEVICEIP,self.DEVICEPORT);
                data = read(socket,500,"int8");

                % extract the position TARGET JOINT VELOCITIES (bytes 61 to 108)
                % https://www.universal-robots.com/articles/ur/interface-communication/remote-control-via-tcpip/#
                % document: Client_Interfaces_1.1.3
                % sheet: RealTime 3.2 -> 3.4
                % row: 6
                currentVelocities = data(61:108);
                pause(0.05);

                clear socket;
            end
        end % checkMoving

        function receiveDataFromConnection(self, src, ~)
            self.communicationData = read(src, 8, "uint8");

            if self.communicationData(1) == 1
                disp("Controllbit is correct")
                self.state = self.communicationData(2);
            elseif self.communicationData(1) == 0
                disp("Controllbit is incorrect, something went wrong")
            end
        end % receiveDataFromMaster
    end
end

function sendComData(client, com_data)
write(client, com_data(:), "uint8")
end % sendComData