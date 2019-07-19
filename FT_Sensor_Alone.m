
%%%% THIS CODE IS TO MEASURE THE OUTPUTS OF THE IIT FT-17 SENSOR %%%%%
%%%% - Erin Nolan, 07/2019 %%%

%% SETUP BEFORE STARTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%% Change the ethernet connection settings of your laptop to:
%%%% IP address: 192.168.1.2
%%%% Subnet Mask: 255.255.0.0

%%%% Make sure your firewall settings allow UDP connections to the sensor:
%%%% IP address: 192.168.1.1
%%%% Port number 23
%%%% Subnet Mask: 255.255.0.0

%%%% Make sure the power supply is ON and the switch underneath is ON

%%%% Connect the sensor via ethernet cable to your laptop.

%%%% Go to address in 192.168.1.1 browser to see demo page.
%%%% If you cannot connect then check security settings.
%%%% If you can see demo page then proceed with running this program.


%% RUNNING THE PROGRAM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%% START: Click "Run" |> 
%%%% STOP: Press the button to stop

%% CLEAR OUT PREVIOUS CODE %%

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end
clear; clc; close all;

%% FT SENSOR SETUP %%%

%Adds the folder with the UDP code to the current search path
addpath('IIIT_FT_17_Sensor','-end');

%Force sensor UDP buffer size
inputBuffer = 100;

%Set up Force sensor COMMS data structure
FT_SensorData = struct(...
        'ChRaw_Offs',int16(zeros(6,1)),...
        'FT',int32(zeros(6,1)),...
        'ChRaw',uint16(zeros(6,1)),...
        'temp_Vdc',int32(0),...
        'tStamp',int32(0),...
        'fault',int16(0),...
        'filt_FT',int32(zeros(6,1)),...
        'ft',double(zeros(6,1)),...
        'filt_ft',double(zeros(6,1)));
    
FT_Sensor_Poll = struct(...
       'Data', FT_SensorData,...
        'Policy0',uint8(0),...
        'Policy1',uint8(0),...
        'UDPPolicy',uint16(0),...
        'BoardNumber',uint8(0),...
        'IP',uint8(16),...
        'Port',uint8(0),...
        'UDPHandle',double(0),...
        'UDPRecvBuff', uint8(zeros(inputBuffer,1)));    

%Provision for multiple sensors, must be kept in, otherwise the struct
%sizes are mismatched and errors are thrown around.
for i = 1:2   
    
    FT_Sensor = FT_Sensor_Poll;
    
end  

%Set the force sensor mandatory broadcast policies, Policy1 = 0 is required
%Policy0 determines the amount of data transmitted, at Policy0 = 127,
%forces and torques for axes are broadcast.
FT_Sensor(1).Policy0 = 127;
FT_Sensor(1).Policy1 = 0;

%Board number definition, by default = 1, can be anything.
FT_Sensor(1).BoardNumber = 1;

%Initialise force and torque variables.
Fx = double(0);
Fy = double(0);
Fz = double(0);
Tx = double(0);
Ty = double(0);
Tz = double(0);

%% UDP SENSOR CONNECTION SETUP %%

% Clear the connection and restart in case anything else was previously
% left running
echoudp('off'); 
fclose('all');
echoudp('on',4012);

% Set values to the default address and port of IIT-FT sensors
FT_Sensor(1).UDPHandle = udp('192.168.1.1',23);
set(FT_Sensor(1).UDPHandle,'DatagramTerminateMode', 'off');
FT_Sensor(1).UDPHandle.Timeout = 0.1;
FT_Sensor(1).UDPHandle.InputBufferSize = inputBuffer;

%Change the default "off" of enabling port sharing so the data can be read
FT_Sensor(1).EnablePortSharing = 'on';

%Open a UDP connection at the above IP and port.
fopen(FT_Sensor(1).UDPHandle);

%Call the other functions, give them command strings and they
%translate and create and populate appropriate datagrams. These datagrams
%are then sent to buffer and broadcast to the sensor.

% sets the mandatory policies
SendUDPcommand('SET_SINGLE_UDP_PACKET_POLICY',FT_Sensor(1));
% receives an echo
SendUDPcommand('GET_SINGLE_UDP_PACKET',FT_Sensor(1));
% zeroes the sensor values
SendUDPcommand('UDP_CALIBRATE_OFFSETS',FT_Sensor(1));

%% COLLECT DATA %%%
try
   
    disp('FT-17 sensor started.');
    
    % THE STOP BUTTON %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    ButtonHandle = uicontrol('Style', 'PushButton', ...
                             'String', 'Stop Recording', ...
                             'Callback', 'delete(gcbf)', 'BackgroundColor','r', 'FontSize', 30);
    ButtonHandle.Position(3) = 400;
    ButtonHandle.Position(4) = 50;
    
    % The stop button will not display until the "pause" later in the code
    % Normal "input" functions cannot be run in parallel to the rest of the
    % code and so makes it wait for a user input before completing the
    % while loop, hence the GUI button is necessary to allow key stops.
    
    datetime.setDefaultFormats('default','HH:mm:ss.SSSSS');
    Start_timer = datetime('now','Format', 'HH:mm:ss.SSSSS');
    
    i = 0;
    
    while true
        
        FT_Sensor(1) = GetFTsensorData(FT_Sensor(1));

        Fx = FT_Sensor(1).Data.ft(1);
        Fy = FT_Sensor(1).Data.ft(2);
        Fz = -FT_Sensor(1).Data.ft(3); 
        F = [Fx;Fy;Fz];

        Tx = FT_Sensor(1).Data.ft(4);
        Ty = FT_Sensor(1).Data.ft(5);
        Tz = FT_Sensor(1).Data.ft(6);
        T = [Tx;Ty;Tz]; 

        Full_timer_t = datetime('now','Format', 'HH:mm:ss.SSSSS');
        Relative_timer = Full_timer_t - Start_timer;
%         disp(Relative_timer);

        if abs(Fx) > 50 || abs(Fy) > 50 || abs(Fz) > 25 || abs(Tx) > 0.5 || abs(Ty) > 0.5 || abs(Tz) > 0.5                   
            datetime('now')
            disp('FT-17 sensor overload.')
            break   
        end
        
        i = i+1;
        ForceArray_a(i,:) = F;
        TorqueArray_a(i,:) = T;
        TimeArray_a(i,1) = Relative_timer;

        pause(.01);
        if ~ishandle(ButtonHandle)
            datetime('now')
            disp('FT-17 sensor no longer recording.')
            break
        else
            continue
        end



    end

    %% CLOSE AND RESET %%%
    catch exception
        rethrow(exception);
end


%% SAVE DATA %%%

name = strcat(datestr(now,'yy-mm-dd'),'_',datestr(now,'HH-MM-SS'),'_ForceTest_.mat');
namefig = strcat(datestr(now,'yy-mm-dd'),'_',datestr(now,'HH-MM-SS'),'_ForceTest_.fig');

pa = plot(TimeArray_a(1:end),ForceArray_a(:,end));

%%%% This is to connect both directions if present %%%%
% hold on
% pb = plot(TimeArray_b(1:end),ForceArray_b(:,end));

saveas(pa,namefig);
save(name);

%% CLOSE DOWN AND CLEAR %%%

% Disconnect and clean up the server connection. 
echoudp('off');
fclose('all');

