clear; clc; close all;

data_n = 50;
repeats = 1;

Mff = zeros(3,data_n);
Mfb = zeros(3,data_n);

%Declare the COMMS port for linear rail 
port = serial('COM6');

%Set up the default ASCII protocol for linear rail
set(port, ...
    'BaudRate', 115200, ...
    'DataBits', 8, ...
    'FlowControl', 'none', ...
    'Parity', 'none', ...
    'StopBits', 1);

%Stop Zaber delay
set(port, 'Timeout', 0.5)
warning off MATLAB:serial:fread:unsuccessfulRead

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

% First disable the Echo of the UDP (not sure why), then close any open UDP
% connections.
echoudp('off'); 
fclose('all');

%Turn echo on again for some reason.
echoudp('on',4012);

%Populate the struct of the default properties.
FT_Sensor(1).UDPHandle = udp('192.168.1.1',23);
set(FT_Sensor(1).UDPHandle,'DatagramTerminateMode', 'off');
FT_Sensor(1).UDPHandle.Timeout = 0.1;
FT_Sensor(1).UDPHandle.InputBufferSize = inputBuffer;

%Open a UDP connection at the above IP and port.
fopen(FT_Sensor(1).UDPHandle);

%Call the other functions, give them command strings and they
%translate and create and populate appropriate datagrams. These datagrams
%are then sent to buffer and broadcast to the sensor. First command sets
%the mandatory policies, second receives an echo and third zeroes the
%sensor values.

SendUDPcommand('SET_SINGLE_UDP_PACKET_POLICY',FT_Sensor(1));

SendUDPcommand('GET_SINGLE_UDP_PACKET',FT_Sensor(1));
 
SendUDPcommand('UDP_CALIBRATE_OFFSETS',FT_Sensor(1));

%Open up a Serial connection at the USB port to talk to linear rail
fopen(port);

%Declare the COMMS protocol for the linear rail.
protocol = Zaber.AsciiProtocol(port);

try
    
device = Zaber.AsciiDevice.initialize(protocol, 1);

%Save and set absolute maximum speed. Important otherwise, some other
%maxspeed may be set from previous motion.
device.waitforidle();
absmaxvel = 206408;
device.set('maxspeed',absmaxvel);
device.waitforidle();

%Save maximum range of the linear rail.
maxrange = device.getrange();

%Mandatory initialisation homing sequence.
device.home();
device.waitforidle();

%Initialise target position and convert microsteps to mm.
targetposmm = 1;

%Conversion to mm, 1 microstep = 0.000047625mm.
steptomm = 0.000047625;

targetposmsteps = targetposmm/steptomm;
inc = targetposmsteps/100;

device.moverelative(targetposmsteps);
device.waitforidle();

device.home();
device.waitforidle();

%Set half speed and force threshold to stop on touch.
vel = absmaxvel*0.5;
device.set('maxspeed',vel);
fthreshold = 0.2;

%Start a scan for objects in the trajectory of motion. Stop on encountering
%an object and record position. If no object encountered by the end of maximum range, 
%home at full speed.

device.moverelative(maxrange(2));

while 1
    
    FT_Sensor(1) = GetFTsensorData(FT_Sensor(1));
    
    Fx = FT_Sensor(1).Data.ft(1);
    Fy = FT_Sensor(1).Data.ft(2);
    Fz = -FT_Sensor(1).Data.ft(3); 
    F = [Fx;Fy;Fz];
        
    Tx = FT_Sensor(1).Data.ft(4);
    Ty = FT_Sensor(1).Data.ft(5);
    Tz = FT_Sensor(1).Data.ft(6);
    T = [Tx;Ty;Tz];
    
    if Fz > fthreshold
        
        device.stop();
        stoppos = device.getposition();
        testfail1 = 0;
        break
        
    elseif device.getposition() == maxrange(2)
              
        device.waitforidle();
        device.set('maxspeed',absmaxvel);
        device.home();
        device.waitforidle();
        testfail1 = 1;
        break
        
    end
    
end
        
%If encountered an object, proceed with measuring sequence in the forward direction. 
%Set speed to 5% and move distance given by testdist while recording force
%readings. If not, abort.

pause(1.0);

%Move back to release preload.
preload = 0.3;
device.moverelative(-(preload/0.000047625));
device.waitforidle();

pause(1.0)

device.set('maxspeed',absmaxvel);
pause(1.0);

if testfail1 == 1
    
    disp('Nothing to measure. Test aborted.')
    fclose(port);
    delete(port);
    clear port;

    return
    
else
    
    pause(1);
    
    posarrayf = [];
    forcearrayf = [];
    posarrayb = [];
    forcearrayb = [];

    testdist = 10; %mm
    vel = round(absmaxvel*0.05);
    device.set('maxspeed',vel);

    %Measure and record the force data on the forward pass.
    %Constantly check for exceeeding the maximum allowed force on the
    %sensor, if exceeded, abort immediately and return to home position.
    
    for k = 1:repeats
    
        for i = 1:data_n

            FT_Sensor(1) = GetFTsensorData(FT_Sensor(1));

            Fx = FT_Sensor(1).Data.ft(1);
            Fy = FT_Sensor(1).Data.ft(2);
            Fz = -FT_Sensor(1).Data.ft(3); 
            F = [Fx;Fy;Fz];

            Tx = FT_Sensor(1).Data.ft(4);
            Ty = FT_Sensor(1).Data.ft(5);
            Tz = FT_Sensor(1).Data.ft(6);
            T = [Tx;Ty;Tz];

            if abs(Fx) < 50 && abs(Fy) < 50 && abs(Fz) < 25 && abs(Tx) < 0.5 && abs(Ty) < 0.5 && abs(Tz) < 0.5

                device.moverelative((testdist/data_n)/steptomm);  
                device.waitforidle();
                posarrayf(i) = (device.getposition() - stoppos + (preload/steptomm)) * steptomm;
                forcearrayf(i) = Fz;
                testfail2 = 0;

            else

                device.stop();
                device.waitforidle();
                device.set('maxspeed',absmaxvel);
                device.home();
                testfail2 = 1;
                break

            end

        end

        %Wait for the force sensor to settle down.

        pause(2.0);

        %Measure and record the force data on the backward pass.
        %Constantly check for exceeeding the maximum allowed force on the
        %sensor, if exceeded, abort immediately and return to home position.

        device.waitforidle();

        for j = 1:data_n

            FT_Sensor(1) = GetFTsensorData(FT_Sensor(1));

            Fx = FT_Sensor(1).Data.ft(1);
            Fy = FT_Sensor(1).Data.ft(2);
            Fz = -FT_Sensor(1).Data.ft(3); 
            F = [Fx;Fy;Fz];

            Tx = FT_Sensor(1).Data.ft(4);
            Ty = FT_Sensor(1).Data.ft(5);
            Tz = FT_Sensor(1).Data.ft(6);
            T = [Tx;Ty;Tz];

            if abs(Fx) < 50 && abs(Fy) < 50 && abs(Fz) < 70 && abs(Tx) < 0.5 && abs(Ty) < 0.5 && abs(Tz) < 0.5

            device.moverelative(-(testdist/data_n)/steptomm);  
            device.waitforidle();    
            posarrayb(j) = (device.getposition() - stoppos + (preload/steptomm)) * steptomm;
            forcearrayb(j) = Fz;
            testfail2 = 0;

            else

                device.stop();
                device.waitforidle();
                device.set('maxspeed',absmaxvel);
                device.home();
                testfail2 = 1;
                break

            end

        end
    
        device.set('maxspeed',absmaxvel);
        device.moverelative(-10/steptomm);
        device.waitforidle();
        pause(5.0);
        device.moverelative(10/steptomm);
        device.waitforidle();
        device.set('maxspeed',vel);
        device.waitforidle();
        
        Mff(k,:) = forcearrayf;
        Mfb(k,:) = forcearrayb;
        
    end
    
end

%If maximum allowed force is exceeded, inform user.

if testfail2 == 1
    
   disp('Maximum allowed force on sensor exceeded. Aborted.');
   fclose(port);
   delete(port);
   clear port;

   return
   
end

device.waitforidle();
pause(2.0);

%Restore maximum speed.

device.set('maxspeed',absmaxvel);

catch exception
    fclose(port);
    rethrow(exception);
    
end

%Test finished successfully, return to home position.

device.home();
device.waitforidle();
    
%Disconnect, delete and clear ports; otherwise the port remains hanging and
%prevents connecting subsequently.

fclose(port);
delete(port);
clear port;

forcearrayavef = mean(Mff);
forcearraystdf = std(Mff)/sqrt(length(forcearrayavef));

forcearrayaveb = mean(Mfb);
forcearraystdb = std(Mfb)/sqrt(length(forcearrayaveb));

%Plot recorded data.

fill([posarrayf;flipud(posarrayf)],[forcearrayavef - (2*forcearraystdf);flipud(forcearrayavef + (forcearraystdf*2))],[.9 .9 .9],'linestyle','none');
line(posarrayf,forcearrayavef);
hold on
fill([posarrayb;flipud(posarrayb)],[forcearrayaveb - (2*forcearraystdb);flipud(forcearrayaveb + (forcearraystdb*2))],[.9 .9 .9],'linestyle','none');
line(posarrayb,forcearrayaveb); 
xlabel('displacement [mm]')
ylabel('force [N]')
% xlim([0 3]);
% ylim([0 1]);

%f1 = fit(posarrayf',forcearrayf','rat33');
%f2 = fit(posarrayb',forcearrayb','rat33');

%plot(f1,posarrayf,forcearrayf);
%hold on
%plot(f2,posarrayb,forcearrayb);

