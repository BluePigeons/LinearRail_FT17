# LinearRail_FT17
The code for running the mechanical testing combined setup in the UCL Soft Haptics lab (https://helge-wurdemann.com/softhapticslab/) that includes the IIT FT-17 sensor and the Zaber X-LSM100A linear rail.

For more information, including the CAD models and instruction manuals, ask Helge for access to the Lab Documentation folder.

This can be used for either the full, combined setup, or just the individual components separately.

## IIT F/T-17 Sensor

### Setup

The sensor connects using UDP (User Datagram Protocol) which is a very minimal internet-style connection that contains no security (unlike TCP and other protocols used for communicating along the same channels but over the public internet that use security to stop randomers being able to access your device). This usefully allows a lot of information to be communicated quickly, but the lack of security coming through the same ports that could be used for general internet access makes it look suspicious to most security settings and by default it will be blocked by your laptop. Therefore it is important that you both change settings to allow it access but please be careful not to just make everything open and make your device vulnerable to all internet-style connections.

1. Connect the port on the sensor (the black box underneath) to your device via an ethernet cable.

2. Change the ethernet connection settings of your laptop to:
IP address (IPv4): 192.168.1.2
Subnet Mask: 255.255.0.0

In Windows, search for the "Ethernet" connection, and 

...Insert screenshot image here...

You may need to untick the IPv6 option for the ethernet connection so that it only uses the IPv4 with the address specified above.

Please note that the IP address is not the same as the one for the sensor itself, in instruction 3 below.

3. Make sure your firewall settings allow UDP connections to the sensor:
IP address: 192.168.1.1
Port number: 23
Subnet Mask: 255.255.0.0

How this is done will vary depending on your choice of security software. 
i. In general you need to find your "Firewall" settings page. 
ii. Then find the list of approved connections, this may look like the screenshot below:

...Insert screenshot image here...

iii. You need to add an additional approved connection, or more specifically a UDP connection.
iv. You will be asked to enter some or all of the sensor address information, although not necessarily in the order listed here.

...Insert screenshot image here...

Please note that if you are asked, you should only need to specify giving access to private, home networks, and not public networks unless you are using your code to connect to another device via another connection you have classed as public (such as over a public WiFi). 

4. Make sure the power supply is ON and the switch underneath is ON.

Please note that the switch underneath can be left ON but do not leave the power supply (at the socket) ON when the sensor is not in use because it increases the risk of accidental damage.

5. Go to address in 192.168.1.1 browser to see IIT Sensor demo page.
If you cannot connect then recheck security settings and steps 1 - 5 again.
If you can see demo page then proceed with running this program.

### Running the Program

1. Download/clone only the "IIT_FT_17_Sensor/" folder along with the "FT_Sensor_Alone.m".

2. Open the "FT_Sensor_Alone.m" file in Matlab and run.

## Zaber X-LSM100A Linear Rail

### Setup

1. Connect the USB on the rail to your device.

2. Switch on power! You should be able to see the green light.

### Running the Program

1. Download/clone only the ".m".

2. You can then either: 
- Download/clone the "Add-ons/" folder from within this repo, containing the Zaber Matlab Toolbox.
- Go to the official Zaber software website https://www.zaber.com/zaber-software and download the official latest Matlab toolbox directly.

Please note that there is no guarantee that the latest software will work with the rest of the code here but please feel free to update the rest of this code to enable that compatibility if you find that is the case so that others are saved the effort of doing so.



## Total Setup

To use the whole thing together.

1. Download/clone complete repo. You don't actually need the files of "FT_Sensor_alone.m" and ".m" but they may be useful for debugging the individual components separately.

2. Follow the IIT F/T-17 Sensor Setup instructions.

3. Follow the Zaber X-LSM100A Linear Rail instructions.

4. 

## Using With Additional Data



## Creating Useful Graphs



## Other Useful Information

Zaber X-LSM100A website https://www.zaber.com/products/linear-stages/X-LSM/details/X-LSM100A

This was created by Erin Nolan, building on code originally pieced together by Jan Petersand Samuel Suchal.

