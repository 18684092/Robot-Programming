# Robot Programming

## Scripts
- **sensor_srv.py**
- **sensor_pub.py**
- **listener.py**

## Examples

The publisher sensors_pub.py can publish all memory fields, 1st example or a single memory field, 2nd example. The listener will see a string value for the field or a serialised dictionary of all fields.

```$ rostopic echo /all_mem```

data: "{\"available\": 13586079744, \"used\": 2278055936, \"free\": 12225290240, \"inactive\"\
  : 810291200, \"active\": 2909175808, \"total\": 16387588096, \"slab\": 255102976,\
  \ \"buffers\": 167620608, \"cached\": 1716621312, \"percent\": 17.1, \"shared\"\
  : 186216448}"

```$ rostopic echo /free_mem```

data: "12177854464"




The sensor_srv.py script provides a ROS service that responds with the host's memory details. It also will reconnect to ROS master should master go down and then come back-up again. The specific memory field can be requested or all fields. The response is a serialised dictionary 

```$ rosservice call /get_memory free```

response: "{\"epoch\": 1663869477, \"free\": 11780108288}"

```$ rosservice call /get_memory available```

response: "{\"available\": 12972105728, \"epoch\": 1663869484}"

```$ rosservice call /get_memory all```

response: "{\"available\": 12956168192, \"used\": 2882674688, \"free\": 11757998080, \"inactive\": 835198976,  \"active\": 3317071872, \"total\": 16387588096, \"slab\": 234094592, \"buffers\": 143945728, \"cached\": 1602969600, \"percent\": 20.9, \"epoch\": 1663869494, \"shared\": 228298752}"

- **Running the script and log output**

The publisher / service scripts can recover from roscore going down and coming back up.

```$ /usr/bin/python /home/andy/catkin_ws/src/andy_msc/scripts/sensor_srv.py```

```
[INFO] [1663869420.587280]: /sensor_srv initiated
[INFO] [1663869477.930538]: /sensor_srv: get_memory_details called with service: free
[INFO] [1663869484.764102]: /sensor_srv: get_memory_details called with service: available
[INFO] [1663869494.098081]: /sensor_srv: get_memory_details called with service: All
22.09.2022-18:59:13: /sensor_srv - ROS master went offine
[WARN] [1663869553.588671]: unable to communicate with ROS Master, registrations are now out of sync
22.09.2022-18:59:17: /sensor_srv - ROS master is online
```


## Services
- Memory.srv
