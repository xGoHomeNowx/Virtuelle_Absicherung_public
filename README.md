Longitudinal and lateral control of a virtual vehicle in Tronis (based on UE4). Logic in C++

Copy the Tronis blueprint as text and paste it into the blueprint editor, then create the following components:

SensorPoseVelocity, SensorBoundBox, SensorCamera, TronisSocket

TronisSocket as a Tronis TCP socket with IP: 127.0.0.1 and port: 7778 (socket timeout may be set to 500ms)
