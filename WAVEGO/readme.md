**NOTICE: the robot will continuously send arrays of IMU pose data via the Serial.**

**New Features:**

1. **Modifed WAVEGO.ino**
1.1 defined `sendIMU2Rpi()` as a task function to send a group of pose elements through UART;
1.2 defined `calculatePose()` function as a task function to calculate the pose elements from the raw IMU data;
1.3 added magnetometer operation mode selection in the `setup()` function;
1.4 relative global variables have been added.

2. **In InitConfig.h**
2.1 added IMU Magnetometer initialization;
2.2 modified `accXYZUpdate()` to enable input arguments for choosing between default pitch, roll calculation algorithms;
2.3 new global parameters has been defined for IMU data storage.
