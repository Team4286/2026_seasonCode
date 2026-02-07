# 2026_seasonCode
2026 Rebuilt seasons code for team 4286, Imperial Robotics.


The Subsystem file controls the base structure of the swereve drive: Based off the Rev-Swerve java template
The Drive AccelerationLimiter is a file designed to smooth the acceleration rate of the wheels of the swereve from going 0-100.
There is a 0.5 curve in acceleration to prevent wheel damage. 

SubFuelLaunch will hold all basic files for the fuel launch cannon.
This will include:
    A physics calculator: estimate what value is needed for a cannon
    Lookup table: stores tested values
    A file that builds the cannon-> fly wheel design
    
No indexer is currently designed.

Intake needs two motors.