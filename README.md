# 2023-Season

This is the Kettering Win committ in Manual Merge and Master.   This is branched from Master.
The changes in ArmPro will replace the Arm and Intake.
This changes the interface calls to use PhoenixPro instead of Phoenix v5.

The point is so that the Rotate CANcoder on rotate shaft is used as selected encoder for the rotate motor instead of the built-in coder in the motor.
The limit switch for rotate can be removed.
Motion Magic is used for the rotate motor.

simple VoltageControl (percentoutput) is still used on Intake motor.

The Arm Extend motor also uses Motion Magic.  
Whether the CANcode on the Extend spool will be used as the selected encoder for the extend motor is still an option by commented out.
