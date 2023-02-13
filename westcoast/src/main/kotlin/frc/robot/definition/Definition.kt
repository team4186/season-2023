package frc.robot.definition

import frc.subsystems.DriveTrainSubsystem

data class Definition(
    val name: String,
    val input: Input,
    val motors: Motors,
    val sensors: Sensors,
    val controllers: Controllers,
    val subsystems: Subsystems = Subsystems(
        driveTrain = DriveTrainSubsystem(
            leftMotor = motors.driveLeft.lead,
            rightMotor = motors.driveRight.lead,
            hMotor = motors.hDrive.lead,
            leftEncoder = sensors.drive.leftEncoder,
            rightEncoder = sensors.drive.rightEncoder,
            gyro = sensors.drive.gyro,
            vision = sensors.drive.vision,
        ),
    )
)