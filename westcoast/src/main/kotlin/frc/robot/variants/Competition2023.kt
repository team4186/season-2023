package frc.robot.variants

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.Joystick
import frc.hardware.DummyGyro
import frc.robot.definition.*
import frc.robot.definition.Motors.Companion.driveCTRMotors
import frc.robot.definition.Motors.Companion.driveSparkMaxMotors
import frc.vision.LimelightRunner

val Competition2023 = Definition(
    name = "Shin Destroyer",
    input = Input(
        joystick = Joystick(0)
    ),
    motors = Motors(
        driveLeft = driveSparkMaxMotors(
            lead = CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless),
            follower0 = CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless),
            invert = false,
        ),
        driveRight = driveSparkMaxMotors(
            lead = CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless),
            follower0 = CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless),
            invert = true,
        ),
        hDrive = driveSparkMaxMotors(
            lead = CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless),
            follower0 = CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless),
            invert = false,
        )
    ),
    sensors = Sensors(
        drive = Sensors.DriveSensors(
            gyro = DummyGyro,
            leftEncoder = encoder(8, 9),
            rightEncoder = encoder(7, 6),
            vision = LimelightRunner()
        ),
    ),
    controllers = Controllers(
        leaveLine = {
            ProfiledPIDController(
                1.0,
                0.0,
                0.4,
                TrapezoidProfile.Constraints(3.0, 4.0)
            ).apply {
                setTolerance(0.10, 0.1)
                disableContinuousInput()
            }
        },
    ),
)

private fun encoder(channelA: Int, channelB: Int): Encoder {
    val encoder = Encoder(channelA, channelB)
    encoder.distancePerPulse = 0.0018868 // 530 pulses = 1 m
    return encoder
}