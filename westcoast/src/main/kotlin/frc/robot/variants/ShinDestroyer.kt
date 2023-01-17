package frc.robot.variants

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.Joystick
import frc.hardware.DummyGyro
import frc.robot.definition.*
import frc.robot.definition.Motors.Companion.driveCTRMotors
import frc.vision.LimelightRunner

val ShinDestroyer = Definition(
    name = "Shin Destroyer",
    input = Input(
        joystick = Joystick(0)
    ),
    motors = Motors(
        driveLeft = driveCTRMotors(
            lead = WPI_TalonSRX(2),
            follower0 = WPI_VictorSPX(3),
            follower1 = WPI_VictorSPX(1),
            invert = true
        ),
        driveRight = driveCTRMotors(
            lead = WPI_TalonSRX(14),
            follower0 = WPI_VictorSPX(15),
            follower1 = WPI_VictorSPX(13),
            invert = false
        ),
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