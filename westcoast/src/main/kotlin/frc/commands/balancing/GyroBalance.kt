package frc.commands.balancing

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.ADIS16448_IMU
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.DriveTrainSubsystem

class GyroBalance(
    private val forward: PIDController,
    private val turn: PIDController,
    private val drive: DriveTrainSubsystem,
    private val gyro: ADIS16448_IMU,
    private val desiredAngle: Double = 0.0
) : CommandBase() {
    var forwardPower = 0.0

    override fun initialize() {
        turn.reset()
        forward.reset()
        gyro.reset()
    }

    override fun execute() {
        forwardPower = forward.calculate(gyro.gyroAngleY, desiredAngle).coerceIn(-0.2865, 0.2865)
        drive.arcade(
            forwardPower,
            turn.calculate(gyro.gyroAngleX, desiredAngle).coerceIn(0.0, 0.0),
            // I don't think this currently aligns to straight?
            false
        )
    }

    override fun end(interrupted: Boolean) {
        drive.stop()
    }
}