package frc.commands.balancing

import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.DriveTrainSubsystem

class GyroBalance(
    private val forward: PIDController,
    private val turn: PIDController,
    private val drive: DriveTrainSubsystem,
    private val gyro: Pigeon2,
    private val desiredAngle: Double = 0.0
) : CommandBase() {
    var forwardPower = 0.0

    override fun initialize() {
        turn.reset()
        forward.reset()
        //no reset for pigeon?
    }

    //pid definitions are in Robot.kt
    override fun execute() {
        forwardPower = forward.calculate(gyro.pitch, desiredAngle).coerceIn(-0.2865, 0.2865)
        drive.arcade(
            forwardPower,
            turn.calculate(-gyro.yaw, desiredAngle).coerceIn(-0.2, -0.2), // if this is going the wrong direction swap signs
            //yaw (z axis) should align robot to straight
            false
        )

    }

    override fun end(interrupted: Boolean) {
        drive.stop()
    }
}