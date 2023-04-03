package frc.commands.balancing

import com.ctre.phoenix.sensors.Pigeon2
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.DriveTrainSubsystem
import kotlin.math.roundToInt

class GyroBalance(
    private val forward: PIDController,
    private val turn: PIDController,
    private val drive: DriveTrainSubsystem,
    private val gyro: Pigeon2,
    private val desiredAngle: Double = 0.0
) : CommandBase() {
    var forwardPower = 0.0
    var turnPower = 0.0
    private var wait = 0

    override fun initialize() {
    }

    //pid definitions are in Robot.kt
    override fun execute() {
        forwardPower = -0.35 * forward.calculate(gyro.pitch, desiredAngle) // 0.35
        turnPower = turn.calculate(gyro.yaw - ((gyro.yaw / 180).roundToInt() * 180), desiredAngle).coerceIn(-0.1, 0.1)
        //tune that!!!!!!!!!!?????????!?!??!? ^^^
        drive.holonomic(
            forwardPower,
            turnPower,
            0.0,
            false
        )

        if (gyro.pitch < 2.5 && gyro.pitch > -2.5) {
            wait++
        }
    }

    override fun isFinished(): Boolean {
        return wait > 100
    }

    override fun end(interrupted: Boolean) {
        drive.stop()
        drive.rightMotor.idleMode = CANSparkMax.IdleMode.kBrake
        drive.leftMotor.idleMode = CANSparkMax.IdleMode.kBrake
        wait = 0
    }
}