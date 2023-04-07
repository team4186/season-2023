package frc.commands.drive

import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.DriveTrainSubsystem
import kotlin.math.roundToInt

class LockRotation(
    private val turn: PIDController,
    private val drive: DriveTrainSubsystem,
    private val gyro: Pigeon2,
    private val pos: Double
) : CommandBase() {
    private var turnPower = 0.0

    override fun execute() {
        turnPower =
            turn.calculate(gyro.yaw, pos)
                .coerceIn(-0.15, 0.15)

        drive.holonomic(
            0.0,
            turnPower,
            0.0,
            false
        )
    }

    override fun end(interrupted: Boolean) {
        turnPower = 0.0
        drive.stop()
    }
}