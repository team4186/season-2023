package frc.commands.drive

import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.DriveTrainSubsystem
import java.util.function.DoubleSupplier
import kotlin.math.roundToInt

class LockRotation(
    private val turn: PIDController,
    private val drive: DriveTrainSubsystem,
    private val gyro: Pigeon2,
    private val pos: DoubleSupplier
) : CommandBase() {
    private var turnPower = 0.0
    private var startPos = 0.0

    override fun initialize() {
        startPos = pos.asDouble
    }

    override fun execute() {
        turnPower =
            turn.calculate(gyro.yaw, startPos)
                .coerceIn(-0.15, 0.15)

        drive.turnOnly(
            turnPower
        )
    }

    override fun end(interrupted: Boolean) {
        turnPower = 0.0
        drive.stop()
    }
}