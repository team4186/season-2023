package frc.commands.drive

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.DriveTrainSubsystem
import java.util.function.DoubleSupplier

class LeaveLine(
    private val distance: Double,
    private val left: PIDController,
    private val right: PIDController,
    private val drive: DriveTrainSubsystem,
    private val rightEncoder: DoubleSupplier,
    private val leftEncoder: DoubleSupplier,
) : CommandBase() {

    private var wait = 0

    override fun initialize() {
        wait = 0
        right.reset()
        left.reset()
    }

    override fun execute() {
        val rightOut = right.calculate(rightEncoder.asDouble, distance).coerceIn(-0.4, 0.4)
        val leftOut = left.calculate(leftEncoder.asDouble, distance).coerceIn(-0.4, 0.4)
        drive.setMotorOutput(leftOut, rightOut)
        wait = if (right.atSetpoint() && left.atSetpoint()) wait + 1 else 0
    }

    override fun end(interrupted: Boolean) {
        drive.stop()
    }

    override fun isFinished(): Boolean {
        return wait >= 10
    }
}