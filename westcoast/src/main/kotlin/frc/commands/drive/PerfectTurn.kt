package frc.commands.drive

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.DriveTrainSubsystem

// THIS CLASS IS UNTUNED, PLEASE DO NOT USE IT UNTIL TUNED
class PerfectTurn(
    private val angle: Double,
    private val angleMultiplier: Double,
    private val left: ProfiledPIDController,
    private val right: ProfiledPIDController,
    private val drive: DriveTrainSubsystem,
    private val rightEncoder: Encoder,
    private val leftEncoder: Encoder
) : CommandBase() {
    private var wait = 0


    override fun initialize() {
        wait = 0
        rightEncoder.reset()
        leftEncoder.reset()
        right.reset(0.0, 0.0)
        left.reset(0.0, 0.0)
    }

    override fun execute() {
        val target = angle * angleMultiplier
        drive.tank(
            left = left.calculate(-leftEncoder.distance, target).coerceIn(-0.4, 0.4),
            right = right.calculate(-rightEncoder.distance, -target).coerceIn(-0.4, 0.4),
            squareInputs = false
        )
        wait = if (right.atGoal() && left.atGoal()) wait + 1 else 0
    }

    override fun end(interrupted: Boolean) {
        drive.stop()
    }

    override fun isFinished(): Boolean {
        return wait >= 10
    }
}