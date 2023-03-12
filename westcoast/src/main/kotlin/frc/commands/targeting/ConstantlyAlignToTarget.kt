package frc.commands.targeting

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.DriveTrainSubsystem
import frc.vision.VisionRunner

class ConstantlyAlignToTarget(
    private val turn: PIDController,
    private val forward: PIDController,
    private val drive: DriveTrainSubsystem,
    private val vision: VisionRunner
) : CommandBase() {
    override fun initialize() {
        turn.reset()
        forward.reset()
    }

    override fun execute() {
        drive.arcade(
            -forward.calculate(vision.distance, 5.0).coerceIn(-0.4, 0.4),
            turn.calculate(vision.xOffset, 0.0).coerceIn(-0.4, 0.4),
            false
        )
    }

    override fun end(interrupted: Boolean) {
        drive.stop()
    }
}