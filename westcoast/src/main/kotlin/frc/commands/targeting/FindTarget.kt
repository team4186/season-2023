package frc.commands.targeting

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.DriveTrainSubsystem
import frc.vision.VisionRunner

class FindTarget(
    private val drive: DriveTrainSubsystem,
    private val vision: VisionRunner
) : CommandBase() {
    override fun execute() {
        drive.tank(-0.35, 0.35, false)
    }

    override fun isFinished(): Boolean {
        return vision.hasTarget
    }
}