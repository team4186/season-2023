package frc.commands.targeting

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.DriveTrainSubsystem
import frc.vision.VisionRunner

class TurnToTarget(
    private val turn: PIDController,
    private val drive: DriveTrainSubsystem,
    private val vision: VisionRunner
) : CommandBase() {
    private var wait = 0

    override fun initialize() {
        wait = 0
    }

    override fun execute() {
        drive.holonomic(
            0.0,
            turn.calculate(vision.xOffset, 0.0),
            0.0,
            false
        )
        if ( vision.xOffset < 1 && vision.xOffset > -1){
            wait++
        }
    }

    override fun end(interrupted: Boolean) {
        drive.stop()
    }

    override fun isFinished(): Boolean {
        return wait > 20 || !vision.hasTarget
    }
}