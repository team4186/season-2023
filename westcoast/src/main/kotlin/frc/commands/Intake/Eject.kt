package frc.commands.Intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.IntakeSubsystem

class Eject(
    private val intake: IntakeSubsystem
) : CommandBase() {
    override fun execute() {
        with(intake) {
            setIntakeMotors(-0.35)
        }
    }

    override fun end(interrupted: Boolean) {
        intake.stop()
    }
}