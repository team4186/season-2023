package frc.commands.Intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.IntakeSubsystem

class Eject(
    private val intake: IntakeSubsystem,
    private val speed: Double
) : CommandBase() {
    override fun execute() {
        with(intake) {
            setIntakeMotors(-speed) // -0.5 prior
        }
    }

    override fun end(interrupted: Boolean) {
        intake.stop()
    }
}