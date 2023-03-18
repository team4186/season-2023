package frc.commands.Intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.IntakeSubsystem

class Intake(
    private val intake: IntakeSubsystem
) : CommandBase() {
    override fun execute() {
        with(intake) {
            if (intakeLimit.get()) {
                setIntakeMotors(0.35)
            }
            else
            {
                intakeMotors.stopMotor()
            }
        }
    }

    override fun end(interrupted: Boolean) {
        intake.stop()
    }
}