package frc.commands.elevator

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.subsystems.ElevatorSubsystem

class MoveStageTwo(
    private val elevator: ElevatorSubsystem,
    position: Double
) : PIDCommand(
    PIDController(0.05, 0.0, 0.0),
    { elevator.stageTwoMotor.encoder.position },
    position,
    { speed ->
        elevator.setStageTwo(speed)
    }
) {
    override fun end(interrupted: Boolean) {
        elevator.stopAll()
    }
}