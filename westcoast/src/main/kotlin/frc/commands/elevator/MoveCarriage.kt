package frc.commands.elevator

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.subsystems.ElevatorSubsystem

class MoveCarriage(
    private val elevator: ElevatorSubsystem,
    private val position: Double
) : PIDCommand(
    PIDController(0.05, 0.0, 0.0),
    { elevator.carriageMotor.encoder.position },
    position,
    { speed ->
        if (elevator.carriageLimitTop.get() && elevator.carriageLimitBottom.get()) {
            elevator.carriageMotor.stopMotor()
        } else {
            elevator.setCarriageMotor(speed)
        }
    }
)