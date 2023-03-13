package frc.commands.elevator

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.subsystems.ElevatorSubsystem

class MoveWrist(
    private val elevator: ElevatorSubsystem,
    position: Double
) : PIDCommand(
    PIDController(0.05, 0.0, 0.0),
    { elevator.wristMotor.encoder.position },
    position,
    { speed ->
        if (elevator.wristLimitTop.get() && elevator.wristLimitBottom.get()) {
            elevator.wristMotor.stopMotor()
        } else {
            elevator.setWristMotor(speed)
        }
    }
){
    override fun end(interrupted: Boolean) {
        elevator.stopAll()
    }
}