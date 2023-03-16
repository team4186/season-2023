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
    { velocity ->
        when {
            velocity > 0 &&
                elevator.wristLimitTop.get() -> elevator.carriageMotor.stopMotor()

            velocity < 0 &&
                elevator.wristLimitBottom.get() -> elevator.carriageMotor.stopMotor()

            else -> elevator.setWristMotor(velocity)
        }
    }
){
    override fun end(interrupted: Boolean) {
        elevator.stopAll()
    }
}