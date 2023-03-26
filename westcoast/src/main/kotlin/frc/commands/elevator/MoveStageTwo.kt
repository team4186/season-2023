package frc.commands.elevator

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.subsystems.ElevatorSubsystem

class MoveStageTwo(
    private val elevator: ElevatorSubsystem,
    position: Double
) : PIDCommand(
    PIDController(0.05, 0.003, 0.0),
    { elevator.stageTwoMotor.encoder.position },
    position,
    { velocity ->
        if(!elevator.carriageLimitTop.get() || !elevator.wristLimitTop.get()){
            elevator.stageTwoMotor.stopMotor()
        } else if (velocity < 0) {
            if(elevator.stageLimitBottom.get()) {
                elevator.stageTwoMotor.stopMotor()
            } else {
                elevator.setStageTwo(velocity.coerceIn(-0.70, 0.8))
            }
        } else {
            if(elevator.stageLimitTop.get()){
                elevator.stageTwoMotor.stopMotor()
            } else {
                elevator.setStageTwo(velocity.coerceIn(-0.45, 0.45))
            }
        }
    }
) {
    override fun isFinished(): Boolean {
        return !elevator.wristLimitTop.get() || !elevator.carriageLimitTop.get()
    }

    override fun end(interrupted: Boolean) {
        elevator.stopAll()
    }
}