package frc.commands.elevator

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.subsystems.ElevatorSubsystem

class MoveStageTwo(
    private val elevator: ElevatorSubsystem,
    position: Double
) : PIDCommand(
    PIDController(0.012, 0.0, 0.0),
    { elevator.stageTwoMotor.encoder.position },
    position,
    { velocity ->
        if(elevator.stageLimitBottom.get() && velocity < 0) {
            elevator.stageTwoMotor.stopMotor()
        } else if(elevator.stageLimitTop.get() && velocity > 0) {
            elevator.stageTwoMotor.stopMotor()
        } else {
            elevator.setStageTwo(0.5)
        }

//        when (true) {
//            (velocity < -0.5 &&
//                elevator.stageLimitTop.get()) -> elevator.stageTwoMotor.stopMotor()
//
//           (
//                elevator.stageLimitBottom.get() )->{
//                    elevator.stageTwoMotor.stopMotor()
//                    SmartDashboard.putBoolean("StopBottomStageTwo", true)
//                }
//
//            else -> {
//                SmartDashboard.putNumber("StageTwoVel", velocity);
//                elevator.setStageTwo(velocity/6.0)
//            }
//        }
//

    }
) {
    override fun end(interrupted: Boolean) {
        elevator.stopAll()
    }
}