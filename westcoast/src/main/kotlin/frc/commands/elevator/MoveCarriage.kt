package frc.commands.elevator

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.subsystems.ElevatorSubsystem

class MoveCarriage(
    private val elevator: ElevatorSubsystem,
    position: Double,
) : PIDCommand(
    PIDController(0.008, 0.001, 0.0),
    { elevator.carriageMotor.encoder.position },
    position,
    { velocity ->
        if(elevator.carriageLimitBottom.get()  && velocity < 0) {
            elevator.carriageMotor.stopMotor()
            elevator.carriageMotor.encoder.position = 0.0
        } else if(elevator.carriageLimitTop.get() && !elevator.stageLimitBottom.get() &&  velocity > 0) {
            elevator.carriageMotor.stopMotor()
        } else {
            elevator.setCarriageMotor(velocity)
            SmartDashboard.putNumber("CarriageVelocity", velocity)
            SmartDashboard.putNumber("CarriagePosition", elevator.carriageMotor.encoder.position)
        }
//        when {
//            velocity > 0 &&
//                elevator.carriageLimitTop.get() -> elevator.carriageMotor.stopMotor()
//
//            velocity < 0 &&
//                elevator.carriageLimitBottom.get() -> elevator.carriageMotor.stopMotor()
//
//            else -> elevator.setCarriageMotor(0.0)
//        }

   }
) {
    override fun end(interrupted: Boolean) {
        elevator.stopAll()
    }
}