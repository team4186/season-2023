package frc.commands.elevator

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.ElevatorSubsystem

class ZeroElevator(
    private val elevator: ElevatorSubsystem
) : CommandBase() {

    override fun execute() {
        with(elevator) {
            if (!carriageLimitBottom.get()) {
                setCarriageMotor(0.1)
            } else {
                carriageMotor.stopMotor()
                carriageMotor.encoder.position = 0.0
            }

            if (!stageLimitBottom.get()) {
                setStageTwo(0.1)
            } else {
                stageTwoMotor.stopMotor()
                stageTwoMotor.encoder.position = 0.0
            }

            if (!wristLimitTop.get()) {
                setWristMotor(0.1)
            } else {
                wristMotor.stopMotor()
                wristMotor.encoder.position = 0.0
            }
        }
    }

    override fun isFinished(): Boolean {
        return elevator.stageLimitBottom.get()
            && elevator.wristLimitTop.get()
            && elevator.carriageLimitBottom.get()
    }

    override fun end(interrupted: Boolean) {
        elevator.stopAll()
    }
}