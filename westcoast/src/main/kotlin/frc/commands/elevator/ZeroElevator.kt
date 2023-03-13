package frc.commands.elevator

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.ElevatorSubsystem

class ZeroElevator(
    private val elevator: ElevatorSubsystem
) : CommandBase() {

    override fun execute() {
        with(elevator) {
            if (!carriageLimitBottom.get() && (carriageMotor.busVoltage * carriageMotor.appliedOutput < 10.0)) {
                setCarriageMotor(0.1)
            } else {
                carriageMotor.stopMotor()
                carriageMotor.encoder.position = 0.0
            }

            if (!stageLimitBottom.get() && (stageTwoMotor.busVoltage * stageTwoMotor.appliedOutput < 10.0)) {
                setStageTwo(0.1)
            } else {
                stageTwoMotor.stopMotor()
                stageTwoMotor.encoder.position = 0.0
            }

            if (!wristLimitTop.get() && (wristMotor.busVoltage * wristMotor.appliedOutput < 10.0)) {
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