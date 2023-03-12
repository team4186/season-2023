package frc.commands.elevator

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.ElevatorSubsystem

class ZeroElevator(
    private val elevator: ElevatorSubsystem
) : CommandBase() {

    override fun execute() {
        with(elevator) {
            if (!carriageLimitBottom.get()) {
                setCarriageMotor(-0.5)
            } else {
                carriageMotor.stopMotor()
                carriageMotor.encoder.position = 0.0
            }
            if (!stageLimitBottom.get()) {
                setStageTwo(-0.5)
            } else {
                stageTwoMotor.stopMotor()
                stageTwoMotor.encoder.position = 0.0
            }
            if (!wristLimitTop.get()) {
                setWristMotor(0.5)
                wristMotor.encoder.position = 0.0
            }
        }
    }
}