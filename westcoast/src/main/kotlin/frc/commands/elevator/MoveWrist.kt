package frc.commands.elevator

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.subsystems.ElevatorSubsystem
import frc.subsystems.WRIST_END

/**
 * when MoveWrist is called, motor power is constricted to 35%
 * if carriage is extended (NOT stageLimitBottom), stop motor
 * if velocity > 0 (moving UP) and hit wristLimitTop (true), stop motor
 * if velocity < 0 (moving DOWN) and hit wristLimitBottom (true), stop motor
 */
class MoveWrist(
    private val elevator: ElevatorSubsystem,
    position: Double
) : PIDCommand(
    PIDController(0.07, 0.01, 0.0),
    { elevator.wristMotor.encoder.position },
    position,
    { velocity ->
        when {
            !elevator.stageLimitBottom.get() ->
                elevator.wristMotor.stopMotor()

            velocity > 0 && elevator.wristLimitTop.get() -> {
                elevator.wristMotor.stopMotor()
                elevator.wristMotor.encoder.position = 0.0
            }

            velocity < 0 && elevator.wristLimitBottom.get() -> {
                elevator.wristMotor.stopMotor()
                elevator.wristMotor.encoder.position = WRIST_END
            }

            else -> elevator.setWristMotor(velocity.coerceIn(-0.35, 0.35))
        }
    }
) {
    override fun end(interrupted: Boolean) {
        elevator.stopAll()
    }
}