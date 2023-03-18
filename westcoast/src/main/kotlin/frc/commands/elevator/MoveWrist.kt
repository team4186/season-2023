package frc.commands.elevator

import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.subsystems.ElevatorSubsystem
import frc.subsystems.WRIST_END

class MoveWrist(
    private val elevator: ElevatorSubsystem,
    position: Double
) : PIDCommand(
    PIDController(0.07, 0.01, 0.0),
    { elevator.wristMotor.encoder.position },
    position,
    { velocity ->
        when {
            velocity > 0 && !elevator.stageLimitBottom.get() && elevator.wristLimitTop.get() ->
            {
                elevator.wristMotor.stopMotor()
                elevator.wristMotor.encoder.position = 0.0
            }

            velocity < 0 && !elevator.stageLimitBottom.get() && elevator.wristLimitBottom.get() ->
            {
                elevator.wristMotor.stopMotor()
                elevator.wristMotor.encoder.position = WRIST_END
            }

            else -> elevator.setWristMotor(velocity.coerceIn(-0.35, 0.35))
        }
    }
){
    override fun end(interrupted: Boolean) {
        elevator.stopAll()
    }
}