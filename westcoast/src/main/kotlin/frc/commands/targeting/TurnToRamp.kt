package frc.commands.targeting

import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.DriveTrainSubsystem
import kotlin.math.roundToInt

class TurnToRamp(
    private val drive: DriveTrainSubsystem,
    private val turn: PIDController,
    private val gyro: Pigeon2,
    private val angle: Double
) : CommandBase() {
    private var turnPower = 0.0
    private var wait = 0

    override fun execute() {
        turnPower =
            turn.calculate(gyro.yaw - ((gyro.yaw/360.0).roundToInt() * 360.0) , angle)
                .coerceIn(-0.15, 0.15)

        drive.turnOnly(
            turnPower
        )
        if ( gyro.yaw >= angle - 1 && gyro.yaw <= angle + 1) {
            wait++ //wait = wait + 1
        }


    }

    override fun isFinished(): Boolean {
        return wait >= 10
    }

    override fun end(interrupted: Boolean) {
        drive.stop()
    }
}