package frc.commands.drive

import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.DriveTrainSubsystem
import java.util.function.DoubleSupplier
import kotlin.math.roundToInt

class LeaveLine(
    private val distance: Double,
    private val forward: PIDController,
    private val turn: PIDController,
    private val drive: DriveTrainSubsystem,
    private val rightEncoder: DoubleSupplier,
    private val leftEncoder: DoubleSupplier,
    private val gyro: Pigeon2,
) : CommandBase() {
    private var wait = 0

    override fun initialize() {
    }

    override fun execute() {
        var forwardPower =
                forward.calculate((leftEncoder.asDouble + rightEncoder.asDouble), distance)
                .coerceIn(-0.5, 0.5)
        var turnPower =
                turn.calculate(gyro.yaw - ((gyro.yaw / 180).roundToInt() * 180), 0.0)
                .coerceIn(-0.15, 0.15)

        drive.holonomic(
            forwardPower,
            turnPower,
            0.0,
            false
        )

        if (leftEncoder.asDouble - distance < 2 &&
            leftEncoder.asDouble - distance > -2 ||
            rightEncoder.asDouble - distance < 2 &&
            leftEncoder.asDouble - distance > -2
            ){
            wait++
        }
    }

    override fun end(interrupted: Boolean) {
        wait = 0
        drive.stop()
    }

    override fun isFinished(): Boolean {
        return wait >= 10
    }
}