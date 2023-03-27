package frc.commands.balancing

import com.ctre.phoenix.sensors.Pigeon2
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.CAN
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.DriveTrainSubsystem
import frc.subsystems.driveSparkMaxMotors
import java.util.function.DoubleSupplier

class GyroBalance(
    private val forward: PIDController,
    private val turn: PIDController,
    private val drive: DriveTrainSubsystem,
    private val gyro: Pigeon2,
    private val desiredAngle: Double = 0.0
) : CommandBase() {
    var forwardPower = 0.0
    var turnPower = 0.0
    private var wait = 0

    override fun initialize() {
        forwardPower = 0.0
        turnPower = 0.0
        wait = 0
    }

    //pid definitions are in Robot.kt
    override fun execute() {
        forwardPower = 0.35 * forward.calculate(gyro.pitch, desiredAngle)
        turnPower = 0.35 * turn.calculate(gyro.yaw, desiredAngle)
        //tune that!!!!!!!!!!?????????!?!??!? ^^^
        drive.arcade(
            -forwardPower,
            turnPower,
            false
        )

        if (gyro.pitch < 2.5 && gyro.pitch > -2.5) {
            wait++
        }
    }

//    override fun isFinished(): Boolean {
//        return wait > 20
//    }

    override fun end(interrupted: Boolean) {
        drive.stop()
        drive.rightMotor.idleMode = CANSparkMax.IdleMode.kBrake
        drive.leftMotor.idleMode = CANSparkMax.IdleMode.kBrake
    }
}