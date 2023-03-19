package frc.commands.balancing

import com.ctre.phoenix.sensors.Pigeon2
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.DriveTrainSubsystem
import java.util.function.DoubleSupplier

class GyroBalance(
    private val forward: PIDController,
    private val turn: PIDController,
    private val drive: DriveTrainSubsystem,
    private val gyro: Pigeon2,
    private val gyroCompassStartPos: DoubleSupplier,
    private val desiredAngle: Double = 0.0
) : CommandBase() {
    var forwardPower = 0.0
    var wait = 0

    override fun initialize() {
//        turn.reset()
//        forward.reset()
        drive.leftMotor.idleMode = CANSparkMax.IdleMode.kBrake
        drive.rightMotor.idleMode = CANSparkMax.IdleMode.kBrake
    }

    //pid definitions are in Robot.kt
    override fun execute() {
        forwardPower = forward.calculate(gyro.pitch, desiredAngle).coerceIn(-0.2865, 0.2865)
        //tune that!!!!!!!!!!?????????!?!??!? ^^^
        drive.arcade(
            forwardPower,
            turn.calculate(gyroCompassStartPos.asDouble, desiredAngle).coerceIn(-0.2, -0.2), // if this is going the wrong direction swap signs
            //yaw (z axis) should align robot to straight
            false
        )
        if (gyro.pitch < 2.5 && gyro.pitch > -2.5){
            wait++
        }
    }

    override fun isFinished(): Boolean {
        return wait > 20
    }

    override fun end(interrupted: Boolean) {
        drive.stop()
    }
}