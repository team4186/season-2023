package frc.commands.balancing

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.ADIS16448_IMU
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.DriveTrainSubsystem

class GyroBalance (
        private val forward: PIDController,
        private val turn: PIDController,
        private val drive: DriveTrainSubsystem
) : CommandBase() {
    private val gyro = ADIS16448_IMU()
    private val desiredAngle = 0.0

    override fun initialize() {
        turn.reset()
        forward.reset()
    }

    override fun execute() {
        drive.arcade(
            forward.calculate(gyro.gyroAngleY, desiredAngle.coerceIn(-1.0, 1.0)),
            turn.calculate(gyro.gyroAngleX, desiredAngle.coerceIn(-1.0, 1.0)),
                // I don't think this currently aligns to straight?
            false
        )
    }

    override fun end(interrupted: Boolean) {
        drive.stop()
    }

    override fun isFinished(): Boolean {
        return gyro.gyroAngleY > 1.0 || gyro.gyroAngleY < -1.0
    }
}