package frc.commands.targeting

import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.DriveTrainSubsystem
import frc.vision.VisionRunner
import java.time.ZoneOffset
import java.util.function.DoubleSupplier

class AlignToTarget(
    private val forward: PIDController,
    private val turn: PIDController,
    private val strafe: PIDController,
    private val drive: DriveTrainSubsystem,
    private val vision: VisionRunner,
    private val offset: Double,
    private val gyro: Pigeon2,
    private val gyroCompassStartPos: DoubleSupplier
) : CommandBase() {
    private var wait = 0

    override fun initialize() {
        wait = 0
    }

    override fun execute() {
        drive.holonomic(
            forward.calculate(28.5 * vision.tagArea, 0.038),
            turn.calculate(gyro.absoluteCompassHeading, gyroCompassStartPos.asDouble),
            strafe.calculate(vision.xOffset + offset, 0.0),
            false
        )
        if (
            vision.distance > 26.0 && vision.distance < 31.0 &&
            vision.xOffset + offset < 2 && vision.xOffset + offset > -2 &&
            gyro.absoluteCompassHeading < gyroCompassStartPos.asDouble + 2 &&
            gyro.absoluteCompassHeading > gyroCompassStartPos.asDouble - 2
        ) {
            wait++
        }
    }

    override fun end(interrupted: Boolean) {
        drive.stop()
    }

    override fun isFinished(): Boolean {
        return wait > 10
    }
}