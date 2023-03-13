package frc.commands.targeting

import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.subsystems.DriveTrainSubsystem
import frc.vision.VisionRunner
import java.util.function.DoubleSupplier

class AlignToTarget(
    controller: PIDController,
    private val drive: DriveTrainSubsystem,
    private val vision: VisionRunner,
    private val gyro: Pigeon2,
    private val gyroCompassStartPos: DoubleSupplier
) : CommandBase() {

    private var wait = 0

    override fun initialize() {
        wait = 0
    }

    override fun execute() {

    }

    override fun end(interrupted: Boolean) {
        drive.stop()
    }

    override fun isFinished(): Boolean {
        return wait > 10 && vision.hasTarget
    }
}