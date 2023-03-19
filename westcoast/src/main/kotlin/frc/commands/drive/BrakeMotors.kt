package frc.commands.drive

import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.DriveTrainSubsystem

class BrakeMotors(
    private val drive: DriveTrainSubsystem
) : CommandBase() {
    init {
        addRequirements(drive)
    }

    override fun execute() {
        drive.leftMotor.idleMode = CANSparkMax.IdleMode.kBrake
        drive.rightMotor.idleMode = CANSparkMax.IdleMode.kBrake
    }

    override fun end(interrupted: Boolean) {
        drive.leftMotor.idleMode = CANSparkMax.IdleMode.kCoast
        drive.rightMotor.idleMode = CANSparkMax.IdleMode.kCoast
    }
}