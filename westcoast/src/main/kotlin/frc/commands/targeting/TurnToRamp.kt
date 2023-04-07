package frc.commands.targeting

import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.subsystems.DriveTrainSubsystem

class TurnToRamp(
    private val turn: PIDController,
    private val drive: DriveTrainSubsystem,
    private val gyro: Pigeon2
) : CommandBase() {

}