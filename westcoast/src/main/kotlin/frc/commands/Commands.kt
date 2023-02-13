package frc.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.commands.drive.CheesyDrive
import frc.commands.drive.TeleopDrive
import frc.robot.definition.Definition

object Commands {
    object DriveCommands {
        fun Definition.raw(): TeleopDrive {
            return TeleopDrive(
                inputThrottle = { input.joystick.y },
                inputTurn = { input.joystick.twist },
                inputYaw = { input.joystick.x },
                drive = subsystems.driveTrain
            )
        }

        fun Definition.cheesy(): Command {
            return CheesyDrive(
                inputThrottle = { input.joystick.y },
                inputYaw = { input.joystick.x },
                drive = subsystems.driveTrain,
                sensitivityHigh = 0.5,
                sensitivityLow = 0.5
            )
        }
    }

    object AutonomousDriveCommands {
    }
}