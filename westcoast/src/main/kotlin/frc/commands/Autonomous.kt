package frc.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.commands.drive.LeaveLine
import frc.robot.definition.Definition

object Autonomous {
    fun Definition.move(distance: Double): Command {
        return LeaveLine(
            distance,
            controllers.leaveLine(),
            controllers.leaveLine(),
            subsystems.driveTrain
        )
    }
}