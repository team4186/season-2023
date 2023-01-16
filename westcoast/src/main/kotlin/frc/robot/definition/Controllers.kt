package frc.robot.definition

import edu.wpi.first.math.controller.ProfiledPIDController

class Controllers(
    val leaveLine: () -> ProfiledPIDController,
)