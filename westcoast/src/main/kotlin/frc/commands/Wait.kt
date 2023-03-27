package frc.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.CommandBase

class Wait(
    private val time: Double,
) : CommandBase() {
    var wait = 0.0

    override fun execute() {
        wait = wait + 0.02
    }

    override fun isFinished(): Boolean {
        return wait > time
    }

    override fun end(interrupted: Boolean) {
        return
    }
}