package frc.robot.definition

import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.interfaces.Gyro
import frc.vision.VisionRunner

data class Sensors(
    val drive: DriveSensors
) {
    data class DriveSensors(
        val gyro: Gyro,
        val leftEncoder: Encoder,
        val rightEncoder: Encoder,
        val vision: VisionRunner
    )
}