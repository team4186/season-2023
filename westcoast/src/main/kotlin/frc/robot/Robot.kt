package frc.robot

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.ADIS16448_IMU
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.interfaces.Gyro
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.commands.Autonomous.move
import frc.commands.Commands.DriveCommands.cheesy
import frc.commands.Commands.DriveCommands.raw
import frc.robot.definition.Definition

class Robot(private val definition: Definition) : TimedRobot() {
    private enum class DriveMode {
        Raw,
        Cheesy
    }

    private val autonomousChooser = SendableChooser<Command>()
    private val driveModeChooser = SendableChooser<DriveMode>()

    override fun robotInit() {
        definition.subsystems.driveTrain.initialize()

        with(autonomousChooser) {
            addOption("Move 2 Meters", definition.move(2.0))
            SmartDashboard.putData("Autonomous Mode", this)
        }

        with(driveModeChooser) {
            setDefaultOption("Raw", DriveMode.Raw)
            addOption("Cheesy", DriveMode.Cheesy)
            SmartDashboard.putData("Drive Mode", this)
        }
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun autonomousInit() {
        val autonomous = autonomousChooser.selected
        autonomous?.schedule()
    }

    override fun autonomousPeriodic() {
    }

    override fun autonomousExit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun teleopInit() {

        when (driveModeChooser.selected) {
            DriveMode.Cheesy -> definition.cheesy()
            DriveMode.Raw -> definition.raw()
            else -> definition.raw()
        }.schedule()
    }

    private val gyro = ADIS16448_IMU()
    override fun teleopPeriodic() {
        SmartDashboard.putNumber("Angle", gyro.angle)
    }

    override fun teleopExit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testInit() {
    }
}