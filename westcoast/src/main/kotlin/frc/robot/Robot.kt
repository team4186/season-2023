package frc.robot

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.ADIS16448_IMU
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.commands.Autonomous.move
import frc.commands.Commands.DriveCommands.cheesy
import frc.commands.Commands.DriveCommands.raw
import frc.commands.balancing.GyroBalance
import frc.robot.definition.Definition
import frc.vision.LimelightRunner
import frc.vision.VisionRunner

const val MAX_SPEED_ENCODER = 2000
//joystick * ^^ = input to pids



class Robot(private val definition: Definition) : TimedRobot() {
    private enum class DriveMode {
        Raw,
        Cheesy
    }

    private val gyro = ADIS16448_IMU()
    private val leftEncoder = Encoder(0,1, true)
    private val rightEncoder = Encoder(2, 3)

    private val gyroBalance = GyroBalance(
        forward = PIDController(
            0.05, 0.0015, 0.002
        ).apply {
            enableContinuousInput(-180.0, 180.0)
            setTolerance(1.5)
        },
        turn = PIDController(
            0.0, 0.0, 0.0
        ),
        drive = definition.subsystems.driveTrain,
        gyro = gyro
    )

    private val autonomousChooser = SendableChooser<Command>()
    private val driveModeChooser = SendableChooser<DriveMode>()

    private val intakeMotor = CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless)

    private val limelight = definition.subsystems.driveTrain.vision

    override fun robotInit() {
        definition.subsystems.driveTrain.initialize()
        gyro.calibrate()

        with(autonomousChooser) {
            addOption("Move 2 Meters", definition.move(2.0))
            addOption("Balance", gyroBalance)
            SmartDashboard.putData("Autonomous Mode", this)
        }

        with(driveModeChooser) {
            setDefaultOption("Raw", DriveMode.Raw)
            addOption("Cheesy", DriveMode.Cheesy)
            SmartDashboard.putData("Drive Mode", this)
        }
    }

    private var leftMaxVal = 0.0
    private var rightMaxVal = 0.0
    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        leftMaxVal = maxOf(leftEncoder.rate, leftMaxVal)
        rightMaxVal = maxOf(rightEncoder.rate, rightMaxVal)
        SmartDashboard.putNumber("Left Encoder", leftMaxVal)
        SmartDashboard.putNumber("Right Encoder", rightMaxVal)
        SmartDashboard.putNumber("AngleX", gyro.gyroAngleX)
        SmartDashboard.putNumber("AngleY", gyro.gyroAngleY)
        SmartDashboard.putNumber("AngleZ", gyro.gyroAngleZ)
        SmartDashboard.putNumber("Forward Power", gyroBalance.forwardPower)
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
        gyro.reset();

        when (driveModeChooser.selected) {
            DriveMode.Cheesy -> definition.cheesy()
            DriveMode.Raw -> definition.raw()
            else -> definition.raw()
        }.schedule()
    }

    override fun teleopPeriodic() {
        limelight.periodic()
    }

    override fun teleopExit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testInit() {
    }
}
