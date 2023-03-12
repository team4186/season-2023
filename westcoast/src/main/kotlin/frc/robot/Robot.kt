package frc.robot

import com.ctre.phoenix.sensors.Pigeon2
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.commands.balancing.GyroBalance
import frc.commands.drive.CheesyDrive
import frc.commands.drive.TeleopDrive
import frc.commands.elevator.MoveCarriage
import frc.commands.elevator.MoveStageTwo
import frc.commands.elevator.MoveWrist
import frc.commands.elevator.ZeroElevator
import frc.subsystems.*
import frc.vision.LimelightRunner

const val MAX_SPEED_ENCODER = 2000
//joystick * ^^ = input to pids


class Robot : TimedRobot() {
    private enum class DriveMode {
        Raw,
        Cheesy
    }

    private val joystick = Joystick(0)

    private val gyro = Pigeon2(10, "rio")
    private val leftEncoder = Encoder(0, 1, true)
    private val rightEncoder = Encoder(2, 3)
    private val driveTrainSubsystem = DriveTrainSubsystem()
    private val elevatorSubsystem = ElevatorSubsystem()

    private val gyroBalance = GyroBalance(
        forward = PIDController(
            0.05, 0.0015, 0.002
        // main PID for balance that needs to be adjusted
        // power first until oscillates, I until no oscillations, then D until fast
        ).apply {
            enableContinuousInput(-180.0, 180.0)
            setTolerance(1.5)
        },
        turn = PIDController(
            0.05, 0.0, 0.0
            // power first until oscillates, I until no oscillations, then D until fast
        ),
        drive = driveTrainSubsystem,
        gyro = gyro
    )
    private val autonomousChooser = SendableChooser<Command>()
    private val driveModeChooser = SendableChooser<DriveMode>()

    private val intakeMotor = CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless)

    private val limelight = LimelightRunner()
    private val cheesyDrive = CheesyDrive(
        inputThrottle = { joystick.y },
        inputYaw = { joystick.x },
        drive = driveTrainSubsystem,
        sensitivityHigh = 0.5,
        sensitivityLow = 0.5
    )
    private val rawDrive = TeleopDrive(
        inputThrottle = { joystick.y },
        inputTurn = { joystick.twist },
        inputYaw = { joystick.x },
        drive = driveTrainSubsystem
    )

    private val triggers = listOf(
        Trigger { joystick.getRawButton(10) }
            .onTrue(
                MoveCarriage(
                    elevatorSubsystem,
                    0.0
                )
            ),
        Trigger { joystick.getRawButton(11) }
            .onTrue(
                MoveCarriage(
                    elevatorSubsystem,
                    CARRIAGE_END
                )
            ),
        Trigger { joystick.getRawButton(8) }
            .onTrue(
                MoveStageTwo(
                    elevatorSubsystem,
                    0.0
                )
            ),
        Trigger { joystick.getRawButton(9) }
            .onTrue(
                MoveStageTwo(
                    elevatorSubsystem,
                    STAGE_TWO_END
                )
            ),
        Trigger { joystick.getRawButton(6) }
            .onTrue(
                MoveWrist(
                    elevatorSubsystem,
                    0.0
                )
            ),
        Trigger { joystick.getRawButton(7) }
            .onTrue(
                MoveWrist(
                    elevatorSubsystem,
                    WRIST_END
                )
            ),
        Trigger { joystick.getRawButton(10) && joystick.getRawButton(11) }
            .onTrue(
                ZeroElevator(
                    elevatorSubsystem
                )
            )
    )

    override fun robotInit() {
        driveTrainSubsystem.initialize()

        with(autonomousChooser) {
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
        SmartDashboard.putNumber("Yaw", gyro.yaw)
        SmartDashboard.putNumber("Pitch", gyro.pitch)
        SmartDashboard.putNumber("Roll", gyro.roll)
        SmartDashboard.putNumber("Forward Power", gyroBalance.forwardPower)
    }

    override fun autonomousInit() {
        val autonomous = autonomousChooser.selected

        val zero = ZeroElevator(elevatorSubsystem)
        val command = if (autonomous != null) {
            zero.andThen(autonomous)
        } else {
            zero
        }
        command.schedule()
    }

    override fun autonomousPeriodic() {
    }

    override fun autonomousExit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun teleopInit() {
        // gyro.reset();


        when (driveModeChooser.selected) {
            DriveMode.Cheesy -> cheesyDrive
            DriveMode.Raw -> rawDrive
            else -> rawDrive
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
