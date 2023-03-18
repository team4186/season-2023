package frc.robot

import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.commands.Intake.Eject
import frc.commands.Intake.Intake
import frc.commands.balancing.GyroBalance
import frc.commands.drive.CheesyDrive
import frc.commands.drive.LeaveLine
import frc.commands.drive.TeleopDrive
import frc.commands.elevator.MoveCarriage
import frc.commands.elevator.MoveStageTwo
import frc.commands.elevator.MoveWrist
import frc.commands.elevator.ZeroElevator
import frc.commands.targeting.AlignToTarget
import frc.commands.targeting.ConstantlyAlignToTarget
import frc.subsystems.*
import frc.vision.LimelightRunner

//const val MAX_SPEED_ENCODER = 2000
//joystick * ^^ = input to pids


class Robot : TimedRobot() {
    private enum class DriveMode {
        Raw,
        Cheesy
    }

    private val joystick = Joystick(0)

    private val gyro = Pigeon2(10, "rio")

    //    private val leftEncoder = Encoder(0, 1, true)
//    private val rightEncoder = Encoder(2, 3)
    private val driveTrainSubsystem = DriveTrainSubsystem()
    private val leftEncoder = driveTrainSubsystem.leftMotor.encoder
    private val rightEncoder = driveTrainSubsystem.rightMotor.encoder
    private val hEncoder = driveTrainSubsystem.hMotor.encoder
    private val elevatorSubsystem = ElevatorSubsystem()
    private val intakeSubsystem = IntakeSubsystem()

    private val gyroBalance = GyroBalance(
        forward = PIDController(
            0.05, 0.0015, 0.002
            // main PID for balance that needs to be adjusted
            // power first until oscillates, I until gets there fast, then D until no oscillations
        ).apply {
            enableContinuousInput(-180.0, 180.0)
            setTolerance(1.5)
        },
        turn = PIDController(
            0.05, 0.0, 0.0
            // power first until oscillates, I until gets there fast, then D until no oscillations
        ),
        drive = driveTrainSubsystem,
        gyro = gyro
    )
    private val autonomousChooser = SendableChooser<Command>()
    private val driveModeChooser = SendableChooser<DriveMode>()

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

    private val offset = 0.0
    private val alignToTarget = AlignToTarget(
        forward = PIDController(
            0.05, 0.0, 0.0
            // power first until oscillates, I until gets there fast, then D until no oscillations
        ),
        turn = PIDController(
            0.05, 0.0, 0.0
            // power first until oscillates, I until gets there fast, then D until no oscillations
        ),
        strafe = PIDController(
            0.05, 0.0, 0.0
            // power first until oscillates, I until gets there fast, then D until no oscillations
        ),
        driveTrainSubsystem,
        limelight,
        offset,
        gyro,
        { gyroCompassStartPos }
    )

    private val triggers = listOf(
        //INTAKE TRIGGERS
        Trigger { joystick.getRawButton(1)}
            .whileTrue(
                Intake(
                    intakeSubsystem
                )
            ),
        Trigger { joystick.getRawButton(2) }
            .whileTrue(
                Eject(
                    intakeSubsystem
                )
            ),

        //Align to target
        Trigger { joystick.getRawButton(3) }
            .whileTrue(
                alignToTarget
            ),

        Trigger { joystick.getRawButton(4) }
            .whileTrue(
                alignToTarget
            ),

        Trigger { joystick.getRawButton(5) } // change button
            .whileTrue(
                alignToTarget
            ),

        //ZERO
        Trigger { joystick.getRawButton(6) }
            .whileTrue(
                ZeroElevator(
                    elevatorSubsystem
                )
            ),

        //STAGE TWO
        Trigger { joystick.getRawButton(7) }
            .onTrue(
                MoveStageTwo(
                    elevatorSubsystem,
                    0.0
                ).until { elevatorSubsystem.stageLimitBottom.get() }
            ),
        Trigger { joystick.getRawButton(8) }
            .onTrue(
                MoveStageTwo(
                    elevatorSubsystem,
                    STAGE_TWO_END
                ).until { elevatorSubsystem.stageLimitTop.get() }
            ),

        //MOVE WRIST
        Trigger { joystick.getRawButton(9) }
            .onTrue(
                MoveWrist(
                    elevatorSubsystem,
                    WRIST_END
                ).until { elevatorSubsystem.wristLimitBottom.get() }
            ),
        Trigger { joystick.getRawButton(10) }
            .onTrue(
                MoveWrist(
                    elevatorSubsystem,
                    0.0
                ).until { elevatorSubsystem.wristLimitTop.get() }
            ),

        //MOVE CARRIAGE
        Trigger { joystick.getRawButton(11) }
            .onTrue(
                MoveCarriage(
                    elevatorSubsystem,
                    0.0
                ).until { elevatorSubsystem.carriageLimitBottom.get() }
            ),
        Trigger { joystick.getRawButton(12) }
            .onTrue(
                MoveCarriage(
                    elevatorSubsystem,
                    CARRIAGE_END
                ).until { elevatorSubsystem.carriageLimitTop.get() }
            ),
        )

    var gyroCompassStartPos = 0.0
    override fun robotInit() {
        driveTrainSubsystem.initialize()
        gyro.zeroGyroBiasNow()
        gyroCompassStartPos = gyro.absoluteCompassHeading

        with(autonomousChooser) {
            setDefaultOption(
                "Default/leave line (move out)",
                LeaveLine(
                    distance = -5.0, // 5m for comp
                    left = PIDController(0.25, 0.0, 0.0),
                    right = PIDController(0.25, 0.0, 0.0),
                    drive = driveTrainSubsystem,
                    rightEncoder = { rightEncoder.position },
                    leftEncoder = { leftEncoder.position }
                )
            )

            addOption("Balance", gyroBalance)
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
        SmartDashboard.putNumber("Left Encoder", leftEncoder.position)
        SmartDashboard.putNumber("Right Encoder", rightEncoder.position)
        SmartDashboard.putNumber("H Drive Encoder", hEncoder.position)

        SmartDashboard.putNumber("Yaw", gyro.yaw)
        SmartDashboard.putNumber("Pitch", gyro.pitch)
        SmartDashboard.putNumber("Roll", gyro.roll)
        SmartDashboard.putNumber("Forward Power", gyroBalance.forwardPower)

        with(elevatorSubsystem) {
            SmartDashboard.putNumber("Carriage encoder", carriageMotor.encoder.position)
            SmartDashboard.putNumber("Stage Two encoder", stageTwoMotor.encoder.position)
            SmartDashboard.putNumber("Wrist encoder", wristMotor.encoder.position)

            SmartDashboard.putBoolean("carriageLimitTop", carriageLimitTop.get())
            SmartDashboard.putBoolean("carriageLimitBottom", carriageLimitBottom.get())
            SmartDashboard.putBoolean("stageLimitTop", stageLimitTop.get())
            SmartDashboard.putBoolean("stageLimitBottom", stageLimitBottom.get())
            SmartDashboard.putBoolean("wristLimitTop", wristLimitTop.get())
            SmartDashboard.putBoolean("wristLimitBottom", wristLimitBottom.get())

            SmartDashboard.putNumber("Carriage Motor Amps", carriageMotor.outputCurrent)
            SmartDashboard.putNumber("Carriage Motor Temp", carriageMotor.motorTemperature)

            SmartDashboard.putNumber("Wrist Motor Amps", wristMotor.outputCurrent)
            SmartDashboard.putNumber("Wrist Motor Temp", wristMotor.motorTemperature)

            SmartDashboard.putNumber("Stage Motor Amps", stageTwoMotor.outputCurrent)
            SmartDashboard.putNumber("Stage Motor Temp", stageTwoMotor.motorTemperature)

        }
        SmartDashboard.putBoolean("Intake Sensor", intakeSubsystem.intakeLimit.get())
        SmartDashboard.putNumber("Intake Motor Amps", intakeSubsystem.intakeMotors.outputCurrent)
        SmartDashboard.putNumber("Intake Motor Temp", intakeSubsystem.intakeMotors.motorTemperature)
        SmartDashboard.putBoolean("StopBottomStageTwo", false)

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
