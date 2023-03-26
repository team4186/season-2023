package frc.robot

import com.ctre.phoenix.sensors.Pigeon2
import com.revrobotics.CANSparkMax
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
import frc.commands.drive.BrakeMotors
import frc.commands.drive.CheesyDrive
import frc.commands.drive.LeaveLine
import frc.commands.drive.TeleopDrive
import frc.commands.elevator.MoveCarriage
import frc.commands.elevator.MoveStageTwo
import frc.commands.elevator.MoveWrist
import frc.commands.elevator.ZeroElevator
import frc.commands.targeting.AlignToTarget
import frc.subsystems.*
import frc.vision.LimelightRunner

//const val MAX_SPEED_ENCODER = 2000
//joystick * ^^ = input to pids


class Robot : TimedRobot() {
    private enum class DriveMode {
        Raw,
        Cheesy
    }

    private val joystick0 = Joystick(0) //drive joystick
    private val joystick1 = Joystick(1) //co-drive joystick


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
        gyro = gyro,
        { gyroCompassStartPos }
    )
    private val autonomousChooser = SendableChooser<Command>()
    private val driveModeChooser = SendableChooser<DriveMode>()

    private val limelight = LimelightRunner()
    private val cheesyDrive = CheesyDrive(
        inputThrottle = { joystick0.y },
        inputYaw = { joystick0.x },
        drive = driveTrainSubsystem,
        sensitivityHigh = 0.5,
        sensitivityLow = 0.5
    )
    private val rawDrive = TeleopDrive(
        inputThrottle = { joystick0.y },
        inputTurn = { joystick0.twist },
        inputYaw = { joystick0.x },
        drive = driveTrainSubsystem
    )

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
        0.0,
        gyro,
        { gyroCompassStartPos }
    )

    private val alignToLeftCone = AlignToTarget(
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
        -33.0,
        gyro,
        { gyroCompassStartPos }
    )

    private val alignToRightCone = AlignToTarget(
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
        33.0,
        gyro,
        { gyroCompassStartPos }
    )

    private val triggers = listOf(
        //Align to target
        Trigger { joystick0.getRawButton(1) }
            .whileTrue(
                alignToTarget
            ),

        //INTAKE TRIGGERS
        Trigger { joystick1.getRawButton(1) || joystick0.getRawButton(2) }
            .whileTrue(
                Intake(
                    intakeSubsystem
                )
            ),
        // EJECT
        Trigger { joystick1.getRawButton(2) || joystick0.getRawButton(2) }
            .whileTrue(
                Eject(
                    intakeSubsystem
                )
            ),

        //Brake Motors
        Trigger { joystick0.getRawButton(2) }
            .whileTrue(
                BrakeMotors(
                    driveTrainSubsystem
                )
            ),

        //Align to target
        Trigger { joystick0.getRawButton(3) || joystick1.getRawButton(3) }
            .whileTrue(
                alignToLeftCone
            ),

        Trigger { joystick0.getRawButton(4) || joystick1.getRawButton(4) }
            .whileTrue(
                alignToRightCone
            ),

        Trigger { joystick0.getRawButton(5) || joystick1.getRawButton(5)}
            .whileTrue(
                gyroBalance
            ),

        //ZERO
        Trigger { joystick0.getRawButton(6) || joystick1.getRawButton(6) }
            .whileTrue(
                ZeroElevator(
                    elevatorSubsystem
                )
            ),

        //STAGE TWO

        // DOWN
        Trigger {
            (joystick0.getRawButton(7) || joystick1.getRawButton(7))
        }
            .onTrue(
                MoveStageTwo(
                    elevatorSubsystem,
                    0.0
                ).until { elevatorSubsystem.stageLimitBottom.get() }
            ),

        // UP
        Trigger {
            (joystick0.getRawButton(8) || joystick1.getRawButton(8))
        }
            .onTrue(
                MoveStageTwo(
                    elevatorSubsystem,
                    STAGE_TWO_END
                ).until { elevatorSubsystem.stageLimitTop.get() }
            ),

        //MOVE WRIST

        // UP
        Trigger { joystick0.getRawButton(9) || joystick1.getRawButton(9) }
            .onTrue(
                MoveWrist(
                    elevatorSubsystem,
                    WRIST_END
                ).until { elevatorSubsystem.wristLimitBottom.get() } // should this be LimitTop ??
            ),
        // DOWN
        Trigger { joystick0.getRawButton(10) || joystick1.getRawButton(10) }
            .onTrue(
                MoveWrist(
                    elevatorSubsystem,
                    0.0
                ).until { elevatorSubsystem.wristLimitTop.get() } // should this be LimitBot ? or am i dumb
            ),

        //MOVE CARRIAGE

        // DOWN
        Trigger { joystick0.getRawButton(11) || joystick1.getRawButton(11) }
            .onTrue(
                MoveCarriage(
                    elevatorSubsystem,
                    0.0
                ).until { elevatorSubsystem.carriageLimitBottom.get() }
            ),
        // UP
        Trigger { joystick0.getRawButton(12) || joystick1.getRawButton(12) }
            .onTrue(
                MoveCarriage(
                    elevatorSubsystem,
                    CARRIAGE_END
                ).until { elevatorSubsystem.carriageLimitTop.get() }
            ),
        Trigger { joystick0.getRawButton(13) || joystick1.getRawButton(13) } // assign buttons
            .onTrue(
                MoveWrist(
                    elevatorSubsystem,
                    WRIST_END
                ).until { elevatorSubsystem.wristLimitTop.get() }
                    .alongWith( // does that work????
                        MoveCarriage(
                            elevatorSubsystem,
                            CARRIAGE_END
                        ).until { elevatorSubsystem.carriageLimitTop.get() }
                    )
            ),
        // carriage and wrist down
        Trigger { joystick0.getRawButton(13) || joystick1.getRawButton(13) } // assign buttons
            .onTrue(
                MoveWrist(
                    elevatorSubsystem,
                    0.0
                ).until { elevatorSubsystem.wristLimitBottom.get() }
                    .alongWith( // does that work????
                        MoveCarriage(
                            elevatorSubsystem,
                            0.0
                        ).until { elevatorSubsystem.carriageLimitBottom.get() }
                    )
            )
    )

    var gyroCompassStartPos = 0.0
    override fun robotInit() {
        driveTrainSubsystem.initialize()
        driveTrainSubsystem.leftMotor.idleMode = CANSparkMax.IdleMode.kCoast
        driveTrainSubsystem.rightMotor.idleMode = CANSparkMax.IdleMode.kCoast
        gyro.zeroGyroBiasNow()
        gyroCompassStartPos = gyro.absoluteCompassHeading
        limelight.setLight(true)

        with(autonomousChooser) {
            addOption(
                "Default/leave line (move out)",
                LeaveLine(
                    distance = 50.0, // 7ft for now; 4.4 encoder/ ticks per foot 30.8 is 7ft
                    left = PIDController(0.15, 0.0, 0.0),
                    right = PIDController(0.15, 0.0, 0.0),
                    drive = driveTrainSubsystem,
                    rightEncoder = { rightEncoder.position },
                    leftEncoder = { leftEncoder.position }
                )
            )
            addOption("Leave Lines & Balance",
                LeaveLine(
                    distance = 55.0, //   14ft?          4.4 encoder/ ticks per foot
                    left = PIDController(0.15, 0.0, 0.0),
                    right = PIDController(0.15, 0.0, 0.0),
                    drive = driveTrainSubsystem,
                    rightEncoder = { rightEncoder.position },
                    leftEncoder = { leftEncoder.position }
                ).andThen(
                    LeaveLine(
                        distance = -15.2, // 3.5 ft
                        left = PIDController(0.15, 0.0, 0.0),
                        right = PIDController(0.15, 0.0, 0.0),
                        drive = driveTrainSubsystem,
                        rightEncoder = { rightEncoder.position },
                        leftEncoder = { leftEncoder.position }
                    )
                ).andThen(
                    gyroBalance
                )
            )
            addOption("Nothing", null)

            // implement new auton: score loaded game piece and then move out

            addOption(
                "Score loaded + leave line",
                LeaveLine(
                    distance = 10.0, // 7ft for now; 4.4 encoder/ ticks per foot 30.8 is 7ft
                    left = PIDController(0.15, 0.0, 0.0),
                    right = PIDController(0.15, 0.0, 0.0),
                    drive = driveTrainSubsystem,
                    rightEncoder = { rightEncoder.position },
                    leftEncoder = { leftEncoder.position }
                ).andThen(
                    MoveCarriage(
                        elevatorSubsystem,
                        CARRIAGE_END
                    ).until { elevatorSubsystem.carriageLimitTop.get() }
                ).andThen(
                    MoveStageTwo(
                        elevatorSubsystem,
                        STAGE_TWO_END
                    ).until { elevatorSubsystem.stageLimitTop.get() }
                ).andThen(
                    MoveWrist(
                        elevatorSubsystem,
                        WRIST_END
                    ).until { elevatorSubsystem.wristLimitBottom.get() }
                ).andThen(
                    Eject(
                        intakeSubsystem
                    ).until { !intakeSubsystem.intakeLimit.get()} // eject till the sensor stops detecting
                ).andThen(
                    MoveWrist(
                        elevatorSubsystem,
                        0.0
                    ).until { elevatorSubsystem.wristLimitTop.get() }
                ).andThen(
                    MoveStageTwo(
                        elevatorSubsystem,
                        0.0
                    ).until { elevatorSubsystem.stageLimitBottom.get() }
                ).andThen(
                    MoveCarriage(
                        elevatorSubsystem,
                        0.0
                    ).until { elevatorSubsystem.carriageLimitBottom.get() }
                ).andThen(
                    LeaveLine(
                        distance = 30.0, // 7ft for now; 4.4 encoder/ ticks per foot 30.8 is 7ft
                        left = PIDController(0.15, 0.0, 0.0),
                        right = PIDController(0.15, 0.0, 0.0),
                        drive = driveTrainSubsystem,
                        rightEncoder = { rightEncoder.position },
                        leftEncoder = { leftEncoder.position }
                    )
                )
            )
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

        limelight.periodic()
    }

    override fun autonomousInit() {
        driveTrainSubsystem.leftMotor.idleMode = CANSparkMax.IdleMode.kBrake
        driveTrainSubsystem.rightMotor.idleMode = CANSparkMax.IdleMode.kBrake
        val autonomous = autonomousChooser.selected
        autonomous.schedule()

//        val zero = ZeroElevator(elevatorSubsystem)
//        val command = if (autonomous != null) {
//            zero.andThen(autonomous)
//        } else {
//            zero
//        }
//        command.schedule
    }

    override fun autonomousPeriodic() {
    }

    override fun autonomousExit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun teleopInit() {
        // gyro.reset();
        driveTrainSubsystem.leftMotor.idleMode = CANSparkMax.IdleMode.kCoast
        driveTrainSubsystem.rightMotor.idleMode = CANSparkMax.IdleMode.kCoast

        limelight.setLight(true)

        when (driveModeChooser.selected) {
            DriveMode.Cheesy -> cheesyDrive
            DriveMode.Raw -> rawDrive
            else -> rawDrive
        }.schedule()
    }

    override fun teleopPeriodic() {
    }

    override fun teleopExit() {
        driveTrainSubsystem.leftMotor.idleMode = CANSparkMax.IdleMode.kBrake
        driveTrainSubsystem.rightMotor.idleMode = CANSparkMax.IdleMode.kBrake
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testInit() {
    }
}
