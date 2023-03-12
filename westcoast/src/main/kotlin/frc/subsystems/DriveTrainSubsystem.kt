package frc.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.MotorSafety
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import edu.wpi.first.wpilibj2.command.SubsystemBase

class DriveTrainSubsystem(
    private val leftMotor: MotorController = driveSparkMaxMotors(
        lead = CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless),
        follower0 = CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless),
        invert = false,
    ),
    private val rightMotor: MotorController = driveSparkMaxMotors(
        lead = CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless),
        follower0 = CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless),
        invert = true,
    ),
    private val hMotor: MotorController = driveSparkMaxMotors(
        lead = CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless),
        follower0 = CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless),
        invert = false,
    )
) : SubsystemBase() {
    private val drive: DifferentialDrive = DifferentialDrive(leftMotor, rightMotor)

    private val motorSafety: MotorSafety = object : MotorSafety() {
        override fun stopMotor() {
            stop()
        }

        override fun getDescription(): String {
            return "EncoderDrive"
        }
    }

    fun initialize() {
        drive.stopMotor()
        drive.isSafetyEnabled = false
    }

    fun stop() {
        drive.stopMotor()
        motorSafety.feed()
    }

    fun arcade(forward: Double, turn: Double, squareInputs: Boolean) {
        drive.arcadeDrive(forward, turn, squareInputs)
    }

    fun holonomic(forward: Double, turn: Double, strafe: Double, squareInputs: Boolean) {
        drive.arcadeDrive(forward, turn, squareInputs)
        hMotor.set(strafe.coerceIn(-0.4, 0.4))
    }

    fun tank(left: Double, right: Double, squareInputs: Boolean) {
        drive.tankDrive(left, right, squareInputs)
    }

    fun setMotorOutput(left: Double, right: Double) {
        leftMotor.set(left)
        rightMotor.set(right)
        motorSafety.feed()
    }
}

fun driveSparkMaxMotors(
    lead: CANSparkMax,
    follower0: CANSparkMax,
    invert: Boolean
): MotorController {
    follower0.follow(lead)
    follower0.inverted = invert

    // endregion

    // region Lead
    lead.inverted = invert

    lead.idleMode = CANSparkMax.IdleMode.kBrake

    // endregion

    // region Voltage Saturation
    // See https://docs.ctre-phoenix.com/en/stable/ch13_MC.html#voltage-compensation
    lead.enableVoltageCompensation(11.0)
    follower0.enableVoltageCompensation(11.0)

    // endregion
    return lead
}