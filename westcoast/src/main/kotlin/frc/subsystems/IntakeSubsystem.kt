package frc.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.MotorSafety
import edu.wpi.first.wpilibj2.command.CommandBase

class IntakeSubsystem(
    val intakeMotors: CANSparkMax = intakeSparkMaxMotors(
        lead = CANSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless),
        follower =  CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless)
    ),
) : CommandBase() {
    var intakeLimit: DigitalInput = DigitalInput(0)

    private val motorSafety: MotorSafety = object : MotorSafety() {
        override fun stopMotor() {
            stop()
        }

        override fun getDescription(): String {
            return "Intake"
        }
    }


    fun stop() {
        intakeMotors.stopMotor()
        motorSafety.feed()
    }

    fun setIntakeMotors(input: Double) {
        intakeMotors.set(input)
        motorSafety.feed()
    }
}

fun intakeSparkMaxMotors(
    lead: CANSparkMax,
    follower: CANSparkMax
): CANSparkMax {
    lead.idleMode = CANSparkMax.IdleMode.kBrake
    follower.idleMode = CANSparkMax.IdleMode.kBrake

    follower.follow(lead, true)

    lead.enableVoltageCompensation(11.0)
    follower.enableVoltageCompensation(11.0)

    return lead
}