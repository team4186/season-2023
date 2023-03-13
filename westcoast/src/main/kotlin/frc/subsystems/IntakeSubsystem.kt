package frc.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.MotorSafety
import edu.wpi.first.wpilibj2.command.CommandBase

class IntakeSubsystem(
    val intakeMotor0: CANSparkMax = intakeSparkMaxMotors(
        lead = CANSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless)
    ),
    val intakeMotor1: CANSparkMax = intakeSparkMaxMotors(
        lead = CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless)
    )
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
        intakeMotor0.stopMotor()
        intakeMotor1.stopMotor()
        motorSafety.feed()
    }

    fun setIntakeMotors(input: Double) {
        intakeMotor0.set(input)
        intakeMotor1.set(-input)
        motorSafety.feed()
    }
}

fun intakeSparkMaxMotors(
    lead: CANSparkMax
): CANSparkMax {
    lead.idleMode = CANSparkMax.IdleMode.kBrake

    lead.enableVoltageCompensation(11.0)

    return lead
}