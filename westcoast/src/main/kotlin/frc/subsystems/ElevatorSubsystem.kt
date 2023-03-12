package frc.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.MotorSafety
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.commands.elevator.ZeroElevator

//change constant values
const val CARRIAGE_END = 1000.0
const val STAGE_TWO_END = 1000.0
const val WRIST_END = 200.0

class ElevatorSubsystem(
    //set IDs
    val carriageMotor: CANSparkMax = elevatorSparkMaxMotors(
        lead = CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless),
        invert = false
    ),
    val stageTwoMotor: CANSparkMax = elevatorSparkMaxMotors(
        lead = CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless),
        invert = false
    ),
    val wristMotor: CANSparkMax = elevatorSparkMaxMotors(
        lead = CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless),
        invert = false
    )
) : SubsystemBase() {
    // change channels
    var carriageLimitTop: DigitalInput = DigitalInput(0)
    var carriageLimitBottom: DigitalInput = DigitalInput(0)
    var stageLimitTop: DigitalInput = DigitalInput(0)
    var stageLimitBottom: DigitalInput = DigitalInput(0)
    var wristLimitTop: DigitalInput = DigitalInput(0)
    var wristLimitBottom: DigitalInput = DigitalInput(0)

    private val motorSafety: MotorSafety = object : MotorSafety() {
        override fun stopMotor() {
            stopAll()
        }

        override fun getDescription(): String {
            return "Elevator"
        }
    }


    fun stopAll() {
        carriageMotor.stopMotor()
        stageTwoMotor.stopMotor()
        wristMotor.stopMotor()
        motorSafety.feed() //what do?
    }

    fun setCarriageMotor(input: Double) {
        carriageMotor.set(input)
        motorSafety.feed()
    }

    fun setStageTwo(input: Double) {
        stageTwoMotor.set(input)
        motorSafety.feed()
    }

    fun setWristMotor(input: Double) {
        wristMotor.set(input)
        motorSafety.feed()
    }
}

fun elevatorSparkMaxMotors(
    lead: CANSparkMax,
    invert: Boolean
): CANSparkMax {

    lead.inverted = invert
    lead.idleMode = CANSparkMax.IdleMode.kBrake

    lead.enableVoltageCompensation(11.0)

    return lead
}