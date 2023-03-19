package frc.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.MotorSafety
import edu.wpi.first.wpilibj2.command.SubsystemBase

//change constant values
const val CARRIAGE_END = 174.0 // |162 - -11| = 173
const val STAGE_TWO_END = 80.0
const val WRIST_END = -91.7

class ElevatorSubsystem(
    //set IDs
    val carriageMotor: CANSparkMax = elevatorSparkMaxMotors(
        lead = CANSparkMax(8, CANSparkMaxLowLevel.MotorType.kBrushless),
        invert = true
    ),
    val stageTwoMotor: CANSparkMax = elevatorSparkMaxMotors(
        lead = CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless),
        invert = true
    ),
    val wristMotor: CANSparkMax = elevatorSparkMaxMotors(
        lead = CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless),
        invert = false
    ).apply {
            idleMode = CANSparkMax.IdleMode.kCoast
    },
    val carriageLimitTop: DigitalInput = DigitalInput(5), // originally 4; swap
    val carriageLimitBottom: DigitalInput = DigitalInput(4), // originally 5; swap
    val stageLimitTop: DigitalInput = DigitalInput(6),
    val stageLimitBottom: DigitalInput = DigitalInput(3),
    val wristLimitTop: DigitalInput = DigitalInput(1),
    val wristLimitBottom: DigitalInput = DigitalInput(2)
) : SubsystemBase() {

    override fun periodic() {
        if (wristMotor.outputCurrent > 20.0){
            wristMotor.stopMotor()
        }
        if (carriageMotor.outputCurrent > 20.0){
            carriageMotor.stopMotor()
        }
        if (stageTwoMotor.outputCurrent > 20.0){
            stageTwoMotor.stopMotor()
        }
    }

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