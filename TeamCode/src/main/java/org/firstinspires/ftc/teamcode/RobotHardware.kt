package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.clamp
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.HardwareConstants.PIVOT_EXTENDED_KG
import org.firstinspires.ftc.teamcode.HardwareConstants.PIVOT_KI
import org.firstinspires.ftc.teamcode.HardwareConstants.PIVOT_KP
import org.firstinspires.ftc.teamcode.HardwareConstants.PIVOT_KS
import org.firstinspires.ftc.teamcode.HardwareConstants.PIVOT_RETRACTED_KG
import org.firstinspires.ftc.teamcode.HardwareConstants.PIVOT_TICKS_PER_RAD
import org.firstinspires.ftc.teamcode.HardwareConstants.SLIDES_EXTENDED_KG
import org.firstinspires.ftc.teamcode.HardwareConstants.SLIDES_KI
import org.firstinspires.ftc.teamcode.HardwareConstants.SLIDES_KP
import org.firstinspires.ftc.teamcode.HardwareConstants.SLIDES_KS
import org.firstinspires.ftc.teamcode.HardwareConstants.SLIDES_RETRACTED_KG
import org.firstinspires.ftc.teamcode.HardwareConstants.SLIDES_TICKS_IN_EXTENSION
import org.firstinspires.ftc.teamcode.HardwareConstants.WRIST_PITCH_OFFSET
import org.firstinspires.ftc.teamcode.HardwareConstants.WRIST_ROLL_OFFSET
import org.firstinspires.ftc.teamcode.HardwareConstants.WRIST_UNITS_PER_RAD
import org.firstinspires.ftc.teamcode.MathUtils.lerp
import java.lang.Double.max
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

@Config object HardwareConstants {

    @JvmField var SLIDES_TICKS_IN_EXTENSION = 2900.0;
    val PIVOT_TICKS_PER_RAD = 2786.2 / (2 * PI);

    @JvmField var SLIDES_KS = 0.6;
    @JvmField var SLIDES_RETRACTED_KG = 0.0;
    @JvmField var SLIDES_EXTENDED_KG = 0.0;
    @JvmField var PIVOT_KS = 0.5;
    @JvmField var PIVOT_RETRACTED_KG = 0.0;
    @JvmField var PIVOT_EXTENDED_KG = 0.0;

    // all PID gains are in units of volts/rad
    @JvmField var PIVOT_KP = 12.0;
    @JvmField var PIVOT_KI = 4.0;
    @JvmField var SLIDES_KP = 50.0;
    @JvmField var SLIDES_KI = 10.0;

    @JvmField var WRIST_UNITS_PER_RAD = 0.4 / (2 * PI) /2; // 2 units per 5 revolutions times 1 rev per 2pi radians (divided by 2 again for some reason)
    @JvmField var WRIST_PITCH_OFFSET = 15.2;
    @JvmField var WRIST_ROLL_OFFSET = -6.4;

    @JvmField var PLUNGER_RETRACTED_POS = 1.0;
    @JvmField var PLUNGER_EXTENDED_POS = 0.05;

}

class RobotHardware (private val hardwareMap: HardwareMap, private val telemetry: Telemetry) {


    var intakeSpeed = 0.0
    var intakeSpin = 0.0 // for rotating the game piece inside the end effector
    var plungerRetracted = false

    var targetPivotAngle = 0.0; // 0.0 when horizontal, pi/2 when vertical
    fun getCurrentPivotAngle(): Double {
        return rightPivot.currentPosition/ PIVOT_TICKS_PER_RAD
    }
    var targetSlideExtension = 0.0; // 0.0 when fully retracted, 1.0 when fully extended
    fun getCurrentSlideExtension(): Double {
        return rightExtend.currentPosition / SLIDES_TICKS_IN_EXTENSION
    }

    private val frontRightDrive: DcMotorEx by lazy {
        hardwareMap.get(DcMotorEx::class.java, "FrontRightDrive")
    }
    private val backRightDrive: DcMotorEx by lazy {
        hardwareMap.get(DcMotorEx::class.java, "BackRightDrive")
    }
    private val backLeftDrive: DcMotorEx by lazy {
        hardwareMap.get(DcMotorEx::class.java, "BackLeftDrive")
    }
    private val frontLeftDrive: DcMotorEx by lazy {
        hardwareMap.get(DcMotorEx::class.java, "FrontLeftDrive")
    }

    private val rightPivot: DcMotorEx by lazy {
        hardwareMap.get(DcMotorEx::class.java, "RightPivot")
    }
    private val leftPivot: DcMotorEx by lazy {
        hardwareMap.get(DcMotorEx::class.java, "LeftPivot")
    }

    private val rightExtend: DcMotorEx by lazy {
        hardwareMap.get(DcMotorEx::class.java, "RightExtend")
    }
    private val leftExtend: DcMotorEx by lazy {
        hardwareMap.get(DcMotorEx::class.java, "LeftExtend")
    }

    private val leftDiffy: Servo by lazy {
        hardwareMap.get(Servo::class.java, "LeftDiffy")
    }
    private val rightDiffy: Servo by lazy {
        hardwareMap.get(Servo::class.java, "RightDiffy")
    }
    private val plunger: Servo by lazy {
        hardwareMap.get(Servo::class.java, "IntakeExtend")
    }
    private val leftIntake: CRServo by lazy {
        hardwareMap.get(CRServo::class.java, "LeftIntake")
    }
    private val rightIntake: CRServo by lazy {
        hardwareMap.get(CRServo::class.java, "RightIntake")
    }

    private val imu: IMU by lazy {
        hardwareMap.get(IMU::class.java, "imu")
    }

    private val voltageSensor: VoltageSensor by lazy {
        hardwareMap.get(VoltageSensor::class.java, "Control Hub")
    }

    fun voltageToPower(voltage: Double): Double {
        return voltage/voltageSensor.voltage
    }

    private val PivotController = PIDSFController(
        ::getCurrentPivotAngle,
        PIVOT_KP,
        PIVOT_KI,
        PIVOT_KS,
        { cos(getCurrentPivotAngle()) * lerp(PIVOT_RETRACTED_KG, PIVOT_EXTENDED_KG, getCurrentSlideExtension()) }
    )
    val pivotVoltage by PivotController::voltage

    private val SlidesController = PIDSFController(
        ::getCurrentSlideExtension,
        SLIDES_KP,
        SLIDES_KI,
        SLIDES_KS,
        { sin(getCurrentPivotAngle()) * lerp(SLIDES_RETRACTED_KG, SLIDES_EXTENDED_KG, getCurrentSlideExtension()) }
    )
    var feedforwardSlidesVoltage = 0.0
    val pidSlidesVoltage by SlidesController::voltage
    var useSlidePID = true

    var wristPitch = 0.0; // radians; 0 for parallel with slides
    var wristRoll = 0.0; // radians; 0 for pulleys facing outward when horizontal

    var driveCommand = PoseVelocity2d(Vector2d(0.0, 0.0), 0.0);
    var zeroHeading = 0.0
    val currentHeading: Double
        get() {
            return -imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS) - zeroHeading
        }
    val currentPitch: Double
        get() {
            return imu.robotYawPitchRollAngles.getPitch(AngleUnit.RADIANS)
        }
    val rawHeading: Double
        get() {
            return -imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
        }
    /*
     * Code to run ONCE when the driver hits INIT
     */
    fun init() {
        val motors = hardwareMap.getAll(DcMotor::class.java)
        motors.forEach { motor -> motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER }

        // Set directions for pairs of motors
        // Also resets directions if another OpMode changed them
        frontLeftDrive.direction = DcMotorSimple.Direction.REVERSE
        backLeftDrive.direction = DcMotorSimple.Direction.REVERSE
        frontRightDrive.direction = DcMotorSimple.Direction.FORWARD
        backRightDrive.direction = DcMotorSimple.Direction.FORWARD

        frontRightDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backRightDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backLeftDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        frontLeftDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE


        // + -> counter-clockwise, when viewed from right
        leftPivot.direction = DcMotorSimple.Direction.FORWARD
        rightPivot.direction = DcMotorSimple.Direction.REVERSE
        leftPivot.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        rightPivot.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        // + -> extend
        leftExtend.direction = DcMotorSimple.Direction.REVERSE
        rightExtend.direction = DcMotorSimple.Direction.FORWARD
        leftExtend.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        rightExtend.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        // + -> counter-clockwise, when viewed from right
        leftDiffy.direction = Servo.Direction.REVERSE
        rightDiffy.direction = Servo.Direction.FORWARD

        // + -> intake
        leftIntake.direction = DcMotorSimple.Direction.FORWARD
        rightIntake.direction = DcMotorSimple.Direction.REVERSE

        imu.initialize(
            IMU.Parameters(
                RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
            )
        )



    }

    fun update() {

        // apply bounds to pivot and extension
        targetPivotAngle = clamp(targetPivotAngle, 0.2, PI/2 * 1.1)
        targetSlideExtension = clamp(targetSlideExtension, 0.01, 1.0)
        PivotController.kP = PIVOT_KP
        PivotController.kI = PIVOT_KI
        PivotController.kS = PIVOT_KS
        SlidesController.kP = SLIDES_KP
        SlidesController.kI = SLIDES_KI
        SlidesController.kS = SLIDES_KS
        PivotController.setPoint = targetPivotAngle
        PivotController.update()
        SlidesController.setPoint = targetSlideExtension
        SlidesController.update()

        var slidesVoltage = if (useSlidePID) pidSlidesVoltage else feedforwardSlidesVoltage
        rightExtend.power = voltageToPower(slidesVoltage)
        leftExtend.power = voltageToPower(slidesVoltage)
        rightPivot.power = voltageToPower(PivotController.voltage)
        leftPivot.power = voltageToPower(PivotController.voltage)

        rightDiffy.position = (wristPitch + wristRoll/2 + WRIST_PITCH_OFFSET + WRIST_ROLL_OFFSET/2) * WRIST_UNITS_PER_RAD
        leftDiffy.position = (wristPitch - wristRoll/2 + WRIST_PITCH_OFFSET - WRIST_ROLL_OFFSET/2) * WRIST_UNITS_PER_RAD

        plunger.position = if (plungerRetracted) HardwareConstants.PLUNGER_RETRACTED_POS else HardwareConstants.PLUNGER_EXTENDED_POS
        rightIntake.power = intakeSpeed + intakeSpin;
        leftIntake.power = intakeSpeed - intakeSpin;

        // I hate the FTC coordinate system
        var fwd = driveCommand.linearVel.x
        var left = driveCommand.linearVel.y
        var turn = driveCommand.angVel

        var frontRightPower = fwd - left - turn
        var backRightPower = fwd + left - turn
        var backLeftPower = fwd - left + turn
        var frontLeftPower = fwd + left + turn

        var maxPower = max(frontRightPower, max(backRightPower, max(backLeftPower, frontLeftPower)))
        if (maxPower > 1) {
            var normalizationFactor = 1/maxPower
            frontRightPower *= normalizationFactor
            backRightPower *= normalizationFactor
            backLeftPower *= normalizationFactor
            frontLeftPower *= normalizationFactor
        }

        frontRightDrive.power = voltageToPower(frontRightPower * 12)
        backRightDrive.power = voltageToPower(backRightPower * 12)
        backLeftDrive.power = voltageToPower(backLeftPower * 12)
        frontLeftDrive.power = voltageToPower(frontLeftPower * 12)


    }

}