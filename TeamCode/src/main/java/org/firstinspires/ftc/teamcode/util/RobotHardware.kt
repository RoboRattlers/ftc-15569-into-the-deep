package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.clamp
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.util.HardwareConstants.DRIVE_MAX_VOLTAGE
import org.firstinspires.ftc.teamcode.util.HardwareConstants.PIVOT_EXTENDED_KG
import org.firstinspires.ftc.teamcode.util.HardwareConstants.PIVOT_KD
import org.firstinspires.ftc.teamcode.util.HardwareConstants.PIVOT_KI
import org.firstinspires.ftc.teamcode.util.HardwareConstants.PIVOT_KP
import org.firstinspires.ftc.teamcode.util.HardwareConstants.PIVOT_KS
import org.firstinspires.ftc.teamcode.util.HardwareConstants.PIVOT_RETRACTED_KG
import org.firstinspires.ftc.teamcode.util.HardwareConstants.PIVOT_TICKS_PER_RAD
import org.firstinspires.ftc.teamcode.util.HardwareConstants.SLIDES_EXTENDED_KG
import org.firstinspires.ftc.teamcode.util.HardwareConstants.SLIDES_KD
import org.firstinspires.ftc.teamcode.util.HardwareConstants.SLIDES_KI
import org.firstinspires.ftc.teamcode.util.HardwareConstants.SLIDES_KP
import org.firstinspires.ftc.teamcode.util.HardwareConstants.SLIDES_KS
import org.firstinspires.ftc.teamcode.util.HardwareConstants.SLIDES_OFFSET_PER_PIVOT_RADIAN
import org.firstinspires.ftc.teamcode.util.HardwareConstants.SLIDES_RETRACTED_KG
import org.firstinspires.ftc.teamcode.util.HardwareConstants.SLIDES_TICKS_IN_EXTENSION
import org.firstinspires.ftc.teamcode.util.HardwareConstants.WRIST_PITCH_OFFSET
import org.firstinspires.ftc.teamcode.util.HardwareConstants.WRIST_ROLL_OFFSET
import org.firstinspires.ftc.teamcode.util.HardwareConstants.WRIST_UNITS_PER_RAD
import org.firstinspires.ftc.teamcode.util.MathUtils.lerp
import java.lang.Double.max
import java.util.function.Supplier
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

@Config object HardwareConstants {

    @JvmField var CAMERA_IN_PER_PIXEL = 1.5 / 36.0 // 1.5 in per 36 pixels
    @JvmField var CAMERA_CENTER_X_OFFSET = 0.0

    @JvmField var FLOOR_FRICTION_MULTIPLIER = 1.05;

    @JvmField var PTO_ACTIVE_POSITION = 0.9
    @JvmField var SLIDES_TICKS_IN_EXTENSION = 1060.0 * 40.0/30.0 // original spool diameter/new spool diameter
    val SLIDE_STROKE = (180.0 * 4.0 / 25.4) // inches
    val SLIDES_EXTENSION_PER_IN = 1.0 / SLIDE_STROKE
    val PIVOT_TICKS_PER_RAD = (/*1930.0*/ 500.0 * 4.0) / (2.0 * PI) // output ticks per revolution  *  (1 revolution / 2pi radians)
    val SLIDES_OFFSET_PER_PIVOT_RADIAN = (-35.0 * 4.0) / (2.0 * PI)
    val DRIVE_MAX_VOLTAGE = 12.2

    @JvmField var SLIDES_KS = 0.93;
    @JvmField var SLIDES_RETRACTED_KG = 2.0;
    @JvmField var SLIDES_EXTENDED_KG = 2.0;
    @JvmField var PIVOT_KS = 0.6;
    @JvmField var PIVOT_RETRACTED_KG = 0.5;
    @JvmField var PIVOT_EXTENDED_KG = 4.0;

    @JvmField var PIVOT_KP = 20.0;
    @JvmField var PIVOT_KI = 0.0;
    @JvmField var PIVOT_KD = 0.0;
    @JvmField var SLIDES_KP = 60.0;
    @JvmField var SLIDES_KI = 2.0;
    @JvmField var SLIDES_KD = 5.0;

    @JvmField var WRIST_UNITS_PER_RAD = 0.21 * (270.0/300.0)// I think this is named incorrectly but idgaf
    @JvmField var WRIST_PITCH_OFFSET = 0.0;
    @JvmField var WRIST_ROLL_OFFSET = 0.0;

    val CHASSIS_WIDTH = 15.0;
    const val CHASSIS_LENGTH = 15.0;
    const val DISTANCE_BETWEEN_CHASSIS_AND_WALL_AT_TILE_CENTER = (24.0 - CHASSIS_LENGTH)/2.0;

}

class RobotHardware (val hardwareMap: HardwareMap, val telemetry: Telemetry) {


    var intakeSpeed = 0.0
    var intakeSpin = 0.0 // for rotating the game piece inside the end effector

    var ptoActive = false

    var commandDrivetrain = true

    var runtime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)
    var targetPivotAngle = 0.0; // 0.0 when horizontal, pi/2 when vertical
    fun getCurrentPivotAngle(): Double {
        return (pivotEncoder.getPositionAndVelocity().position.toDouble() / PIVOT_TICKS_PER_RAD) //+ 0.03 //rightPivot.currentPosition/ PIVOT_TICKS_PER_RAD
    }
    var targetSlideExtension = 0.0; // 0.0 when fully retracted, 1.0 when fully extended
    fun getCurrentSlideExtension(): Double {
        return (rightExtend.currentPosition - SLIDES_OFFSET_PER_PIVOT_RADIAN * getCurrentPivotAngle() ) / SLIDES_TICKS_IN_EXTENSION
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
    private val leftPTO: Servo by lazy {
        hardwareMap.get(Servo::class.java, "LeftPTO")
    }
    private val leftIntake: CRServo by lazy {
        hardwareMap.get(CRServo::class.java, "LeftIntake")
    }
    private val rightIntake: CRServo by lazy {
        hardwareMap.get(CRServo::class.java, "RightIntake")
    }
    private val pivotEncoder: OverflowEncoder by lazy {
        OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, "BackRightDrive")));
    }
    private val colorSensor: ColorSensor by lazy {
        hardwareMap.get(ColorSensor::class.java, "ColorSensor")
    }

    val hubs: List<LynxModule> by lazy {
        hardwareMap.getAll(LynxModule::class.java)
    }

    val imu: IMU by lazy {
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
        -10.0, 10.0,
        PIVOT_KI,
        PIVOT_KD,
        PIVOT_KS,
        { cos(getCurrentPivotAngle()) * lerp(PIVOT_RETRACTED_KG, PIVOT_EXTENDED_KG, getCurrentSlideExtension()) }
    )
    val pivotVoltage by PivotController::voltage

    private val SlidesController = PIDSFController(
        ::getCurrentSlideExtension,
        SLIDES_KP,
        -14.0, 14.0,
        SLIDES_KI,
        SLIDES_KD,
        SLIDES_KS,
        { sin(getCurrentPivotAngle()) * lerp(SLIDES_RETRACTED_KG, SLIDES_EXTENDED_KG, getCurrentSlideExtension()) }
    )
    var feedforwardSlidesVoltage = 0.0
    val pidSlidesVoltage by SlidesController::voltage
    var useSlidePID = true

    var wristPitch = 0.0; // radians; 0 for parallel with slides
    var wristRoll = 0.0; // radians; 0 for pulleys facing inward when horizontal

    var driveCommand = PoseVelocity2d(Vector2d(0.0, 0.0), 0.0);
    val currentHeading: Double
        get() {
            return -imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS) //- zeroHeading
        }
    val currentPitch: Double
        get() {
            return imu.robotYawPitchRollAngles.getPitch(AngleUnit.RADIANS)
        }
    val rawHeading: Double
        get() {
            return -imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
        }

    fun getSampleInsideIntake(): SampleColor {
            val red = colorSensor.red()
            val green = colorSensor.green()
            val blue = colorSensor.blue()
            val maxColor = red.coerceAtLeast(green).coerceAtLeast(blue)
            return if ((colorSensor as OpticalDistanceSensor).lightDetected < 0.13) SampleColor.NONE
                else if (maxColor == red) SampleColor.RED
                else if (maxColor == green) SampleColor.YELLOW
                else SampleColor.BLUE
        }

    fun pivotToAngleAction(angle: Double, tolerance: Double = 0.07): Action {
        return Action {
            targetPivotAngle = angle
            return@Action abs(getCurrentPivotAngle() - targetPivotAngle) > tolerance
        }
    }

    fun slideToPosAction(pos: () -> Double, tolerance: Double = 0.07): Action {
        return Action {
            targetSlideExtension = pos.invoke()
            return@Action abs(getCurrentSlideExtension() - targetSlideExtension) > tolerance
        }
    }

    fun slideToPosAction(pos: Double, tolerance: Double = 0.07): Action {
        return slideToPosAction({ pos}, tolerance)
    }


    private fun timedAction(func: (Boolean, TelemetryPacket) -> Boolean, timeout: Double): Action {
        var startTime: Double = -1.0
        return Action {
            p: TelemetryPacket ->
            if (startTime == -1.0) {
                startTime = runtime.seconds()
            }
            val shouldRun = timeout == -1.0 || runtime.seconds() - startTime < timeout
            return@Action func(shouldRun, p) && shouldRun
        }

    }

    fun resetEncoders() {
        rightPivot.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftPivot.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightExtend.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftExtend.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightPivot.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        leftPivot.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightExtend.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        leftExtend.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun wristRollAction(angle: () -> Double, timeout: Double): Action {
        return timedAction({
            shouldRun, p ->
            wristRoll = angle.invoke()
            return@timedAction true
        }, timeout)
    }
    fun wristRollAction(angle: Double, timeout: Double): Action {
        return timedAction({
                shouldRun, p ->
            wristRoll = angle
            return@timedAction true
        }, timeout)
    }
    fun wristPitchAction(angle: Double, timeout: Double): Action {
        return timedAction({
                shouldRun, p ->
            wristPitch = angle
            return@timedAction true
        }, timeout)
    }
    fun intakeAction(speed: Double, timeout: Double = 0.0): Action {
        return timedAction({
                shouldRun, p ->
            intakeSpeed = if (shouldRun || timeout <= 0.0) speed else 0.0
            return@timedAction true
        }, timeout)
    }
    fun intakeSpinAction(speed: Double, timeout: Double): Action {
        return timedAction({
                shouldRun, p ->
            intakeSpin = if (shouldRun) speed else 0.0
            return@timedAction true
        }, timeout)
    }

    fun driveAction(command: PoseVelocity2d, timeout: Double): Action {
        return timedAction({
                shouldRun, p ->
            this.driveCommand = if (shouldRun || timeout <= 0.0) command else PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
            return@timedAction true
        }, timeout)
    }

    fun init() {
        val motors = hardwareMap.getAll(DcMotor::class.java)
        motors.forEach { motor -> motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER }

        hubs.forEach { hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL) }

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
        leftPivot.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightPivot.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        pivotEncoder.direction = DcMotorSimple.Direction.REVERSE

        // + -> extend
        leftExtend.direction = DcMotorSimple.Direction.FORWARD
        rightExtend.direction = DcMotorSimple.Direction.FORWARD
        leftExtend.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightExtend.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // + -> counter-clockwise, when viewed from right
        leftDiffy.direction = Servo.Direction.REVERSE
        rightDiffy.direction = Servo.Direction.FORWARD

        // + -> intake
        leftIntake.direction = DcMotorSimple.Direction.REVERSE
        rightIntake.direction = DcMotorSimple.Direction.FORWARD

        imu.initialize(
            IMU.Parameters(
                RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
            )
        )

        runtime.reset()



    }

    fun startingPose() {
        targetPivotAngle = 0.6
        wristPitch = 1.8
        wristRoll = 0.0
        targetSlideExtension = 0.0
    }

    fun update() {

        // apply bounds to pivot and extension
        hubs.forEach { hub -> hub.clearBulkCache() }

        targetPivotAngle = clamp(targetPivotAngle, -0.2, PI)
        targetSlideExtension = clamp(targetSlideExtension, -1.0, 1.3)
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

        val slidesVoltage = if (targetSlideExtension < -0.5) -14.0
            else if (useSlidePID) pidSlidesVoltage
            else feedforwardSlidesVoltage
        rightPivot.power = voltageToPower(PivotController.voltage)
        leftPivot.power = voltageToPower(PivotController.voltage)

        rightDiffy.position = 0.5 + (wristPitch + wristRoll/2 + WRIST_PITCH_OFFSET + WRIST_ROLL_OFFSET/2) * WRIST_UNITS_PER_RAD
        leftDiffy.position = 0.5 + (wristPitch - wristRoll/2 + WRIST_PITCH_OFFSET - WRIST_ROLL_OFFSET/2) * WRIST_UNITS_PER_RAD

        leftPTO.position = if (ptoActive) HardwareConstants.PTO_ACTIVE_POSITION else 0.5
        rightIntake.power = intakeSpeed + intakeSpin;
        leftIntake.power = intakeSpeed - intakeSpin * 0.75;

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

        if (ptoActive) {
            val voltage = slidesVoltage/12.0//voltageToPower(slidesVoltage)
            frontRightDrive.power = -voltage
            backRightDrive.power = -voltage
            backLeftDrive.power = -voltage
            frontLeftDrive.power = -voltage
            rightExtend.power = voltage
            leftExtend.power = voltage
        } else if (commandDrivetrain) {
            frontRightDrive.power = voltageToPower(frontRightPower * DRIVE_MAX_VOLTAGE)
            backRightDrive.power = voltageToPower(backRightPower * DRIVE_MAX_VOLTAGE)
            backLeftDrive.power = voltageToPower(backLeftPower * DRIVE_MAX_VOLTAGE)
            frontLeftDrive.power = voltageToPower(frontLeftPower * DRIVE_MAX_VOLTAGE)
        }

        if (!ptoActive) {
            rightExtend.power = voltageToPower(slidesVoltage)
            leftExtend.power = voltageToPower(slidesVoltage)
        }

        telemetry.addData("Target slide extension", targetSlideExtension)
        telemetry.addData("Slides voltage", slidesVoltage)
        telemetry.addData("Right slide power", rightExtend.power)
        telemetry.addData("Left slide power", leftExtend.power)
        telemetry.addData("Slide extension", getCurrentSlideExtension())
        telemetry.addData("Slide setpoint", SlidesController.setPoint)
        telemetry.addData("Pivot angle", getCurrentPivotAngle())
        telemetry.addData("Pivot voltage", pivotVoltage)

    }

}