package org.firstinspires.ftc.teamcode.util

import android.util.Size
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.RaceAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.vision.VisionPortal
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

class SubVisionHelper(val hardware: RobotHardware) {

    var processor: SampleDetectionProcessor = SampleDetectionProcessor()
    val hardwareMap = hardware.hardwareMap
    val autoHelper = AutoHelper(hardware)
    var portal: VisionPortal

    init {
        portal = VisionPortal.Builder()
            .addProcessor(processor)
            .setCameraResolution(Size(320, 240))
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .build()
        portal.setProcessorEnabled(processor, false)
        portal.resumeStreaming()
        portal.resumeLiveView()
    }

    fun visionEnable() {
        portal.setProcessorEnabled(processor, true)
    }

    fun visionDisable() {
        portal.setProcessorEnabled(processor, false)
    }

    fun visionPoseAction(): Action {
        return RaceAction(
            ParallelAction(
                hardware.wristPitchAction(-Math.PI/4, 0.0),
                hardware.wristRollAction(0.0, 0.0),
                hardware.pivotToAngleAction(0.5, 0.05),
                hardware.slideToPosAction(0.35, 0.05)
            ),
            SleepAction(1.25)
        )
    }

    fun readyToIntakePoseAction(extension: () -> Double = { 0.35 }): Action {
        return RaceAction(
            ParallelAction(
                hardware.pivotToAngleAction(0.2, 0.1),
                hardware.wristPitchAction(-Math.PI/2, 0.0),
                hardware.slideToPosAction(extension.invoke(), 0.045)
            ),
            SleepAction(1.0)
        )
    }

    fun setVisionEnabledAction(enabled: Boolean): Action {
        return Action {
            if (enabled) { visionEnable() } else { visionDisable() }
            return@Action false
        }
    }

    fun autoIntakeAction(allowableColors: List<SampleColor>, preferableColors: List<SampleColor>, mecanumDrive: MecanumDrive, repeat: Boolean = true): Action {
        var detectedPose = Pose2d(0.0, 0.0, 0.0)
        var driveAction = autoHelper.driveToPointAction(
            mecanumDrive,
            { Pose2d(0.0, detectedPose.position.y, 0.0) },
            CoordinateSpace.ROBOT,
            0.4,
            Math.toRadians(3.0),
            3.0,
            Math.toRadians(30.0))
        var wristRollAction = hardware.wristRollAction({ detectedPose.heading.toDouble() }, 0.1)
        var extendAction = RaceAction(
            ParallelAction(
                hardware.pivotToAngleAction(0.2, 0.1),
                hardware.wristPitchAction(-Math.PI/2, 0.0),
                hardware.slideToPosAction({ min(0.5, 0.375 +
                        detectedPose.position.x * HardwareConstants.SLIDES_EXTENSION_PER_IN -
                        1.0 * HardwareConstants.SLIDES_EXTENSION_PER_IN) }, 0.045)
            ),
            SleepAction(1.0)
        )
        var finalAction = Action { return@Action false }
        val timer = ElapsedTime()
        var detecting = false
        var detectionFailed = false

        return SequentialAction(
            Action { p ->
                processor.preferableColors = preferableColors
                return@Action false
            },
            setVisionEnabledAction(true),
            visionPoseAction(),
            Action { p ->
                if (!detecting) {
                    detecting = true
                    timer.reset()
                }
                val detection = getLeastEvilDetection(allowableColors)
                if (detection == null) {
                    mecanumDrive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.2), 0.0))
                    detectionFailed = true
                    hardware.telemetry.addData("No detections so sad", ":(")
                    hardware.telemetry.update()
                    return@Action timer.seconds() < 1.0
                }
                detectionFailed = false
                mecanumDrive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
                detectedPose = detection.pose
                hardware.telemetry.addData("Detected pose X", detectedPose.position.x)
                hardware.telemetry.addData("Detected pose Y", detectedPose.position.y)
                hardware.telemetry.addData("Detected pose heading", detectedPose.heading)
                hardware.telemetry.update()
                return@Action false
            },
            setVisionEnabledAction(false),
            SequentialAction(
                ParallelAction(
                    driveAction,
                    wristRollAction,
                    hardware.wristPitchAction(-Math.PI/2.0, 0.4),
                    hardware.slideToPosAction(0.2),
                    hardware.pivotToAngleAction(0.25, 0.1)
                ),
                extendAction,
                RaceAction(
                    hardware.pivotToAngleAction(-.2, 0.25),
                    hardware.intakeAction(1.0, 2.0)
                ),
                hardware.intakeAction(1.0, 1.0),
                ParallelAction(
                    hardware.pivotToAngleAction(0.2),
                    hardware.slideToPosAction(0.1),
                    hardware.wristPitchAction(0.0, 0.0),
                    hardware.intakeAction(1.0, 0.3)
                ),
                hardware.intakeAction(0.0, 0.0),
                Action { p ->
                    hardware.telemetry.addData("lol", hardware.getSampleInsideIntake())
                    hardware.telemetry.update()
                    if (hardware.getSampleInsideIntake() !in allowableColors) {
                        finalAction = hardware.intakeAction(-1.0, 0.5)
                    }
                    return@Action false
                },
                Action { p -> finalAction.run(p) }
            )
        )
    }

    fun getLeastEvilDetection(allowableColors: List<SampleColor>): SampleDetection? {
        if (processor.detections == null) { return null }
        return SubVisionSingleton.getLeastEvilDetection(processor.detections!!, allowableColors)
    }

}

object SubVisionSingleton {

    fun getEvilPoses(detections: List<SampleDetection>, allowableColors: List<SampleColor>): ArrayList<Pose2d> {
        val evilPoses = ArrayList<Pose2d>()

        // get evil poses
        for (detection in detections) {
            if (!allowableColors.contains(detection.color)) {
                evilPoses.add(detection.pose)
            }
        }

        return evilPoses
    }

    fun getDetectionEvilness(detection: SampleDetection, evilPoses: List<Pose2d>): Double {
        var evilness = 0.0
        for (evilPose in evilPoses) {

            val distance = (evilPose.position - detection.pose.position).norm()

            val distanceContribution = MathUtils.mapRange(distance, 1.5, 3.5, 2.0, 0.0, true)
            val headingContribution = MathUtils.mapRange(
                Math.toDegrees(abs(MathUtils.wrapAngle(detection.pose.heading.toDouble() - evilPose.heading.toDouble()))),
                45.0,
                90.0,
                1.0,
                0.0,
                true
            ) * MathUtils.mapRange(distance, 1.5, 2.5, 1.0, 0.0, true)

            val BORDER_SIZE = 70.0
            val BORDER_MAX_EVILNESS = 0.8
            val edgeClosenessContribution = MathUtils.mapRange(detection.boundingBox.center.y, 0.0,  BORDER_SIZE, BORDER_MAX_EVILNESS, 0.0, true) +
                    MathUtils.mapRange(detection.boundingBox.center.y, 240.0,  240.0 - BORDER_SIZE, BORDER_MAX_EVILNESS, 0.0, true) +
                    MathUtils.mapRange(detection.boundingBox.center.x, 0.0,  BORDER_SIZE, BORDER_MAX_EVILNESS, 0.0, true) +
                    MathUtils.mapRange(detection.boundingBox.center.x, 320.0,  320.0 - BORDER_SIZE, BORDER_MAX_EVILNESS, 0.0, true)

            evilness += distanceContribution + headingContribution + edgeClosenessContribution
        }
        return evilness
    }

    fun getLeastEvilDetection(detections: List<SampleDetection>, allowableColors: List<SampleColor>): SampleDetection? {

        val evilPoses = getEvilPoses(detections, allowableColors)

        var minEvilness: Double? = null
        var minEvilnessDetection: SampleDetection? = null
        for (detection in detections) {

            if (!allowableColors.contains(detection.color)) { continue }

            val evilness = getDetectionEvilness(detection, evilPoses)
            if (minEvilness == null || evilness < minEvilness) {
                minEvilness = evilness
                minEvilnessDetection = detection
            }

        }

        return minEvilnessDetection

    }

}