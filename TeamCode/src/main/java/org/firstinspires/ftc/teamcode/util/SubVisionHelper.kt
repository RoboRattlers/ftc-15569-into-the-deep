package org.firstinspires.ftc.teamcode.util

import android.util.Size
import android.view.textclassifier.TextClassifierEvent.TextLinkifyEvent
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.RaceAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.vision.VisionPortal
import kotlin.math.sign
import kotlin.math.tan

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

        portal.setProcessorEnabled(processor, true)
    }

    fun enable() {
        portal.resumeStreaming()
        portal.resumeLiveView()
    }

    fun disable() {
        portal.stopStreaming()
        portal.stopLiveView()
    }

    fun visionPoseAction(): Action {
        return ParallelAction(
            hardware.wristPitchAction(0.0, 0.0),
            hardware.wristRollAction(0.0, 0.0),
            hardware.pivotToAngleAction(0.7, 0.12),
            hardware.slideToPosAction(0.35, 0.05)
        )
    }

    fun readyToIntakePoseAction(): Action {
        return ParallelAction(
            hardware.pivotToAngleAction(0.3, 0.03),
            hardware.wristPitchAction(-Math.PI/2, 0.0),
            hardware.slideToPosAction(0.35, 0.03)
        )
    }

    fun setVisionEnabledAction(enabled: Boolean): Action {
        return Action {
            if (enabled) { enable() } else { disable() }
            return@Action false
        }
    }

    fun autoIntakeAction(color: SampleColor, mecanumDrive: MecanumDrive): Action {
        var detectedPose = Pose2d(0.0, 0.0, 0.0)
        var driveAction = mecanumDrive.actionBuilder(Pose2d(0.0, 0.0, 0.0)).build()
        var wristRollAction = hardware.wristRollAction(0.0, 0.0)
        return SequentialAction(
            setVisionEnabledAction(true),
            visionPoseAction(),
            Action {
                val detection = getCentermostDetection(color)
                detectedPose = detection?.pose ?: Pose2d(0.0, 0.0, 0.0)
                val tangent = Math.atan2(detectedPose.position.y, detectedPose.position.x)
                driveAction = autoHelper.driveToPointAction(mecanumDrive, detectedPose, CoordinateSpace.ROBOT, 0.05, 0.05)
                wristRollAction = hardware.wristRollAction(detectedPose.heading.toDouble(), 0.0)
                return@Action false
            },
            ParallelAction(
                Action {
                    return@Action driveAction.run(TelemetryPacket())
                },
                Action {
                    return@Action wristRollAction.run(TelemetryPacket())
                },
                hardware.wristPitchAction(-1.5, 0.5),
            ),
            readyToIntakePoseAction(),
            RaceAction(
                hardware.pivotToAngleAction(0.0, 0.1),
                hardware.intakeAction(1.0, 1.0)
            ),
            setVisionEnabledAction(false),
            hardware.wristPitchAction(0.0, 0.0),
            hardware.pivotToAngleAction(0.3)
        )
    }

    fun getCentermostDetection(color: SampleColor): SampleDetection? {

        var minDist: Double? = null
        var minDistDetection: SampleDetection? = null

        if (processor.detections != null) {
            for (detection in processor.detections!!) {
                if (detection.color != color) continue
                val dist = detection.pose.position.norm()
                if (minDist == null || dist < minDist) {
                    minDist = dist
                    minDistDetection = detection
                }
            }
        }

        return minDistDetection

    }

}