package org.firstinspires.ftc.teamcode.opmodes.comp

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.RaceAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.TranslationalVelConstraint
import com.acmerobotics.roadrunner.VelConstraint
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.util.AutoHelper
import org.firstinspires.ftc.teamcode.util.componentWisePlus
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.util.HardwareConstants
import org.firstinspires.ftc.teamcode.util.MathUtils.gridToFieldCoords
import org.firstinspires.ftc.teamcode.util.RobotHardware
import org.firstinspires.ftc.teamcode.util.SubVisionHelper
import org.firstinspires.ftc.teamcode.util.transformBy

// 0,0 in "grid" is red alliance's net zone
// +5,0 is red alliance's observation zone
// 0,+5 is blue alliance

enum class SplineType {
    SplineHeading,
    LinearHeading
}

@Autonomous
open class `5Spec` : LinearOpMode() {

    private lateinit var hardware: RobotHardware
    private lateinit var autoHelper: AutoHelper
    private lateinit var subVisionHelper: SubVisionHelper
    private val SCORING_OUTTAKE_TIMEOUT = 0.25
    private val OUTTAKE_SPEED = -0.4;
    private val PICKUP_POSE = gridToFieldCoords(3.75, 0.0, -Math.PI/2)

    fun getChamberPose(offset: Double): Pose2d {
        return gridToFieldCoords(2.5, 1.0, -Math.PI/2).componentWisePlus(Pose2d(offset, 0.0, 0.0))
    }

    override fun runOpMode() {
        val beginPose = gridToFieldCoords(3.0, 0.0, Math.PI / 2).componentWisePlus(
            HardwareConstants.DISTANCE_BETWEEN_CHASSIS_AND_WALL_AT_TILE_CENTER,
            -HardwareConstants.DISTANCE_BETWEEN_CHASSIS_AND_WALL_AT_TILE_CENTER,
            0.0
        )
        val drive = MecanumDrive(hardwareMap, beginPose)

        hardware = RobotHardware(hardwareMap, telemetry)
        hardware.commandDrivetrain = false
        hardware.init();

        autoHelper = AutoHelper(hardware)

        while (opModeInInit()) {
            hardware.startingPose()
            hardware.update()
            telemetry.update()
        }

        hardware.imu.resetYaw()

        var lastPose = beginPose
        fun generateSplineAction(pose: Pose2d, startTangent: Double, endTangent: Double, constraint: VelConstraint?, splineType: SplineType = SplineType.LinearHeading): Action {
            if (splineType == SplineType.LinearHeading) {
                val action = drive.actionBuilder(lastPose)
                    .setTangent(startTangent)
                    .splineToLinearHeading(pose, endTangent, constraint)
                    .build()
                lastPose = pose
                return action
            } else {
                val action = drive.actionBuilder(lastPose)
                    .setTangent(startTangent)
                    .splineToSplineHeading(pose, endTangent, constraint)
                    .build()
                lastPose = pose
                return action
            }
        }

        fun generateSplineAction(pose: Pose2d, tangent: Double, constraint: VelConstraint?, splineType: SplineType = SplineType.LinearHeading): Action {
            return generateSplineAction(pose, tangent, tangent, constraint, splineType)
        }

        fun generateRotateAction(heading: Double): Action {
            val action = drive.actionBuilder(lastPose)
                .turnTo(heading)
                .build()
            lastPose = Pose2d(lastPose.position.x, lastPose.position.y, heading)
            return action
        }

        fun grabPresetUnit(pose: Pose2d, startTangent: Double, endTangent: Double, extension: Double, roll: Double): Action {
            return SequentialAction(
                ParallelAction(
                    autoHelper.readyToGrabGamePieceAction(extension, roll),
                    generateSplineAction(
                        pose,
                        startTangent,
                        endTangent,
                        TranslationalVelConstraint(35.0)
                    )
                ),
                autoHelper.grabGamePieceAction(),
                // place in zone
                ParallelAction(
                    hardware.wristRollAction(0.0, 0.0),
                    hardware.wristPitchAction(0.0, 0.0),
                    hardware.slideToPosAction(0.5),
                    generateRotateAction(-Math.PI/2)
                ),
                hardware.intakeAction(OUTTAKE_SPEED, SCORING_OUTTAKE_TIMEOUT)
            )
        }

        var numScored = -1
        fun scoreUnit(): Action {
            numScored++
            return SequentialAction(
                // score
                ParallelAction(
                    generateSplineAction(
                        getChamberPose(2.0 * numScored.toDouble()),
                        Math.PI/2,
                        null,
                        SplineType.LinearHeading
                    ),
                    SequentialAction(
                        SleepAction(0.6),
                        autoHelper.readyToScoreSpecimenAction()
                    )
                ),
                autoHelper.scoreSpecimenAction(drive),
                // grab
                ParallelAction(
                    autoHelper.readyToGrabSpecimenAction(),
                    SequentialAction(
                        SleepAction(0.5),
                        hardware.intakeAction(1.0)
                    ),
                    generateSplineAction(PICKUP_POSE, -Math.PI/2, -Math.PI/2, null, SplineType.LinearHeading)
                ),
                hardware.intakeAction(0.0)
            )
        }

        val autoAction = SequentialAction(

            // score preload
            ParallelAction(
                hardware.wristPitchAction(-0.25, 0.0),
                hardware.wristRollAction(0.0, 0.0),
                hardware.pivotToAngleAction(1.0),
                hardware.slideToPosAction(0.35),
                generateSplineAction(gridToFieldCoords(2.5, 1.0, Math.PI/2).componentWisePlus(0.0, -2.0, 0.0), Math.PI/2, TranslationalVelConstraint(40.0))
            ),
            hardware.pivotToAngleAction(0.8),
            autoHelper.timeoutAction(hardware.slideToPosAction(0.1), 0.5),
            hardware.intakeAction(OUTTAKE_SPEED, SCORING_OUTTAKE_TIMEOUT),
            grabPresetUnit(gridToFieldCoords(3.5, 0.5, Math.PI/4), -Math.PI/2, Math.toRadians(10.0), 0.38, -Math.PI/6),
            grabPresetUnit(gridToFieldCoords(4.0, 0.5, Math.PI/4), 0.0, 0.0, 0.38, -Math.PI/6),
            grabPresetUnit(gridToFieldCoords(4.5, 0.5, Math.PI/4), 0.0, 0.0, 0.38, -Math.PI/6),
            ParallelAction(
                autoHelper.readyToGrabSpecimenAction(),
                hardware.intakeAction(1.0),
                generateSplineAction(PICKUP_POSE, 5 * Math.PI/6, -Math.PI/2, null)
            ),
            scoreUnit(),
            scoreUnit(),
            scoreUnit(),
            scoreUnit()
        )

        runBlocking(
            RaceAction(
                Action {
                    hardware.update()
                    telemetry.update()
                    return@Action true },
                autoAction
            )
        )

    }
}