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
import org.firstinspires.ftc.teamcode.util.SampleColor
import org.firstinspires.ftc.teamcode.util.SubVisionHelper

// 0,0 in "grid" is red alliance's net zone
// +5,0 is red alliance's observation zone
// 0,+5 is blue alliance
@Autonomous
open class `5SampBlue` : LinearOpMode() {

    private lateinit var hardware: RobotHardware
    private lateinit var autoHelper: AutoHelper
    private lateinit var subVisionHelper: SubVisionHelper
    private val SCORING_OUTTAKE_TIMEOUT = 0.35
    private val OUTTAKE_SPEED = -0.3;
    private val BASKET_POSE = gridToFieldCoords(0.145, 0.162, Math.PI / 3)
    var VALID_COLORS = listOf(SampleColor.BLUE, SampleColor.YELLOW)

    override fun runOpMode() {
        val beginPose = gridToFieldCoords(1.0, 0.0, Math.PI / 2).componentWisePlus(
            -HardwareConstants.DISTANCE_BETWEEN_CHASSIS_AND_WALL_AT_TILE_CENTER,
            -HardwareConstants.DISTANCE_BETWEEN_CHASSIS_AND_WALL_AT_TILE_CENTER,
            0.0
        )
        val drive = MecanumDrive(hardwareMap, beginPose)

        hardware = RobotHardware(hardwareMap, telemetry)
        hardware.commandDrivetrain = false
        hardware.init();

        autoHelper = AutoHelper(hardware)
        subVisionHelper = SubVisionHelper(hardware)

        // in an ideal universe, these constraints would be individually tuned for each step of the auto, but I don't feel like doing allat
        val slowDownConstraint = autoHelper.rampVelConstraint(MecanumDrive.PARAMS.maxWheelVel, 22.0, 0.0, 0.5) // used when going up
        val speedUpConstraint = autoHelper.rampVelConstraint(22.0, MecanumDrive.PARAMS.maxWheelVel, 0.1, 0.3) // used when going down

        while (opModeInInit()) {
            hardware.startingPose()
            hardware.update()
            telemetry.update()
        }

        hardware.imu.resetYaw()

        var lastPose = beginPose
        fun generateSplineAction(pose: Pose2d, startTangent: Double, endTangent: Double, constraint: VelConstraint?): Action {
            val action = drive.actionBuilder(lastPose)
                .setTangent(startTangent)
                .splineToLinearHeading(pose, endTangent, constraint)
                .build()
            lastPose = pose
            return action
        }

        fun generateSplineAction(pose: Pose2d, tangent: Double, constraint: VelConstraint?): Action {
            return generateSplineAction(pose, tangent, tangent, constraint)
        }

        val autoAction = SequentialAction(

            // score preload
            ParallelAction(
                hardware.wristPitchAction(0.0, 0.0),
                hardware.wristRollAction(0.0, 0.0),
                autoHelper.readyToScoreInBasketAction(),
                generateSplineAction(BASKET_POSE, 3 * Math.PI/4, 5 * Math.PI/4, slowDownConstraint)
            ),
            hardware.intakeAction(OUTTAKE_SPEED, SCORING_OUTTAKE_TIMEOUT),


            // grab 2nd sample
            ParallelAction(
                autoHelper.readyToGrabGamePieceAction(0.393, 0.0),
                generateSplineAction(gridToFieldCoords(0.51, 0.5, Math.PI/2), Math.PI/2, TranslationalVelConstraint(25.0))
            ),
            autoHelper.grabGamePieceAction(),

            // score 2nd sample
            ParallelAction(
                autoHelper.readyToScoreInBasketAction(),
                generateSplineAction(BASKET_POSE, -Math.PI/2, slowDownConstraint)
            ),
            hardware.intakeAction(OUTTAKE_SPEED, SCORING_OUTTAKE_TIMEOUT),

            // grab 3rd sample
            ParallelAction(
                autoHelper.readyToGrabGamePieceAction(0.393, 0.0),
                generateSplineAction(gridToFieldCoords(0.02, 0.5, Math.PI/2), Math.PI/2, TranslationalVelConstraint(25.0))
            ),
            autoHelper.grabGamePieceAction(),

            // score 3rd sample
            ParallelAction(
                autoHelper.readyToScoreInBasketAction(),
                generateSplineAction(BASKET_POSE, -Math.PI/2, slowDownConstraint)
            ),
            hardware.intakeAction(OUTTAKE_SPEED, SCORING_OUTTAKE_TIMEOUT),

            // grab 4th sample
            ParallelAction(
                autoHelper.readyToGrabGamePieceAction(0.41, -Math.PI/6),
                generateSplineAction(gridToFieldCoords(-0.1, 0.5, Math.toRadians(110.0)), Math.PI/2, TranslationalVelConstraint(25.0))
            ),
            autoHelper.grabGamePieceAction(),

            // score 4th sample
            ParallelAction(
                autoHelper.readyToScoreInBasketAction(),
                generateSplineAction(BASKET_POSE, -Math.PI/2, TranslationalVelConstraint(25.0))
            ),
            hardware.intakeAction(OUTTAKE_SPEED, SCORING_OUTTAKE_TIMEOUT),
            hardware.wristPitchAction(0.0, 0.0),

            // grab 5th sample
            ParallelAction(
                hardware.slideToPosAction(0.0),
                hardware.pivotToAngleAction(0.5),
                generateSplineAction(gridToFieldCoords(1.4, 2.2, 0.0), Math.PI/2, 0.0, speedUpConstraint)
            ),
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

        while (hardware.getSampleInsideIntake() !in VALID_COLORS && opModeIsActive()) {
            runBlocking(
                RaceAction(
                    Action {
                        hardware.update()
                        telemetry.update()
                        return@Action true },
                    subVisionHelper.autoIntakeAction(VALID_COLORS, VALID_COLORS, drive)
                )
            )
        }

        if (hardware.getSampleInsideIntake() !in VALID_COLORS) { return }

        runBlocking(
            RaceAction(
                Action {
                    hardware.update()
                    telemetry.update()
                    return@Action true },
                SequentialAction(
                    hardware.slideToPosAction(0.05),

                    // score 5th sample
                    ParallelAction(
                        SequentialAction(
                            SleepAction(2.0),
                            autoHelper.readyToScoreInBasketAction()
                        ),
                        generateSplineAction(BASKET_POSE, Math.PI, Math.PI/2, TranslationalVelConstraint(35.0)),
                        Action {
                            val shouldSpin = hardware.getCurrentSlideExtension() > 0.97
                            if (shouldSpin) {
                                hardware.intakeSpeed = -1.0
                            }
                            return@Action !shouldSpin
                        }
                    ),
                    hardware.intakeAction(OUTTAKE_SPEED, SCORING_OUTTAKE_TIMEOUT),
                    hardware.wristPitchAction(0.0, 0.0),

                    // park
                    ParallelAction(
                        autoHelper.readyToLvl1AscentAction(),
                        generateSplineAction(gridToFieldCoords(1.7, 2.3, Math.PI), Math.PI/2, 0.0, TranslationalVelConstraint(35.0))
                    )
                )
            )
        )

    }
}