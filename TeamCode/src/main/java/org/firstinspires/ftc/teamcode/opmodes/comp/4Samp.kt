package org.firstinspires.ftc.teamcode.opmodes.comp

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.TranslationalVelConstraint
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.util.AutoHelper
import org.firstinspires.ftc.teamcode.util.plus
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.Params
import org.firstinspires.ftc.teamcode.util.HardwareConstants
import org.firstinspires.ftc.teamcode.util.MathUtils.gridToFieldCoords
import org.firstinspires.ftc.teamcode.util.RobotHardware

// 0,0 in "grid" is red alliance's net zone
// +5,0 is red alliance's observation zone
// 0,+5 is blue alliance
@Autonomous
class `4Samp` : LinearOpMode() {

    private lateinit var hardware: RobotHardware
    private lateinit var autoHelper: AutoHelper
    private val SCORING_OUTTAKE_TIMEOUT = 1.0
    private val OUTTAKE_SPEED = -0.2;

    override fun runOpMode() {
        val beginPose = gridToFieldCoords(1.0, 0.0, Math.PI / 2).plus(
            -HardwareConstants.DISTANCE_BETWEEN_CHASSIS_AND_WALL_AT_TILE_CENTER,
            -HardwareConstants.DISTANCE_BETWEEN_CHASSIS_AND_WALL_AT_TILE_CENTER,
            0.0
        )
        val drive = MecanumDrive(hardwareMap, beginPose)

        hardware = RobotHardware(hardwareMap, telemetry)
        hardware.commandDrivetrain = false
        hardware.init();

        autoHelper = AutoHelper(hardware)

        // in an ideal universe, these constraints would be individually tuned for each step of the auto, but I don't feel like doing allat
        val slowDownConstraint = autoHelper.rampVelConstraint(MecanumDrive.PARAMS.maxWheelVel, 15.0, 0.0, 0.5) // used when going up
        val speedUpConstraint = autoHelper.rampVelConstraint(15.0, MecanumDrive.PARAMS.maxWheelVel, 0.2, 0.5) // used when going down

        waitForStart()
        hardware.imu.resetYaw()

        val autoAction = SequentialAction(

            // score preload
            ParallelAction(
                hardware.wristPitchAction(0.0, 0.0),
                hardware.wristRollAction(0.0, 0.0),
                autoHelper.readyToScoreInBasketAction(),
                drive.actionBuilder(beginPose)
                    .setTangent(3 * Math.PI/4)
                    .splineToSplineHeading(gridToFieldCoords(0.18, 0.19, Math.PI / 4), 3 * Math.PI/4, slowDownConstraint)
                    .build()
            ),
            hardware.intakeAction(OUTTAKE_SPEED, SCORING_OUTTAKE_TIMEOUT),


            // grab 2nd sample
            ParallelAction(
                autoHelper.readyToGrabGamePieceAction(0.3, 0.0),
                drive.actionBuilder(drive.pose)
                    .setTangent(Math.PI/2)
                    .splineToLinearHeading(gridToFieldCoords(0.49, 0.5, Math.PI/2), Math.PI/2, TranslationalVelConstraint(15.0))
                    .build()
            ),
            autoHelper.grabGamePieceAction(),

            // score 2nd sample
            ParallelAction(
                autoHelper.readyToScoreInBasketAction(),
                drive.actionBuilder(drive.pose)
                    .setTangent(-Math.PI/2)
                    .splineToLinearHeading(gridToFieldCoords(0.18, 0.19, Math.PI / 4), -Math.PI/2, slowDownConstraint)
                    .build()
            ),
            hardware.intakeAction(OUTTAKE_SPEED, SCORING_OUTTAKE_TIMEOUT),

            // grab 3rd sample
            ParallelAction(
                autoHelper.readyToGrabGamePieceAction(0.3, 0.0),
                drive.actionBuilder(drive.pose)
                    .setTangent(Math.PI/2)
                    .splineToLinearHeading(gridToFieldCoords(-0.03, 0.5, Math.PI/2), Math.PI/2, TranslationalVelConstraint(15.0))
                    .build()
            ),
            autoHelper.grabGamePieceAction(),

            // score 3rd sample
            ParallelAction(
                autoHelper.readyToScoreInBasketAction(),
                drive.actionBuilder(drive.pose)
                    .setTangent(-Math.PI/2)
                    .splineToLinearHeading(gridToFieldCoords(0.18, 0.19, Math.PI / 4), -Math.PI/2, slowDownConstraint)
                    .build()
            ),
            hardware.intakeAction(OUTTAKE_SPEED, SCORING_OUTTAKE_TIMEOUT),

            // grab 4th sample
            ParallelAction(
                autoHelper.readyToGrabGamePieceAction(0.1, 4.0),
                drive.actionBuilder(drive.pose)
                    .setTangent(Math.PI/2)
                    .splineToLinearHeading(gridToFieldCoords(0.17, 1.48, Math.PI), 2 * Math.PI/3, TranslationalVelConstraint(15.0))
                    .build()
            ),
            autoHelper.grabGamePieceAction(),

            // score 4th sample
            ParallelAction(
                ParallelAction(
                    SleepAction(2.0),
                    autoHelper.readyToScoreInBasketAction()
                ),
                drive.actionBuilder(drive.pose)
                    .setTangent(-Math.toRadians(90.0))
                    .splineToLinearHeading(gridToFieldCoords(0.18, 0.18, Math.PI / 4), -3 * Math.PI/4,
                        MinVelConstraint(listOf(
                          TranslationalVelConstraint(8.0),
                            AngularVelConstraint(Math.PI/8)
                        )))
                    .build()
            ),
            hardware.intakeAction(OUTTAKE_SPEED, SCORING_OUTTAKE_TIMEOUT),

            // park
            ParallelAction(
                autoHelper.readyToLvl1AscentAction(),
                drive.actionBuilder(drive.pose)
                    .setTangent(Math.PI/2)
                    .splineToSplineHeading(gridToFieldCoords(1.7, 2.3, Math.PI), 0.0, speedUpConstraint)
                    .build()
            )

        )

        runBlocking(
            ParallelAction(
                Action {
                    hardware.update()
                    return@Action true },
                autoAction
            )
        )
    }
}