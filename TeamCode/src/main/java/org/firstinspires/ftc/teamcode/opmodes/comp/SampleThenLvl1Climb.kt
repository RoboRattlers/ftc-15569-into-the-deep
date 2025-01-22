package org.firstinspires.ftc.teamcode.opmodes.comp

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.TranslationalVelConstraint
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.util.AutoHelper
import org.firstinspires.ftc.teamcode.util.plus
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.util.HardwareConstants
import org.firstinspires.ftc.teamcode.util.MathUtils.gridToFieldCoords
import org.firstinspires.ftc.teamcode.util.RobotHardware

// 0,0 in "grid" is red alliance's net zone
// +5,0 is red alliance's observation zone
// 0,+5 is blue alliance
@Autonomous
class SampleThenLvl1Climb : LinearOpMode() {

    private lateinit var hardware: RobotHardware
    private lateinit var autoHelper: AutoHelper
    private val SCORING_OUTTAKE_TIMEOUT = 0.5;

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

        // in an ideal universe, these constraints would be individiually tuned for each step of the auto, but I don't feel like doing allat
        val slowDownConstraint = autoHelper.rampVelConstraint(MecanumDrive.PARAMS.maxWheelVel, 15.0, 0.0, 0.5) // used when going up
        val speedUpConstraint = autoHelper.rampVelConstraint(15.0, MecanumDrive.PARAMS.maxWheelVel, 0.5, 1.0) // used when going down

        waitForStart()

        runBlocking(
            ParallelAction(
                Action {
                    hardware.update()
                    return@Action true },
                drive.actionBuilder(beginPose)
                    // initialize
                    .setTangent(3 * Math.PI/4)
                    .afterTime(0.0, hardware.wristPitchAction(0.0, 0.0))
                    .afterTime(0.0, hardware.wristRollAction(0.0, 0.0))

                    // place preload
                    .afterTime(0.1, autoHelper.readyToScoreInBasketAction())
                    .splineToSplineHeading(gridToFieldCoords(0.15, 0.15, Math.PI / 4), 3 * Math.PI/4, slowDownConstraint)
                    .waitSeconds(0.0) // for some reason, RR doesn't wait for actions in the middle of the path to complete, so we have to do it ourselves
                    .stopAndAdd(hardware.intakeAction(-1.0, SCORING_OUTTAKE_TIMEOUT))

                    // grab 2nd sample
                    .afterTime(0.0, autoHelper.readyToGrabGamePieceAction(0.3))
                    .splineTo(gridToFieldCoords(0.5, 0.5), Math.PI/2, TranslationalVelConstraint(15.0))
                    .waitSeconds(0.0)
                    .stopAndAdd(autoHelper.grabGamePieceAction())
                    .setTangent(-Math.PI/2)

                    // place 2nd sample
                    .afterTime(0.0, autoHelper.readyToScoreInBasketAction())
                    .splineToLinearHeading(gridToFieldCoords(0.15, 0.15, Math.PI / 4), -Math.PI/2, slowDownConstraint)
                    .waitSeconds(0.0)
                    .stopAndAdd(hardware.intakeAction(-1.0, SCORING_OUTTAKE_TIMEOUT))

                    // lvl 1 ascent
                    .afterTime(0.0, hardware.slideToPosAction(0.0))
                    .splineToSplineHeading(gridToFieldCoords(1.7, 2.0, Math.PI), 0.0, speedUpConstraint)
                    .stopAndAdd(hardware.pivotToAngleAction(1.5))
                    .build()
            )
        )
    }
}