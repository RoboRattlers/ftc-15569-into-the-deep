package org.firstinspires.ftc.teamcode.opmodes.comp

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.util.AutoHelper
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.util.RobotHardware
import org.firstinspires.ftc.teamcode.util.SampleColor
import org.firstinspires.ftc.teamcode.util.SubVisionHelper

// 0,0 in "grid" is red alliance's net zone
// +5,0 is red alliance's observation zone
// 0,+5 is blue alliance
@Autonomous
class AutoIntakeTest : LinearOpMode() {

    private lateinit var hardware: RobotHardware
    private lateinit var autoHelper: AutoHelper
    private lateinit var subVisionHelper: SubVisionHelper
    private val SCORING_OUTTAKE_TIMEOUT = 1.0
    private val OUTTAKE_SPEED = -0.2;

    override fun runOpMode() {

        val drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))

        hardware = RobotHardware(hardwareMap, telemetry)
        hardware.commandDrivetrain = false
        hardware.init();
        subVisionHelper = SubVisionHelper(hardware)
        subVisionHelper.enable()

        autoHelper = AutoHelper(hardware)

        // in an ideal universe, these constraints would be individually tuned for each step of the auto, but I don't feel like doing allat
        val slowDownConstraint = autoHelper.rampVelConstraint(MecanumDrive.PARAMS.maxWheelVel, 15.0, 0.0, 0.5) // used when going up
        val speedUpConstraint = autoHelper.rampVelConstraint(15.0, MecanumDrive.PARAMS.maxWheelVel, 0.2, 0.5) // used when going down

        waitForStart()
        hardware.imu.resetYaw()

        //val autoAction =

        runBlocking(
            ParallelAction(
                Action {
                    hardware.update()
                    return@Action true },
                subVisionHelper.autoIntakeAction(SampleColor.BLUE, drive)
            )
        )
    }
}