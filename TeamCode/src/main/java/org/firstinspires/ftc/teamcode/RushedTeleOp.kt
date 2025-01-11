/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.MathUtils.round
import org.firstinspires.ftc.teamcode.MathUtils.wrapAngle
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin


/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

enum class TeleOpState {
    INTAKING,
    DRIVING,
    SCORING,
    CLIMB_1_START,
    CLIMB_1_OVEREXTEND,
    CLIMB_1_RETRACT,
    CLIMB_2_START,
    CLIMB_2_OVEREXTEND,
    CLIMB_2_RETRACT
}

@TeleOp(name = "Rushed TeleOp", group = "Iterative OpMode")
class RushedTeleOp : OpMode() {
    // Declare OpMode members.
    private val runtime = ElapsedTime()
    private var stateSwitchTime = 0.0
    private lateinit var hardware : RobotHardware
    private var state : TeleOpState = TeleOpState.DRIVING
        set (value) {
            justSwitchedState = true
            stateSwitchTime = runtime.seconds()
            field = value
        }

    private var lastGamepad1 = Gamepad()
    private var nowGamepad1 = Gamepad()
    private var driveSpeedMult = 1.0
    private var justSwitchedState = true
    private var mayUseHeadingPID = true
    private var useHeadingPID = false
    private var targetHeading = 0.0

    private var climbing = false

    private var scoreHeightIndex = 0
    private val scoreHeights = arrayOf(0.25, 0.35, 0.5, 0.75, 0.92)

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    override fun init() {
        hardware = RobotHardware(hardwareMap, telemetry)
        hardware.init();
    }

    override fun init_loop() {
    }



    override fun start() {
        runtime.reset()
    }

    /*

     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    override fun loop() {

        lastGamepad1.copy(nowGamepad1)
        nowGamepad1.copy(gamepad1)


        val removeJustSwitchedStateLater = justSwitchedState

        // TODO: bring all non-continuous actuators to their zero positions

        when (state) {
            TeleOpState.DRIVING -> {

                if (justSwitchedState) {
                    hardware.wristRoll = 0.0
                    hardware.wristPitch = 1.7
                    hardware.plungerRetracted = false
                    hardware.targetSlideExtension = 0.0
                    hardware.targetPivotAngle = 1.0

                    mayUseHeadingPID = true
                    useHeadingPID = false
                }

                if (abs(nowGamepad1.right_stick_x) > 0.2) {
                    useHeadingPID = false
                }

                if (hardware.getCurrentSlideExtension() < 0.15) {
                    driveSpeedMult = 1.0
                    if (nowGamepad1.dpad_down and !lastGamepad1.dpad_down) {
                        state = TeleOpState.INTAKING
                    }
                } else {
                    driveSpeedMult = 0.5
                }

                if (nowGamepad1.dpad_up && !lastGamepad1.dpad_up) {
                    state = TeleOpState.SCORING
                }

                if (nowGamepad1.right_trigger > 0.5 && lastGamepad1.right_trigger <= 0.5) {
                    state = TeleOpState.CLIMB_1_START
                }

            }
            TeleOpState.INTAKING -> {

                val wristRollStep = 1.5
                if (justSwitchedState) {
                    hardware.plungerRetracted = true
                    hardware.targetPivotAngle = 0.0
                    hardware.wristRoll = 0.0
                    hardware.wristPitch = -1.4

                    mayUseHeadingPID = true
                    useHeadingPID = true
                    targetHeading = round(hardware.currentHeading, PI/2)
                }

                if (hardware.useSlidePID && hardware.getCurrentPivotAngle() < 0.5) {
                    hardware.targetSlideExtension = 0.25
                }

                driveSpeedMult = 0.6

                 if (nowGamepad1.right_trigger > 0.2) {
                    hardware.wristPitch = -1.6
                    hardware.plungerRetracted = false
                    hardware.intakeSpeed = 1.0
                } else if (nowGamepad1.left_trigger > 0.2) {
                    hardware.plungerRetracted = true
                    hardware.intakeSpeed = -1.0
                } else {
                    hardware.wristPitch = -1.6
                    hardware.plungerRetracted = true
                    hardware.intakeSpeed = 0.0
                }

                if (nowGamepad1.right_stick_x > 0.8 && lastGamepad1.right_stick_x <= 0.8) {
                    hardware.wristRoll = clamp(hardware.wristRoll - wristRollStep, -wristRollStep * 2, wristRollStep * 2)
                }
                if (nowGamepad1.right_stick_x < -0.8 && lastGamepad1.right_stick_x >= -0.8) {
                    hardware.wristRoll = clamp(hardware.wristRoll + wristRollStep, -wristRollStep * 2, wristRollStep * 2)
                }

                if (abs(nowGamepad1.right_stick_y) > 0.2 ) {
                    hardware.useSlidePID = false
                }
                if (!hardware.useSlidePID) {
                    val extension = hardware.getCurrentSlideExtension()
                    hardware.feedforwardSlidesVoltage = clamp(
                        9.0 * -nowGamepad1.right_stick_y,
                        if (extension < 0.04) 0.0 else -12.0,
                        if (extension > 0.45) -6.0
                        else if (extension > 0.4) 0.0
                        else 12.0
                    )
                }

                if (nowGamepad1.dpad_up && !lastGamepad1.dpad_up ) {
                    hardware.useSlidePID = true
                    state = TeleOpState.DRIVING
                }


            }
            TeleOpState.SCORING -> {

                if (justSwitchedState) {
                    scoreHeightIndex = 0
                    mayUseHeadingPID = false
                    useHeadingPID = false
                }

                hardware.wristRoll = 0.0
                hardware.wristPitch = 1.7
                hardware.plungerRetracted = false
                hardware.targetSlideExtension = if (hardware.getCurrentPivotAngle() > 1.2)
                    scoreHeights[scoreHeightIndex]
                    else 0.0
                hardware.targetPivotAngle = PI/2

                telemetry.addData("num score heights", scoreHeights.size)
                telemetry.update()

                if (nowGamepad1.right_bumper && !lastGamepad1.right_bumper) {
                    scoreHeightIndex = if (scoreHeightIndex < scoreHeights.size - 1) scoreHeightIndex + 1 else scoreHeightIndex
                }
                if (nowGamepad1.left_bumper && !lastGamepad1.left_bumper) {
                    scoreHeightIndex = if (scoreHeightIndex > 0) scoreHeightIndex - 1 else 0
                }

                driveSpeedMult = 0.5

                hardware.intakeSpeed = -nowGamepad1.left_trigger * 0.5
                hardware.intakeSpin = nowGamepad1.right_trigger.toDouble()

                if (nowGamepad1.dpad_down && !lastGamepad1.dpad_down ) {
                    state = TeleOpState.DRIVING
                }

            }
            TeleOpState.CLIMB_1_START -> {

                hardware.wristPitch = -1.5
                hardware.targetSlideExtension = 0.35
                hardware.targetPivotAngle = 1.2

                mayUseHeadingPID = true
                useHeadingPID = false
                driveSpeedMult = 1.0

                if (nowGamepad1.right_trigger > 0.5 && lastGamepad1.right_trigger <= 0.5) {
                    state = TeleOpState.CLIMB_1_OVEREXTEND
                }
                if (nowGamepad1.left_trigger > 0.5) {
                    state = TeleOpState.DRIVING
                }
            }
            TeleOpState.CLIMB_1_OVEREXTEND -> {
                hardware.targetSlideExtension = 0.35
                hardware.targetPivotAngle = 1.6
                driveSpeedMult = 0.0

                if (nowGamepad1.right_trigger > 0.5 && lastGamepad1.right_trigger <= 0.5) {
                    state = TeleOpState.CLIMB_1_RETRACT
                } else if (nowGamepad1.left_trigger > 0.5 && lastGamepad1.left_trigger <= 0.5) {
                    state = TeleOpState.CLIMB_1_START
                }
            }
            TeleOpState.CLIMB_1_RETRACT -> {
                hardware.targetSlideExtension = -0.1
                if (nowGamepad1.right_trigger > 0.5 && lastGamepad1.right_trigger <= 0.5) {
                    state = TeleOpState.CLIMB_2_START
                }
            }
            TeleOpState.CLIMB_2_START -> {
                hardware.targetSlideExtension = 0.6
                hardware.targetPivotAngle = 1.2
                if (nowGamepad1.right_trigger > 0.5 && lastGamepad1.right_trigger <= 0.5) {
                    state = TeleOpState.CLIMB_2_OVEREXTEND
                }
            }
            TeleOpState.CLIMB_2_OVEREXTEND -> {
                hardware.targetPivotAngle = 2.3
                if (nowGamepad1.right_trigger > 0.5 && lastGamepad1.right_trigger <= 0.5) {
                    state = TeleOpState.CLIMB_2_RETRACT
                }
            }
            TeleOpState.CLIMB_2_RETRACT -> {
                hardware.targetSlideExtension = -0.1
                if (hardware.getCurrentSlideExtension() < 0.2) {
                    hardware.targetPivotAngle = 0.5
                }
            }
        }


        // I hate the FTC coordinate system

        val ANGLE_SNAP_THRESHOLD = Math.toRadians(5.0);

        if (nowGamepad1.right_bumper && !lastGamepad1.right_bumper && mayUseHeadingPID) {
            val nearestAngleDiff = wrapAngle( round(hardware.currentHeading, PI/2) - hardware.currentHeading)
            if (nearestAngleDiff > ANGLE_SNAP_THRESHOLD  && useHeadingPID == false) {
                targetHeading = round(hardware.currentHeading, PI/2)
            } else {
                targetHeading = round(hardware.currentHeading + PI/2, PI/2)
            }
            useHeadingPID = true
        }

        if (nowGamepad1.left_bumper && !lastGamepad1.left_bumper && mayUseHeadingPID) {

            val nearestAngleDiff = wrapAngle( round(hardware.currentHeading, PI/2) - hardware.currentHeading)
            if (nearestAngleDiff < ANGLE_SNAP_THRESHOLD && !useHeadingPID) {
                targetHeading = round(hardware.currentHeading, PI/2)
            } else {
                targetHeading = round(hardware.currentHeading - PI/2, PI/2)
            }
            useHeadingPID = true
        }

        //set zero heading
        if (nowGamepad1.b && !lastGamepad1.b) {
            targetHeading -= hardware.rawHeading
            hardware.zeroHeading = hardware.rawHeading
        }

        val heading_kP = 0.5
        val turnPower = if (useHeadingPID)
            wrapAngle(targetHeading - hardware.currentHeading) * heading_kP
            else gamepad1.right_stick_x.toDouble()
        var fieldXBasisInRobotSpace = Vector2d(cos(-hardware.currentHeading), sin(-hardware.currentHeading))
        var fieldYBasisInRobotSpace = Vector2d(sin(-hardware.currentHeading), -cos(-hardware.currentHeading))

        hardware.driveCommand = if (climbing) PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
            else PoseVelocity2d(
            fieldXBasisInRobotSpace.times((-gamepad1.left_stick_y).toDouble())
                .plus( fieldYBasisInRobotSpace.times((-gamepad1.left_stick_x).toDouble()) ).times(driveSpeedMult),
            turnPower
        )

        hardware.update()

        telemetry.addData("Robot pitch", hardware.currentPitch)
        telemetry.update()

        if (removeJustSwitchedStateLater) {
            justSwitchedState = false
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    override fun stop() {
    }
}
