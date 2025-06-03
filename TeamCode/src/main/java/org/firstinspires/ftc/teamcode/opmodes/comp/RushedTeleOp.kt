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
package org.firstinspires.ftc.teamcode.opmodes.comp

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.util.AutoHelper
import org.firstinspires.ftc.teamcode.util.GamepadEx
import org.firstinspires.ftc.teamcode.util.MathUtils.clampInt
import org.firstinspires.ftc.teamcode.util.MathUtils.mapRange
import org.firstinspires.ftc.teamcode.util.MathUtils.powerCurve
import org.firstinspires.ftc.teamcode.util.MathUtils.round
import org.firstinspires.ftc.teamcode.util.MathUtils.wrapAngle
import org.firstinspires.ftc.teamcode.util.RobotHardware
import java.lang.Math.pow
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.pow
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

fun Boolean.toInt() = if (this) 1 else 0

enum class TeleOpState {
    INTAKING,
    DRIVING,
    SCORING,
    CLIMB_1_START,
    CLIMB_1_OVEREXTEND,
    CLIMB_1_RETRACT,
    CLIMB_2_START,
    CLIMB_2_OVEREXTEND,
    CLIMB_2_RETRACT,
    CLIMB_1_RETRACT_2,
    CLIMB_2_START_AGAIN,
    CLIMB_2_RETRACT_1,
    CLIMB_2_FINISH,
    HOMING
}

@TeleOp(name = "Rushed TeleOp", group = "Iterative OpMode")
class RushedTeleOp : OpMode() {
    // Declare OpMode members.

    private val dash: FtcDashboard = FtcDashboard.getInstance()

    private var lastUpdateTime = 0.0
    private val runtime = ElapsedTime()
    private var stateSwitchTime = 0.0
    private lateinit var hardware : RobotHardware
    private lateinit var autoHelper : AutoHelper
    private var state : TeleOpState = TeleOpState.DRIVING
        set (value) {
            justSwitchedState = true
            stateSwitchTime = runtime.seconds()
            field = value
        }

    private lateinit var driver1: GamepadEx
    private lateinit var driver2: GamepadEx


    private var driveSpeedMult = 1.0
    private var mayUseHeadingPID = true
    private var useHeadingPID = false
    private var targetHeading = 0.0

    private var justSwitchedState = true

    private var climbing = false
    private var specIntake = false

    private var scoreHeightIndex = 0
    private val scoreHeights = arrayOf(0.5, 1.0)

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    override fun init() {
        hardware = RobotHardware(hardwareMap, telemetry)
        hardware.init();
        driver1 = GamepadEx(gamepad1)
        driver2 = GamepadEx(gamepad2)
        autoHelper = AutoHelper(hardware)
    }

    override fun init_loop() {
        hardware.startingPose()
        hardware.update()
    }


    override fun start() {
        runtime.reset()
    }

    /*

     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */


    val actionQueue = ArrayList<Action>()
    fun runActionAsync(action: Action) {
        //TODO: add cancelable actions
        if (actionQueue.isEmpty()) {
            actionQueue.add(action)
        }
    }

    fun handleRunningActions(packet: TelemetryPacket) {
        if (actionQueue.isEmpty()) return
        val currentAction = actionQueue[0]
        if (!currentAction.run(packet)) {
            actionQueue.removeAt(0)
        }
    }

    override fun loop() {

        val packet = TelemetryPacket()

        var deltaTime = runtime.seconds() - lastUpdateTime
        lastUpdateTime = runtime.seconds()

        driver1.update()
        driver2.update()
        val removeJustSwitchedStateLater = justSwitchedState

        when (state) {
            TeleOpState.DRIVING -> {

                hardware.wristRoll = 0.0
                hardware.wristPitch = 1.7
                hardware.intakeSpeed = 0.0
                hardware.targetSlideExtension = 0.05
                hardware.ptoActive = false

                if (hardware.getCurrentSlideExtension() < 0.5) {
                    hardware.targetPivotAngle = 1.2
                }

                mayUseHeadingPID = true
                climbing = false

                if ( driver1.right_stick_x isOver 0.2 ) {
                    useHeadingPID = false
                }

                if (hardware.getCurrentSlideExtension() < 0.15) {
                    driveSpeedMult = 1.0
                    if (driver1.dpad_down.wasPressed || driver2.dpad_down.wasPressed) {
                        state = TeleOpState.INTAKING
                    }
                } else {
                    driveSpeedMult = 0.5
                }

                if (driver1.dpad_up.wasPressed || driver2.dpad_up.wasPressed || driver1.left_stick_button.value) {
                    state = TeleOpState.SCORING
                }
                if (driver1.right_trigger.wasPressed(0.5)) {
                    state = TeleOpState.CLIMB_1_START
                }

                if (driver1.guide.wasPressed) {
                    state = TeleOpState.HOMING
                }

            }
            TeleOpState.INTAKING -> {

                val wristRollStep = Math.PI/4
                if (justSwitchedState) {
                    specIntake = false
                    hardware.wristRoll = 0.0
                    hardware.targetPivotAngle = 0.0
                    hardware.targetSlideExtension = 0.1
                    mayUseHeadingPID = true
                    useHeadingPID = false
                    targetHeading = round(hardware.currentHeading, PI/2)
                }

                if ( driver1.right_stick_x isOver 0.5 ) {
                    useHeadingPID = false
                }
                driveSpeedMult = if (specIntake) 0.3 else 0.45

                if (driver1.right_stick_y isOver 0.2) {
                    hardware.targetSlideExtension -= Math.pow(driver1.right_stick_y.value, 3.0) * deltaTime
                    hardware.targetSlideExtension = clamp(hardware.targetSlideExtension, 0.0, 0.5)
                }

                if (driver1.y.wasPressed) {
                    specIntake = !specIntake
                }

                if (specIntake) {
                    hardware.targetSlideExtension = 0.0
                    hardware.targetPivotAngle = 0.28 + (if (driver1.a.value) 0.05 else if (driver1.x.value) -0.05 else 0.0 )
                    hardware.wristPitch = -0.38
                    if (driver1.right_trigger isOver 0.2) {
                        hardware.intakeSpeed = 1.0
                    } else if (driver1.left_trigger isOver 0.2) {
                        hardware.intakeSpeed = -1.0
                    } else {
                        hardware.intakeSpeed = 0.0
                    }
                } else {
                    hardware.wristPitch = -Math.PI/2
                    hardware.targetPivotAngle = 0.25 - hardware.getCurrentSlideExtension() * 0.06;
                    if (driver1.right_trigger isOver 0.2) {
                        hardware.targetPivotAngle = 0.0
                        hardware.intakeSpeed = 1.0
                    } else if (driver1.left_trigger isOver 0.2) {
                        hardware.intakeSpeed = -1.0
                    } else {
                        hardware.intakeSpeed = 0.0
                    }
                }

                if (driver1.right_stick_button.wasPressed || driver1.left_stick_button.wasPressed
                    || driver2.right_stick_button.wasPressed || driver2.left_stick_button.wasPressed ) {
                    val delta = driver1.right_stick_button.value.toInt() +
                            driver2.right_stick_button.value.toInt() -
                            driver1.left_stick_button.value.toInt() -
                            driver2.left_stick_button.value.toInt()
                    hardware.wristRoll = clamp(hardware.wristRoll - wristRollStep * delta.toDouble(), -wristRollStep * 2.0, wristRollStep * 2.0)
                }

                if (driver1.dpad_up.wasPressed || driver2.dpad_up.wasPressed) {
                    state = TeleOpState.DRIVING
                }

            }
            TeleOpState.HOMING -> {
                hardware.targetPivotAngle = -0.1
                hardware.targetSlideExtension = -0.04
                if (runtime.seconds() - stateSwitchTime > 1.1) {
                    hardware.targetPivotAngle = 0.105
                }
                if (runtime.seconds() - stateSwitchTime > 1.5) {
                    hardware.resetEncoders()
                    state = TeleOpState.DRIVING
                }
            }
            TeleOpState.SCORING -> {

                if (justSwitchedState) {
                    scoreHeightIndex = 0
                    mayUseHeadingPID = true
                    useHeadingPID = false
                }

                if ( driver1.right_stick_x isOver 0.2 ) {
                    useHeadingPID = false
                }

                driveSpeedMult = mapRange(hardware.getCurrentSlideExtension(), 0.0, 1.0, 1.0, 0.6)

                if (actionQueue.isEmpty()) {

                    // pivot
                    hardware.targetPivotAngle = 1.58 + if (driver1.a.value) 0.05 else 0.0 //+ hardware.getCurrentSlideExtension() * 0.06

                    // extension
                    if (driver1.left_stick_button.value) {
                        scoreHeightIndex = scoreHeights.size - 1
                        targetHeading = Math.PI/4
                    }
                    val deltaIndex = driver1.dpad_up.wasPressed.toInt() +
                            driver2.dpad_up.wasPressed.toInt() -
                            driver1.dpad_down.wasPressed.toInt() -
                            driver2.dpad_down.wasPressed.toInt()
                    scoreHeightIndex =
                        clampInt(scoreHeightIndex + deltaIndex, 0, scoreHeights.size - 1)
                    hardware.targetSlideExtension = if (hardware.getCurrentPivotAngle() > 1.0)
                        scoreHeights[scoreHeightIndex]
                    else 0.0

                    // wrist
                    hardware.wristRoll = -Math.PI/2 - 0.04
                    hardware.wristPitch = 1.6
                    hardware.intakeSpeed =
                        -driver1.left_trigger.value * 0.25 + (if (driver1.right_stick_button.value) 1.0 else 0.0)
                    hardware.intakeSpin = driver1.right_trigger.value

                    if (driver1.dpad_down.wasPressed || driver2.dpad_down.wasPressed) {
                        state = TeleOpState.DRIVING
                    }

                }

                if (driver1.x.wasPressed && actionQueue.isEmpty()) {
                    runActionAsync(SequentialAction(
                        autoHelper.readyToScoreSpecimenAction(),
                        Action { return@Action driver1.x.value },
                        autoHelper.scoreSpecimenAction(),
                    ))
                }

            }
            TeleOpState.CLIMB_1_START -> {

                hardware.wristPitch = -1.5
                hardware.targetSlideExtension = 0.62
                hardware.targetPivotAngle = 1.35

                mayUseHeadingPID = true
                useHeadingPID = false
                driveSpeedMult = 1.0

                if (driver1.right_trigger.wasPressed(0.5)) {
                    state = TeleOpState.CLIMB_1_OVEREXTEND
                }
                if (driver1.left_trigger.wasPressed(0.5)) {
                    state = TeleOpState.DRIVING
                }
            }
            TeleOpState.CLIMB_1_OVEREXTEND -> {
                hardware.targetPivotAngle = 1.8
                climbing = true
                driveSpeedMult = 0.0

                if (driver1.right_trigger.wasPressed(0.5)) {
                    state = TeleOpState.CLIMB_1_RETRACT
                } else if (driver1.left_trigger.wasPressed(0.5)) {
                    state = TeleOpState.CLIMB_1_START
                }
            }
            TeleOpState.CLIMB_1_RETRACT -> {
                hardware.targetSlideExtension = -1.0
                hardware.targetPivotAngle = 1.35
                hardware.ptoActive = true
                if (driver1.right_trigger.wasPressed(0.5)) {
                    state = TeleOpState.CLIMB_1_RETRACT_2
                }
                else if (driver1.left_trigger.wasPressed(0.5)) {
                    state = TeleOpState.CLIMB_1_OVEREXTEND
                }
            }
            TeleOpState.CLIMB_1_RETRACT_2 -> {
                hardware.targetPivotAngle = 1.7
                if (driver1.right_trigger.wasPressed(0.5)) {
                    state = TeleOpState.CLIMB_2_START
                }
                else if (driver1.left_trigger.wasPressed(0.5)) {
                    state = TeleOpState.CLIMB_1_OVEREXTEND
                }
            }
            TeleOpState.CLIMB_2_START -> {
                hardware.targetSlideExtension = 0.3
                hardware.wristPitch = -1.5
                if (driver1.right_trigger.wasPressed(0.5)) {
                    state = TeleOpState.CLIMB_2_START_AGAIN
                }
                hardware.ptoActive = false
            }
            TeleOpState.CLIMB_2_START_AGAIN -> {
                hardware.targetPivotAngle = 1.7
                hardware.targetSlideExtension = 1.0
                if (driver1.right_trigger.wasPressed(0.5)) {
                    state = TeleOpState.CLIMB_2_OVEREXTEND
                }
            }
            TeleOpState.CLIMB_2_OVEREXTEND -> {
                hardware.targetSlideExtension = 1.3
                hardware.wristPitch = 1.5
                hardware.targetPivotAngle = 2.4
                if (driver1.right_trigger.wasPressed(0.5)) {
                    state = TeleOpState.CLIMB_2_RETRACT
                }
            }
            TeleOpState.CLIMB_2_RETRACT -> {
                hardware.targetSlideExtension = -1.0
                if (hardware.getCurrentSlideExtension() < 0.1) {
                    hardware.targetPivotAngle = 1.9
                } else {
                    hardware.targetPivotAngle = 2.0
                }
                if (driver1.left_trigger.wasPressed(0.5)) {
                    state = TeleOpState.CLIMB_2_OVEREXTEND
                }
                if (driver1.right_trigger.wasPressed(0.5)) {
                    state = if (hardware.getCurrentSlideExtension() < 0.1) TeleOpState.CLIMB_2_FINISH else TeleOpState.CLIMB_2_RETRACT_1
                }
                hardware.ptoActive = true
            }
            TeleOpState.CLIMB_2_RETRACT_1 -> {
                hardware.targetSlideExtension = -1.0
                if (hardware.getCurrentSlideExtension() < 0.05) {
                    hardware.targetPivotAngle = 2.3
                } else {
                    hardware.targetPivotAngle = 1.5
                }
                if (driver1.right_trigger.wasPressed(0.5)) {
                    state = if (hardware.getCurrentSlideExtension() < 0.1) TeleOpState.CLIMB_2_FINISH else TeleOpState.CLIMB_2_RETRACT
                }
            }
            TeleOpState.CLIMB_2_FINISH -> {
                hardware.targetSlideExtension = 0.3
                if (hardware.getCurrentSlideExtension() > 0.2) {
                    hardware.targetPivotAngle = 0.95
                }
                hardware.ptoActive = false
            }
        }

        // I hate the FTC coordinate system
        val ANGLE_SNAP_THRESHOLD = Math.toRadians(5.0);

        if (driver1.right_bumper.wasPressed && mayUseHeadingPID) {
            val nearestAngleDiff = wrapAngle( round(hardware.currentHeading, PI/4) - hardware.currentHeading)
            if (nearestAngleDiff > ANGLE_SNAP_THRESHOLD && !useHeadingPID) {
                targetHeading = round(hardware.currentHeading, PI/4)
            } else {
                targetHeading = round(targetHeading + PI/4, PI/4)
            }
            useHeadingPID = true
        }

        if (driver1.left_bumper.wasPressed && mayUseHeadingPID) {

            val nearestAngleDiff = wrapAngle( round(hardware.currentHeading, PI/4) - hardware.currentHeading)
            if (nearestAngleDiff < ANGLE_SNAP_THRESHOLD && !useHeadingPID) {
                targetHeading = round(hardware.currentHeading, PI/4)
            } else {
                targetHeading = round(targetHeading - PI/4, PI/4)
            }
            useHeadingPID = true
        }

        //set zero heading
        if (driver1.b.wasPressed) {
            hardware.imu.resetYaw()
            //targetHeading -= hardware.rawHeading
            //hardware.zeroHeading = hardware.rawHeading
        }

        val heading_kP = 0.5
        val turnPower = if (useHeadingPID)
            wrapAngle(targetHeading - hardware.currentHeading) * heading_kP
            else powerCurve(gamepad1.right_stick_x.toDouble(), 1.5)
        var fieldXBasisInRobotSpace = Vector2d(cos(-hardware.currentHeading), sin(-hardware.currentHeading))
        var fieldYBasisInRobotSpace = Vector2d(sin(-hardware.currentHeading), -cos(-hardware.currentHeading))

        var fwdCommand = powerCurve((-gamepad1.left_stick_y).toDouble(), 1.5)
        var sideCommand = powerCurve((-gamepad1.left_stick_x).toDouble(), 1.5)

        hardware.driveCommand = if (climbing) PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
            else PoseVelocity2d(
            fieldXBasisInRobotSpace.times( fwdCommand )
                .plus( fieldYBasisInRobotSpace.times(sideCommand) ).times(driveSpeedMult),
            turnPower * driveSpeedMult.pow(1.0 / 3.0) * (if (state == TeleOpState.INTAKING) 0.45 else 1.0 )
        )

        handleRunningActions(packet)
        hardware.update()

        telemetry.addData("Robot pitch", hardware.currentPitch)
        telemetry.update()
        dash.sendTelemetryPacket(packet)

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
