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

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime


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

@Config
object PIDTunerPositions {
    @JvmField var slideExtension = 0.0
    @JvmField var pivotAngle = 0.0
    @JvmField var wristPitch = 0.0
    @JvmField var wristRoll = 0.0
    @JvmField var intakeSpeed = 0.0
    @JvmField var plungerRetracted = false
}

@TeleOp(name = "PID Tuner", group = "Iterative OpMode")
class PositionTuner : OpMode() {
    // Declare OpMode members.
    private val runtime = ElapsedTime()
    private lateinit var hardware : RobotHardware

    // usually, these would be declared in an "IntoTheDeepHardware" class
    // that would allow us to easily import hardware into OpModes that need it, helping us not repeat ourselves
    // but because this code is intentionally bad, I have not done that
    // 15569 traditionally orders drive motors starting at the front right and going in a clockwise fashion

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    override fun init() {
        hardware = RobotHardware(hardwareMap, telemetry)
        hardware.init();
    }

    override fun init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    override fun start() {
        runtime.reset()

        // TODO: bring all non-continuous actuators to their zero positions

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    override fun loop() {

        hardware.targetPivotAngle = PIDTunerPositions.pivotAngle
        hardware.targetSlideExtension = PIDTunerPositions.slideExtension
        hardware.wristPitch = PIDTunerPositions.wristPitch
        hardware.wristRoll = PIDTunerPositions.wristRoll
        hardware.plungerRetracted = PIDTunerPositions.plungerRetracted
        hardware.intakeSpeed = PIDTunerPositions.intakeSpeed
        hardware.update()
        telemetry.addData("Pivot Voltage", hardware.pivotVoltage)
        telemetry.addData("Slides Voltage", hardware.pidSlidesVoltage)
        telemetry.update()

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    override fun stop() {
    }
}
