/*
 * Copyright (c) 2024 Phil Malone
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package org.firstinspires.ftc.teamcode.opmodes.util

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.util.RobotHardware
import org.firstinspires.ftc.teamcode.util.SampleColor
import org.firstinspires.ftc.teamcode.util.SubVisionHelper
import java.util.Locale

/*
 * This OpMode illustrates how to use a video source (camera) as a color sensor
 *
 * A "color sensor" will typically determine the color of the object that it is pointed at.
 *
 * This sample performs the same function, except it uses a video camera to inspect an object or scene.
 * The user may choose to inspect all, or just a Region of Interest (ROI), of the active camera view.
 * The user must also provide a list of "acceptable colors" (Swatches) from which the closest matching color will be selected.
 *
 * To perform this function, a VisionPortal runs a PredominantColorProcessor process.
 *   The PredominantColorProcessor process is created first, and then the VisionPortal is built to use this process.
 *   The PredominantColorProcessor analyses the ROI and splits the colored pixels into several color-clusters.
 *   The largest of these clusters is then considered to be the "Predominant Color"
 *   The process then matches the Predominant Color with the closest Swatch and returns that match.
 *
 * To aid the user, a colored rectangle is drawn on the camera preview to show the RegionOfInterest,
 * The Predominant Color is used to paint the rectangle border, so the user can verify that the color is reasonable.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp
class SampleDetectionViewer : OpMode() {

    private lateinit var hardware: RobotHardware
    private lateinit var subVisionHelper: SubVisionHelper
    private lateinit var visionPoseAction: Action
    private var isActionRunning = true

    override fun init() {

        hardware = RobotHardware(hardwareMap, telemetry)
        hardware.init();
        subVisionHelper = SubVisionHelper(hardware)
        subVisionHelper.visionEnable()
        subVisionHelper.processor.preferableColors = listOf(SampleColor.BLUE, SampleColor.YELLOW)
        visionPoseAction = subVisionHelper.visionPoseAction()

    }

    fun poseToString(pose: Pose2d): String {
        return String.format(Locale.US, "%.2f, %.2f, %.2f", pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()))
    }

    override fun init_loop() {
        val packet = TelemetryPacket()
        if (isActionRunning) { isActionRunning = visionPoseAction.run(TelemetryPacket()) }
        hardware.update()
    }

    override fun loop() {
    }
}
