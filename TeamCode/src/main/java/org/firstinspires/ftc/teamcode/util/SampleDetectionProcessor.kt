package org.firstinspires.ftc.teamcode.util

import android.annotation.SuppressLint
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.core.RotatedRect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import kotlin.math.pow


enum class SampleColor {
    BLUE,
    RED,
    YELLOW,
    NONE
}

data class SampleDetection(val color: SampleColor, val pose: Pose2d, val boundingBox: RotatedRect)

data class VisionDebugPreview(val bitmap: Bitmap, val detections: List<SampleDetection>)

class SampleDetectionProcessor : VisionProcessor {

    var yellowHueMin = 5.0
    var yellowHueMax = 75.0
    var blueHueMin = 75.0
    var blueHueMax = 160.0
    var saturationMin = 110.0
    var valueMin = 120.0

    var blueHSVMin: Scalar = Scalar(blueHueMin, 150.0, 130.0, 0.0)
    var blueHSVMax: Scalar = Scalar(blueHueMax, 255.0, 255.0, 0.0)
    var yellowHSVMin: Scalar = Scalar(yellowHueMin, 140.0, 140.0, 0.0)
    var yellowHSVMax: Scalar = Scalar(yellowHueMax, 255.0, 255.0, 0.0)
    // red has to be flipped since it occupies both the start and end of the hsv spectrum
    var redHSVMin0: Scalar = Scalar(160.0, saturationMin, valueMin, 0.0)
    var redHSVMax0: Scalar = Scalar(180.0, 255.0, 255.0, 0.0)
    var redHSVMin1: Scalar = Scalar(0.0, saturationMin, valueMin, 0.0)
    var redHSVMax1: Scalar = Scalar(5.0, 255.0, 255.0, 0.0)

    private val blankMat = Mat()
    private val hsvMat = Mat()
    private val blueBinaryMat = Mat()
    private val yellowBinaryMat = Mat()
    private val redIntermediateBinaryMat1 = Mat()
    private val redIntermediateBinaryMat2 = Mat()
    private val redBinaryMat = Mat()

    private val erodedInputMat = Mat()

    private val contours = ArrayList<MatOfPoint>()
    private val hierarchy = Mat()

    var minAreaContour: Int = 100
    var minAreaRect: Int = 800
    var maxAreaRect: Int = 4700
    var minAspectRatio = 1.5
    private val contoursByArea = ArrayList<MatOfPoint>()

    private val contoursByArea2f = MatOfPoint2f()
    private val modifiableSampleDetections = ArrayList<SampleDetection>()
    var detections: List<SampleDetection>? = null

    var modifying = false

    var lineColor: Scalar = Scalar(0.0, 255.0, 0.0, 0.0)
    var lineThickness: Int = 3

    var preferableColors = listOf(SampleColor.YELLOW)

    private val inputRotRects = Mat()

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
    }

    private fun getRealPosFromCameraFractionalPos(cameraPos: Vector2d): Vector2d {
        val topX = 26.57857 * 2.0.pow(-1.77269 * cameraPos.x) - 14.37857
        val bottomX = 28.09 * 2.0.pow(-1.36705 * cameraPos.x) - 17.49
        val x = topX + (bottomX - topX) * cameraPos.y

        val yAtTop = MathUtils.mapRange(topX, -6.6, 12.2, -4.3, -8.0)
        val yAtBottom = MathUtils.mapRange(bottomX, -6.6, 10.6, 4.3, 7.7)
        val y = yAtTop + (yAtBottom - yAtTop) * cameraPos.y
        return Vector2d(x + HardwareConstants.CAMERA_CENTER_X_OFFSET, y)
    }

    private fun getScalingFactor(cameraFractionalX: Double): Double {
        return 1.04167 * 2.0.pow( 1.23334 * cameraFractionalX) - 0.597222
    }

    fun detectFromBinaryMat(binaryMat: Mat, color: SampleColor) {
        contours.clear()
        hierarchy.release()
        Imgproc.findContours(
            binaryMat,
            contours,
            hierarchy,
            Imgproc.RETR_EXTERNAL,
            Imgproc.CHAIN_APPROX_SIMPLE
        )

        contoursByArea.clear()
        for (contour in contours) {
            val area = Imgproc.contourArea(contour)
            if ((area >= minAreaContour) && (area <= 10000)) {
                contoursByArea.add(contour)
            }
        }

        for (points in contoursByArea) {
            contoursByArea2f.release()
            points.convertTo(contoursByArea2f, CvType.CV_32F)

            val boundingRect = Imgproc.minAreaRect(contoursByArea2f)
            /*if (allowableColors.contains(color)) {
                val aspectRatio = max(boundingRect.size.width, boundingRect.size.height) /
                        min(boundingRect.size.width, boundingRect.size.height)
                if (aspectRatio < minAspectRatio || boundingRect.size.area() < minAreaRect || boundingRect.size.area() > maxAreaRect) {
                    continue
                }
            }*/

            val IN_PER_PIXEL = HardwareConstants.CAMERA_IN_PER_PIXEL
            val CENTER_X_OFFSET = HardwareConstants.CAMERA_CENTER_X_OFFSET
            var angle = if (boundingRect.size.width < boundingRect.size.height) boundingRect.angle + 90.0 else boundingRect.angle
            if (angle > 90.0) {
                angle = angle - 180.0
            }
            val transformedPose = Pose2d(
                getRealPosFromCameraFractionalPos(Vector2d(boundingRect.center.x/320.0, boundingRect.center.y/240.0)),
                -Math.toRadians(angle)
            )
            if (0.375 + transformedPose.position.x * HardwareConstants.SLIDES_EXTENSION_PER_IN < 0.48) {
                modifiableSampleDetections.add(
                    SampleDetection(
                        color,
                        transformedPose,
                        boundingRect
                    )
                )
            }
        }
    }

    override fun processFrame(input: Mat, captureTimeNanos: Long): Any {

        Imgproc.erode(input, erodedInputMat, blankMat, Point(-1.0, -1.0), 3)
        Imgproc.cvtColor(erodedInputMat, hsvMat, Imgproc.COLOR_RGB2HSV)

        Core.inRange(hsvMat, blueHSVMin, blueHSVMax, blueBinaryMat)
        Core.inRange(hsvMat, yellowHSVMin, yellowHSVMax, yellowBinaryMat)
        Core.inRange(hsvMat, redHSVMin0, redHSVMax0, redIntermediateBinaryMat1)
        Core.inRange(hsvMat, redHSVMin1, redHSVMax1, redIntermediateBinaryMat2)
        Core.add(redIntermediateBinaryMat1, redIntermediateBinaryMat2, redBinaryMat)


        modifiableSampleDetections.clear()
        detectFromBinaryMat(blueBinaryMat, SampleColor.BLUE)
        detectFromBinaryMat(yellowBinaryMat, SampleColor.YELLOW)
        detectFromBinaryMat(redBinaryMat, SampleColor.RED)
        detections = modifiableSampleDetections.toList()
        return modifiableSampleDetections.toList()

    }

    @SuppressLint("DefaultLocale")
    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {
        if (canvas == null) { return }
        var rectPaint = Paint()
        rectPaint.setStyle(Paint.Style.STROKE)
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4)

        var textPaint = Paint()
        textPaint.setStyle(Paint.Style.FILL)
        textPaint.setColor(Color.WHITE)

        if (userContext == null) { return }

        val detections: List<SampleDetection> = userContext as List<SampleDetection>
        val evilPoses = SubVisionSingleton.getEvilPoses(detections, listOf(SampleColor.BLUE, SampleColor.YELLOW))
        for (detection in detections) {
            val evilness = SubVisionSingleton.getDetectionEvilness(detection, evilPoses)
            rectPaint.setColor(
                if (detection.color == SampleColor.RED) Color.MAGENTA
                        else if (detection.color == SampleColor.BLUE) Color.CYAN
                        else Color.GREEN)
            val rectPoints = arrayOfNulls<Point>(4)
            detection.boundingBox.points(rectPoints)
            for (i in 0..3) {
                val point = rectPoints[i]
                val nextPoint = rectPoints[(i + 1) % 4]
                if (point != null && nextPoint != null) {
                    canvas.drawLine(
                        (point.x * scaleBmpPxToCanvasPx).toFloat(),
                        (point.y * scaleBmpPxToCanvasPx).toFloat(),
                        (nextPoint.x * scaleBmpPxToCanvasPx).toFloat(),
                        (nextPoint.y * scaleBmpPxToCanvasPx).toFloat(), rectPaint
                    )
                }
            }
            canvas.drawText(
                String.format("(%.2f, %.2f) %.2f evilness %.0f px",
                    detection.pose.position.x,
                    detection.pose.position.y,
                    evilness,
                    detection.boundingBox.size.area()
                ),
                (detection.boundingBox.center.x * scaleBmpPxToCanvasPx).toFloat(),
                (detection.boundingBox.center.y * scaleBmpPxToCanvasPx).toFloat(),
                textPaint
            )
        }
    }
}