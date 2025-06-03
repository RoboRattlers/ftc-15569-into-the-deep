package org.firstinspires.ftc.teamcode.util

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import com.acmerobotics.roadrunner.Pose2d
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


enum class SampleColor {
    BLUE,
    RED,
    YELLOW
}

data class SampleDetection(val color: SampleColor, val pose: Pose2d, val boundingBox: RotatedRect)

data class VisionDebugPreview(val bitmap: Bitmap, val detections: List<SampleDetection>)

class SampleDetectionProcessor : VisionProcessor {

    var yellowHueMin = 20.0
    var yellowHueMax = 100.0
    var blueHueMin = 100.0
    var blueHueMax = 160.0
    var saturationMin = 170.0
    var valueMin = 140.0

    var blueHSVMin: Scalar = Scalar(blueHueMin, 120.0, 100.0, 0.0)
    var blueHSVMax: Scalar = Scalar(blueHueMax, 255.0, 255.0, 0.0)
    var yellowHSVMin: Scalar = Scalar(yellowHueMin, 170.0, 150.0, 0.0)
    var yellowHSVMax: Scalar = Scalar(yellowHueMax, 255.0, 255.0, 0.0)
    // red has to be flipped since it occupies both the start and end of the hsv spectrum
    var redHSVMin0: Scalar = Scalar(0.0, saturationMin, valueMin, 0.0)
    var redHSVMax0: Scalar = Scalar(10.0, 255.0, 255.0, 0.0)
    var redHSVMin1: Scalar = Scalar(170.0, saturationMin, valueMin, 0.0)
    var redHSVMax1: Scalar = Scalar(180.0, 255.0, 255.0, 0.0)

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

    var minArea: Int = 40
    var maxArea: Int = 10000
    private val contoursByArea = ArrayList<MatOfPoint>()

    private val contoursByArea2f = MatOfPoint2f()
    private val modifiableSampleDetections = ArrayList<SampleDetection>()
    var detections: List<SampleDetection>? = null

    var modifying = false

    var lineColor: Scalar = Scalar(0.0, 255.0, 0.0, 0.0)
    var lineThickness: Int = 3

    private val inputRotRects = Mat()

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
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
            if ((area >= minArea) && (area <= maxArea)) {
                contoursByArea.add(contour)
            }
        }

        for (points in contoursByArea) {
            contoursByArea2f.release()
            points.convertTo(contoursByArea2f, CvType.CV_32F)

            val boundingRect = Imgproc.minAreaRect(contoursByArea2f)
            val IN_PER_PIXEL = HardwareConstants.CAMERA_IN_PER_PIXEL
            val CENTER_X_OFFSET = HardwareConstants.CAMERA_CENTER_X_OFFSET
            var angle = if (boundingRect.size.width < boundingRect.size.height) boundingRect.angle + 90.0 else boundingRect.angle
            if (angle > 90.0) {
                angle = angle - 180.0
            }
            val transformedPose = Pose2d(
                -(boundingRect.center.x - 160.0) * IN_PER_PIXEL + CENTER_X_OFFSET,
                (boundingRect.center.y - 120.0) * IN_PER_PIXEL,
                -Math.toRadians(angle)
            )
            modifiableSampleDetections.add(SampleDetection(color, transformedPose, boundingRect))
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

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {
        var rectPaint = Paint()
        rectPaint.setStyle(Paint.Style.STROKE)
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4)

        val detections: List<SampleDetection> = userContext as List<SampleDetection>
        for (detection in detections) {
            rectPaint.setColor(
                if (detection.color == SampleColor.RED) Color.MAGENTA
                        else if (detection.color == SampleColor.BLUE) Color.CYAN
                        else Color.GREEN)
            val rectPoints = arrayOfNulls<Point>(4)
            detection.boundingBox.points(rectPoints)
            for (i in 0..3) {
                val point = rectPoints[i]
                val nextPoint = rectPoints[(i + 1) % 4]
                if (canvas != null && point != null && nextPoint != null) {
                    canvas.drawLine(
                        (point.x * scaleBmpPxToCanvasPx).toFloat(),
                        (point.y * scaleBmpPxToCanvasPx).toFloat(),
                        (nextPoint.x * scaleBmpPxToCanvasPx).toFloat(),
                        (nextPoint.y * scaleBmpPxToCanvasPx).toFloat(), rectPaint
                    )
                }
            }
        }
    }
}