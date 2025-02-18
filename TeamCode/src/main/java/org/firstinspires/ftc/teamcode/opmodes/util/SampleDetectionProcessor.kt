package org.firstinspires.ftc.teamcode.opmodes.util

import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
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

class SampleDetectionProcessor : VisionProcessor {

    var yellowHueMin = 50.0
    var yellowHueMax = 100.0
    var blueHueMin = 100.0
    var blueHueMax = 160.0
    var saturationMin = 125.0
    var valueMin = 50.0

    var blueHSVMin: Scalar = Scalar(blueHueMin, saturationMin, valueMin, 0.0)
    var blueHSVMax: Scalar = Scalar(blueHueMax, 255.0, 255.0, 0.0)
    var yellowHSVMin: Scalar = Scalar(yellowHueMin, saturationMin, valueMin, 0.0)
    var yellowHSVMax: Scalar = Scalar(yellowHueMax, 255.0, 255.0, 0.0)
    // red has to be flipped since it occupies both the start and end of the hsv spectrum
    var redInverseHSVMin: Scalar = Scalar(yellowHueMin, 0.0, 0.0, 0.0)
    var redInverseHSVMax: Scalar = Scalar(blueHueMax, saturationMin, valueMin, 0.0)

    private val hsvMat = Mat()
    private val blueBinaryMat = Mat()
    private val yellowBinaryMat = Mat()
    private val redIntermediateBinaryMat = Mat()
    private val redBinaryMat = Mat()

    private val contours = ArrayList<MatOfPoint>()
    private val hierarchy = Mat()

    var minArea: Int = 1
    var maxArea: Int = 10000
    private val contoursByArea = ArrayList<MatOfPoint>()

    private val contoursByArea2f = MatOfPoint2f()
    private val contourBoundingBoxes = ArrayList<RotatedRect>()

    var modifying = false

    var lineColor: Scalar = Scalar(0.0, 255.0, 0.0, 0.0)
    var lineThickness: Int = 3

    private val inputRotRects = Mat()

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
    }

    override fun processFrame(input: Mat, captureTimeNanos: Long): Any {

        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV)
        Core.inRange(hsvMat, blueHSVMin, blueHSVMax, blueBinaryMat)
        Core.inRange(hsvMat, yellowHSVMin, yellowHSVMax, yellowBinaryMat)
        Core.inRange(hsvMat, redInverseHSVMin, redInverseHSVMax, redIntermediateBinaryMat)
        // red has to be inverted
        Core.bitwise_not(redIntermediateBinaryMat, redBinaryMat)

        contours.clear()
        hierarchy.release()
        Imgproc.findContours(
            redBinaryMat,
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

        contourBoundingBoxes.clear()
        for (points in contoursByArea) {
            contoursByArea2f.release()
            points.convertTo(contoursByArea2f, CvType.CV_32F)

            contourBoundingBoxes.add(Imgproc.minAreaRect(contoursByArea2f))
        }

        return contourBoundingBoxes.toList()

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
        rectPaint.setColor(Color.RED)
        rectPaint.setStyle(Paint.Style.STROKE)
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4)

        val boxes: List<RotatedRect> = userContext as List<RotatedRect>
        for (rect in boxes) {
            val rectPoints = arrayOfNulls<Point>(4)
            rect.points(rectPoints)
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