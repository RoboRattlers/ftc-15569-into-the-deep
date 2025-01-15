package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.clamp
import kotlin.math.PI

object MathUtils {

    fun lerp(a: Double, b: Double, alpha: Double): Double {
        return a + (b - a) * alpha
    }

    fun round(value: Double, multipleOf: Double = 1.0): Double {
        return kotlin.math.round(value/multipleOf) * multipleOf
    }

    fun wrapAngle(angle: Double): Double {
        var offsetAngle = (angle.rem( 2 * PI)  + 2 * PI).rem(2 * PI) + PI
        return offsetAngle.rem(2 * PI) - PI
    }

    fun mapRange(value: Double, oldMin: Double, oldMax: Double, newMin: Double, newMax: Double, clampValue: Boolean = false): Double {
        var newValue = (value - oldMin) / (oldMax - oldMin) * (newMax - newMin) + newMin
        return if (clampValue) clamp(newValue, newMin, newMax) else newValue
    }

    fun clampInt(value: Int, min: Int, max: Int): Int {
        return if (value < min) min else if (value > max) max else value
    }


    fun gridToFieldCoords(x: Double, y: Double): Vector2d {
        return Vector2d((x + 0.5) * 24, (y + 0.5) * 24 )
    }

    fun gridToFieldCoords(cell: Vector2d): Vector2d {
        return gridToFieldCoords(cell.x, cell.y)
    }

    fun gridToFieldCoords(x: Double, y: Double, heading: Double): Pose2d {
        return Pose2d(gridToFieldCoords(x, y), heading)
    }

    fun gridToFieldCoords(cell: Pose2d): Pose2d {
        return Pose2d(gridToFieldCoords(cell.position.x, cell.position.y), cell.heading)
    }

}