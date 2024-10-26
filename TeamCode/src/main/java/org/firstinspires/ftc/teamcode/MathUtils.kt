package org.firstinspires.ftc.teamcode

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

}