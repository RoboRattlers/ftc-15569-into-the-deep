package org.firstinspires.ftc.teamcode

import androidx.core.util.Supplier
import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.MathUtils.lerp
import kotlin.math.exp
import kotlin.math.sign

data class ValTimePair(val value: Double, val time: Double)

class PIDSFController(private var value: () -> Double,
                      var kP: Double,
                      var kI: Double = 0.0,
                      var kD: Double = 0.0,
                      var kS: Double = 0.0,
                      var kF: Supplier<Double>?) {

    private val runtime = ElapsedTime()

    var setPoint = 0.0;
    var voltage = 0.0
        private set
    private var lastUpdateSetpoint = 0.0;
    private var lastUpdateTime = 0.0;
    private var accumulatedIntegralGain = 0.0;
    private var smoothDerivative = 0.0;
    private var lastError = 0.0;
    private var error = 0.0;

    fun update() {
        val currentTime = runtime.seconds()
        val deltaTime = currentTime - lastUpdateTime
        lastUpdateTime = currentTime

        lastError = error
        error = setPoint - value()

        // apparently in the biz they call this a "first order lag filter."
        // I'm a game dev so to me this is "lerp smoothing"
        var noisyDerivative = (error - lastError) / deltaTime
        smoothDerivative = lerp(noisyDerivative, smoothDerivative, exp(-35 * deltaTime))

        accumulatedIntegralGain += error * deltaTime * kI
        accumulatedIntegralGain = clamp(accumulatedIntegralGain, -3.0, 3.0)
        voltage = kP * error + kD * smoothDerivative + accumulatedIntegralGain + kS * sign(error) + (kF?.get() ?: 0.0)
    }

}