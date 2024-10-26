package org.firstinspires.ftc.teamcode

import androidx.core.util.Supplier
import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.sign

class PISFController(private var value: () -> Double, var kP: Double, var kI: Double = 0.0, var kS: Double = 0.0, var kF: Supplier<Double>?) {

    private val runtime = ElapsedTime()

    var setPoint = 0.0;
    var voltage = 0.0
        private set
    private var lastUpdateSetpoint = 0.0;
    private var lastUpdateTime = 0.0;
    private var accumulatedIntegralGain = 0.0;

    fun update() {
        val currentTime = runtime.seconds()
        val deltaTime = currentTime - lastUpdateTime
        lastUpdateTime = currentTime

        val error = setPoint - value()
        accumulatedIntegralGain += error * deltaTime * kI
        accumulatedIntegralGain = clamp(accumulatedIntegralGain, -3.0, 3.0)
        voltage = kP * error + accumulatedIntegralGain + kS * sign(error) + (kF?.get() ?: 0.0)
    }


}