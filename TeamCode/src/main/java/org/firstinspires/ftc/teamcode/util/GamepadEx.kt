package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.Gamepad
import kotlin.reflect.KProperty0

class Button internal constructor(private var currentProperty: KProperty0<Boolean>,
             private var lastProperty: KProperty0<Boolean>) {

    var value: Boolean = currentProperty.get()
        get() = currentProperty.get()
        private set
    var wasPressed: Boolean = false
        get() = currentProperty.get() && !lastProperty.get()
        private set
    var wasReleased: Boolean = false
        get() = !currentProperty.get() && lastProperty.get()
        private set

}

class Axis internal constructor(private var currentProperty: KProperty0<Float>,
                                private var lastProperty: KProperty0<Float>) {

    var value: Double = currentProperty.get().toDouble()
        get() = currentProperty.get().toDouble()
        private set
    fun wasPressed(threshold: Double = 0.3): Boolean {
        require(threshold > 0.0)
        return Math.abs(currentProperty.get()) >= threshold && Math.abs(lastProperty.get()) < threshold
    }
    fun wasReleased(threshold: Double = 0.3): Boolean {
        require(threshold > 0.0)
        return Math.abs(currentProperty.get()) < threshold && Math.abs(lastProperty.get()) >= threshold
    }
    infix fun isOver(threshold: Double): Boolean {
        require(threshold > 0.0)
        return Math.abs(currentProperty.get()) > threshold
    }

}

class GamepadEx(private var realGamepad: Gamepad) {

    var currentGamepad = Gamepad()
    var lastGamepad = Gamepad()

    var a = Button(currentGamepad::a, lastGamepad::a)
    var b = Button(currentGamepad::b, lastGamepad::b)
    var x = Button(currentGamepad::x, lastGamepad::x)
    var y = Button(currentGamepad::y, lastGamepad::y)
    var dpad_up = Button(currentGamepad::dpad_up, lastGamepad::dpad_up)
    var dpad_down = Button(currentGamepad::dpad_down, lastGamepad::dpad_down)
    var dpad_left = Button(currentGamepad::dpad_left, lastGamepad::dpad_left)
    var dpad_right = Button(currentGamepad::dpad_right, lastGamepad::dpad_right)
    var back = Button(currentGamepad::back, lastGamepad::back)
    var start = Button(currentGamepad::start, lastGamepad::start)
    var guide = Button(currentGamepad::guide, lastGamepad::guide)
    var left_bumper = Button(currentGamepad::left_bumper, lastGamepad::left_bumper)
    var right_bumper = Button(currentGamepad::right_bumper, lastGamepad::right_bumper)
    var left_stick_button = Button(currentGamepad::left_stick_button, lastGamepad::left_stick_button)
    var right_stick_button = Button(currentGamepad::right_stick_button, lastGamepad::right_stick_button)

    var left_stick_x = Axis(currentGamepad::left_stick_x, lastGamepad::left_stick_x)
    var left_stick_y = Axis(currentGamepad::left_stick_y, lastGamepad::left_stick_y)
    var right_stick_x = Axis(currentGamepad::right_stick_x, lastGamepad::right_stick_x)
    var right_stick_y = Axis(currentGamepad::right_stick_y, lastGamepad::right_stick_y)
    var left_trigger = Axis(currentGamepad::left_trigger, lastGamepad::left_trigger)
    var right_trigger = Axis(currentGamepad::right_trigger, lastGamepad::right_trigger)

    fun update() {
        lastGamepad.copy(currentGamepad)
        currentGamepad.copy(realGamepad)
    }

}