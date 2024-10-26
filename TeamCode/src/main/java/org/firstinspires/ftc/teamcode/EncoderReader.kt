package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp
class EncoderReader : LinearOpMode() {
    override fun runOpMode() {
        val motors = hardwareMap.getAll(DcMotor::class.java)

        waitForStart()

        while (opModeIsActive()) {
            for (motor in motors) {
                telemetry.addData(motor.deviceName, motor.currentPosition)
            }
            telemetry.update()
        }
    }
}