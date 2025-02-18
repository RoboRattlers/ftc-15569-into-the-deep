package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Arclength
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PosePath
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.RaceAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.VelConstraint

fun Pose2d.plus(x: Double, y: Double, heading: Double): Pose2d {
    return Pose2d(
        this.position.plus(Vector2d(x, y)),
        this.heading.plus(heading)
    )
}

class AutoHelper(val hardware: RobotHardware) {

    fun rampVelConstraint(startVel: Double, endVel: Double, startDispFraction: Double, endDispFraction: Double): VelConstraint {
        return VelConstraint {

            pose: Pose2dDual<Arclength>,
            path: PosePath,
            disp: Double ->

            val fractionAlongPath = disp/path.length()
            MathUtils.mapRange(fractionAlongPath, startDispFraction, endDispFraction, startVel, endVel, true)

        }
    }

    fun readyToScoreInBasketAction(): Action {
        return SequentialAction(
            ParallelAction(
                hardware.plungerAction(true, 0.1),
                hardware.wristPitchAction(0.6, 0.1),
                hardware.wristRollAction(-0.4, 0.1),
            ),
            hardware.slideToPosAction(0.1),
            hardware.pivotToAngleAction(1.35),
            hardware.slideToPosAction(0.755),
            hardware.wristRollAction(-0.4, 0.0),
        )
    }

    fun readyToGrabGamePieceAction(extension: Double, roll: Double): Action {
        return SequentialAction(
            hardware.plungerAction(true, 0.0),
            hardware.wristPitchAction(-2.0, 0.0),
            hardware.wristRollAction(roll, 0.0),
            hardware.slideToPosAction(0.02),
            hardware.pivotToAngleAction(0.2 + extension * 0.2, 0.075),
            hardware.slideToPosAction(extension)
        )
    }

    fun readyToLvl1AscentAction(): Action {
        return SequentialAction(
            hardware.plungerAction(false, 0.0),
            hardware.wristPitchAction(0.6, 0.0),
            hardware.wristRollAction(0.4, 0.0),
            hardware.slideToPosAction(0.0),
            hardware.pivotToAngleAction(1.35),
            hardware.slideToPosAction(0.0),
        )
    }

    fun grabGamePieceAction(): Action {
        return SequentialAction(
            ParallelAction(
                hardware.plungerAction(false, 0.0),
                hardware.intakeAction(1.0, 1.0),
                hardware.pivotToAngleAction(0.0),
            ),
            hardware.intakeAction(0.0, 0.0),
            hardware.plungerAction(true, 0.0),
        )
    }

    fun readyToScoreSpecimenAction(): Action {
        return SequentialAction(
            hardware.plungerAction(true, 0.0),
            hardware.wristRollAction(0.0, 0.0),
            hardware.wristPitchAction(1.2, 0.0),
            hardware.intakeAction(1.0),
            ParallelAction(
                hardware.pivotToAngleAction(1.05),
                hardware.slideToPosAction(0.21, 0.04),
            ),
            hardware.intakeAction(0.0),
        )
    }

    fun scoreSpecimenAction(): Action {
        return SequentialAction(
            RaceAction(
                hardware.driveAction(PoseVelocity2d(Vector2d(-0.1, 0.0), 0.0), -1.0),
                SequentialAction(
                    SleepAction(0.5),
                    hardware.pivotToAngleAction(1.4),
                    hardware.intakeAction(-0.2, 0.1),
                    hardware.slideToPosAction(0.34),
                    hardware.intakeAction(1.0, 0.5),
                    hardware.pivotToAngleAction(1.5),
                    hardware.wristPitchAction(1.0, 0.1),
                    hardware.slideToPosAction(0.4),
                    hardware.intakeAction(-1.0, 1.0),
                )
            ),
            hardware.driveAction(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0), 0.0),
        )
    }

}