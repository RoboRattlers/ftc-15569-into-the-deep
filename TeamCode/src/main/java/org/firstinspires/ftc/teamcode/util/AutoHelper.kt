package org.firstinspires.ftc.teamcode.util

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
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.VelConstraint
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import kotlin.math.abs
import kotlin.math.sign

fun Pose2d.componentWisePlus(x: Double, y: Double, heading: Double): Pose2d {
    return Pose2d(
        this.position.plus(Vector2d(x, y)),
        this.heading.plus(heading)
    )
}

fun Pose2d.componentWisePlus(other: Pose2d): Pose2d {
    return Pose2d(
        this.position.plus(Vector2d(other.position.x, other.position.y)),
        this.heading.toDouble().plus(other.heading.toDouble())
    )
}
fun Pose2d.componentWiseMinus(other: Pose2d): Pose2d {
    return Pose2d(
        this.position.minus(Vector2d(other.position.x, other.position.y)),
        this.heading.toDouble() - other.heading.toDouble()
    )
}

enum class CoordinateSpace {
    ROBOT,
    FIELD
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
                hardware.wristPitchAction(0.6, 0.1),
                hardware.wristRollAction(-Math.PI/2, 0.1),
            ),
            hardware.slideToPosAction(0.1),
            hardware.pivotToAngleAction(Math.PI/2),
            hardware.slideToPosAction(1.0),
            hardware.wristRollAction(-0.4, 0.0),
        )
    }

    fun readyToGrabGamePieceAction(extension: Double, roll: Double): Action {
        return SequentialAction(
            hardware.wristPitchAction(-2.0, 0.0),
            hardware.wristRollAction(roll, 0.0),
            RaceAction(
                hardware.pivotToAngleAction(1.3),
                hardware.slideToPosAction(0.02),
            ),
            hardware.pivotToAngleAction(0.4 - extension * 0.1, 0.1),
            hardware.slideToPosAction(extension)
        )
    }

    fun readyToLvl1AscentAction(): Action {
        return SequentialAction(
            hardware.wristPitchAction(0.6, 0.0),
            hardware.wristRollAction(0.4, 0.0),
            hardware.slideToPosAction(0.0),
            hardware.pivotToAngleAction(Math.PI/2),
            hardware.slideToPosAction(0.0),
        )
    }

    fun grabGamePieceAction(): Action {
        return SequentialAction(
            ParallelAction(
                hardware.intakeAction(1.0, 1.0),
                hardware.pivotToAngleAction(0.0),
            ),
            hardware.intakeAction(0.0, 0.0),
        )
    }

    fun readyToScoreSpecimenAction(): Action {
        return SequentialAction(
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
                    hardware.pivotToAngleAction(1.45),
                    hardware.intakeAction(-0.2, 0.1),
                    hardware.slideToPosAction(0.34 * 5.0/4.0),
                    hardware.intakeAction(1.0, 0.5),
                    hardware.pivotToAngleAction(1.8),
                    hardware.wristPitchAction(1.0, 0.1),
                    hardware.slideToPosAction(0.5),
                    hardware.intakeAction(-1.0, 1.0),
                )
            ),
            hardware.driveAction(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0), 0.0),
        )
    }

    fun driveToPointAction(drive: MecanumDrive, point: Pose2d, space: CoordinateSpace, posThreshold: Double = 0.05, headingThreshold: Double = 0.05 ): Action {
        var targetPose = Pose2d(0.0, 0.0, 0.0)
        var initialized = false
        return Action {

            if (!initialized) {
                initialized = true
                targetPose = if (space == CoordinateSpace.ROBOT) drive.pose.times(point) else point
            }

            val error = targetPose.times(drive.pose.inverse())
            val pos_kS = 0.2
            val pos_kP = 4.0
            val heading_kP = 2.0
            val heading_kS = 0.2
            drive.setDrivePowers(PoseVelocity2d(
                Vector2d(
                    error.position.x * pos_kP + sign(error.position.x) * pos_kS,
                    error.position.y * pos_kP + sign(error.position.y) * pos_kS ),
                error.heading.toDouble() * heading_kP + sign(error.heading.toDouble()) * heading_kS
            ))

            return@Action error.position.norm() > posThreshold || abs(error.heading.toDouble()) > headingThreshold

        }
    }

}