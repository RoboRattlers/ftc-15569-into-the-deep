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
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.VelConstraint
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.util.HardwareConstants.FLOOR_FRICTION_MULTIPLIER
import java.util.function.Supplier
import kotlin.math.abs
import kotlin.math.sign

fun Pose2d.componentWisePlus(x: Double, y: Double, heading: Double): Pose2d {
    return Pose2d(
        this.position.plus(Vector2d(x, y)),
        this.heading.plus(heading)
    )
}

fun Pose2d.twistify(): Twist2d {
    return this.minus(Pose2d(0.0, 0.0, 0.0))
}

fun Pose2d.transformBy(other: Pose2d): Pose2d {
    return this.plus(other.twistify())
}

fun Twist2d.poseify(): Pose2d {
    return Pose2d(0.0, 0.0, 0.0).plus(this)
}

fun Vector2d.safeUnit(): Vector2d {
    return if (this.norm() > 0.0) this.div(this.norm()) else this
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

    fun timeoutAction(action: Action, timeout: Double): Action {
        return RaceAction(SleepAction(timeout), action)
    }

    fun readyToScoreInBasketAction(): Action {
        return SequentialAction(
            ParallelAction(
                hardware.wristPitchAction(1.5, 0.0),
                hardware.wristRollAction(-Math.PI/2, 0.0),
                hardware.pivotToAngleAction(Math.PI/2, 0.3),
                SequentialAction(
                    hardware.slideToPosAction(0.1, 0.2),
                    Action { return@Action hardware.getCurrentPivotAngle() < 0.7 }, // wait until mostly vertical
                    hardware.slideToPosAction(1.0)
                )
            )
        )
    }

    fun readyToGrabGamePieceAction(extension: Double, roll: Double): Action {
        return SequentialAction(
            hardware.wristPitchAction(-Math.PI/2, 0.05),
            hardware.wristRollAction(roll, 0.05),
            RaceAction(
                SleepAction(0.5),
                hardware.slideToPosAction(0.5)
            ),
            ParallelAction(
                hardware.pivotToAngleAction(1.35, 0.08),
                hardware.slideToPosAction(0.1, 0.15),
            ),
            hardware.pivotToAngleAction(0.25 - extension * 0.06, 0.1),
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
                hardware.intakeAction(1.0, 0.5),
                hardware.pivotToAngleAction(0.0),
            ),
            hardware.intakeAction(0.0, 0.0),
        )
    }

    fun readyToGrabSpecimenAction(): Action {
        return ParallelAction(
            hardware.slideToPosAction(0.0),
            hardware.pivotToAngleAction(0.35),
            hardware.wristPitchAction(-0.2, 0.5)
        )
    }

    fun readyToScoreSpecimenAction(): Action {
        return SequentialAction(
            hardware.wristRollAction(0.0, 0.0),
            hardware.wristPitchAction(1.4, 0.0),
            timeoutAction(ParallelAction(
                hardware.intakeAction(1.0, 0.5),
                hardware.pivotToAngleAction(1.55),
                hardware.slideToPosAction(0.275, 0.03),
            ), 0.6),
        )
    }

    fun scoreSpecimenAction(drive: MecanumDrive?): Action {
        val drivePower = PoseVelocity2d(Vector2d(-0.2, 0.0), 0.0)
        return SequentialAction(
            RaceAction(
                if (drive == null) hardware.driveAction(drivePower, -1.0)
                    else Action { drive.updatePoseEstimate(); drive.setDrivePowers(drivePower); return@Action true },
                timeoutAction(SequentialAction(
                    hardware.wristPitchAction(1.5, 0.5),
                    timeoutAction(ParallelAction(
                    hardware.pivotToAngleAction(1.8),
                    hardware.slideToPosAction(0.5)), 0.9),
                    //hardware.intakeAction(-1.0, 0.0),
                ), 3.0)
            ),
            if (drive == null) hardware.driveAction(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0), 0.0)
                else Action { drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)); return@Action false }
        )
    }

    fun driveToPointAction(drive: MecanumDrive, point: () -> Pose2d, space: CoordinateSpace,
                           posThreshold: Double = 0.6,
                           headingThreshold: Double = Math.toRadians(3.0),
                           linearVelThreshold: Double = 0.6,
                           angVelThreshold: Double = Math.toRadians(15.0)): Action {
        var targetPose = Pose2d(0.0, 0.0, 0.0)
        var initialized = false
        val timer = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)
        var lastTime = 0.0
        return Action {

            drive.updatePoseEstimate()
            if (!initialized) {
                initialized = true
                timer.reset()
                lastTime = timer.seconds()
                targetPose = if (space == CoordinateSpace.ROBOT) drive.pose.plus(point.invoke().minus(Pose2d(0.0, 0.0, 0.0)))
                    else point.invoke()
            }

            val error = Pose2d(0.0, 0.0, 0.0).plus(targetPose.minus(drive.pose))
            val pos_axial_kS = 0.06 * FLOOR_FRICTION_MULTIPLIER
            val pos_lateral_kS = 0.13 * FLOOR_FRICTION_MULTIPLIER
            val pos_kP = 0.1
            val pos_kD = 0.0
            val heading_kP = 0.2
            val heading_kS = 0.04 * FLOOR_FRICTION_MULTIPLIER
            hardware.telemetry.clearAll()
            val currentTime = timer.seconds()
            val dt = currentTime - lastTime
            lastTime = currentTime

            //hardware.telemetry.addData("Hz", (1.0 / dt).toInt())
            //hardware.telemetry.addData("Error X", error.position.x)
            //hardware.telemetry.addData("Error Y", error.position.y)
            //hardware.telemetry.addData("Error heading", Math.toDegrees(error.heading.toDouble()))

            val axialness = abs(error.position.safeUnit().dot(Vector2d(1.0, 0.0)))

            drive.setDrivePowers(
                PoseVelocity2d(
                    error.position * pos_kP
                            - drive.velocityRobot.linearVel * pos_kD
                            + error.position.safeUnit() * MathUtils.lerp(pos_lateral_kS, pos_axial_kS, axialness),

                    error.heading.toDouble() * heading_kP + sign(error.heading.toDouble()) * heading_kS
                )
            )

            val shouldContinue = error.position.norm() > posThreshold
                    || abs(error.heading.toDouble()) > headingThreshold
                    || drive.velocityRobot.linearVel.norm() > linearVelThreshold
                    || drive.velocityRobot.angVel > angVelThreshold
            if (!shouldContinue) {
                drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
            }
            //hardware.telemetry.addData("Continue?", shouldContinue)
            //hardware.telemetry.update()

            shouldContinue

        }
    }

    fun driveToPointAction(drive: MecanumDrive, point: Pose2d, space: CoordinateSpace,
        posThreshold: Double = 0.6,
        headingThreshold: Double = Math.toRadians(3.0),
        linearVelThreshold: Double = 0.6,
        angVelThreshold: Double = Math.toRadians(15.0)): Action {
        return driveToPointAction(drive, { point }, space, posThreshold, headingThreshold, linearVelThreshold, angVelThreshold)
    }


}