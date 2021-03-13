package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveTechnicolor;
import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.mechanisms.RingShooter;
import org.firstinspires.ftc.teamcode.utils.FieldMap;
import org.firstinspires.ftc.teamcode.utils.TensorFlowUtil;
import org.jetbrains.annotations.NotNull;

public class RobotTechnicolorRR {

    public RRMecanumDriveTechnicolor drive;

    public GoalLift goalLift;
    public RingShooter ringShooter;
    public TensorFlowUtil tfod;

    private final double FLY_WHEEL_RADIUS = 4; //in inches

    public static double PUSHED_POSTITION = 0.3;
    public static double RETRACTED_POSTITION = 0.07;

    public static double OPEN_POSTITION = 1.0;
    public static double CLOSED_POSTITION = 0.0;

    public RobotTechnicolorRR(HardwareMap hw, OpMode op) {
        drive = new RRMecanumDriveTechnicolor(hw);
        goalLift = new GoalLift(hw, OPEN_POSTITION, CLOSED_POSTITION);
        ringShooter = new RingShooter(hw, FLY_WHEEL_RADIUS, PUSHED_POSTITION, RETRACTED_POSTITION);
        tfod = new TensorFlowUtil(hw, op);
    }

    /**
     * Shoots a ring at a specified target
     * @param target target at which to shoot a ring at
     */
    public void shootAtTarget(OpenGLMatrix target) {
        //rotate towards target
        //drive.turn(ShootingMath.getAngleToTarget(drive.getPoseEstimate(), FieldMap.toInches(FieldMap.toVectorF(target))));
        //assuming we are now lined up for the shot
        //shoot using velocity required to hit the target
        ringShooter.launchRingPower(0.85);
        //ringShooter.launchRingVelocity(ShootingMath.getVelocityToTarget(FieldMap.RobotInfo.getRingLaunchPointPosition().toVector(), target.toVector(), ringShooter.getLaunchAngle()), DistanceUnit.MM);
    }

    /**
     * Sets the position of the lift
     * @param liftPosition position at which to move the lift to
     * @param liftPower power at which to move the lift
     */
    public void setLiftPosition(GoalLift.LiftPosition liftPosition, double liftPower) {
        goalLift.setGoalLiftPosition(liftPosition, liftPower);
    }

    public void setIntakePower(double power) {
        ringShooter.setIntakeMotorPower(power);
    }

    public void driveTo(@NotNull Pose2d... poses) {
        TrajectoryBuilder trajectoryBuilder = trajectoryBuilder();
        for(Pose2d pose : poses) {
            trajectoryBuilder = trajectoryBuilder.splineToLinearHeading(pose, 0);
        }
        drive(trajectoryBuilder.build());
    }

    public TrajectoryBuilder trajectoryBuilder() {
        //If the robot's location is known and stored in field map, use that. else, get a pose estimate from the dead wheels only
        return FieldMap.RobotInfo.robotLocation != null ? drive.trajectoryBuilder(FieldMap.toPose2d(FieldMap.RobotInfo.robotLocation)) : drive.trajectoryBuilder(drive.getPoseEstimate());
    }

    public void drive(Trajectory trajectory) {
        drive.followTrajectory(trajectory);
    }

    public void driveAsync(Trajectory trajectory) {
        drive.followTrajectoryAsync(trajectory);
    }

    public void setPosition(Pose2d currentPosition) {
        drive.setPoseEstimate(currentPosition);
    }


}
