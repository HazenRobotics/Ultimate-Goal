package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.RoadRunnerMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.mechanisms.RingShooter;
import org.firstinspires.ftc.teamcode.utils.FieldMap;
import org.firstinspires.ftc.teamcode.utils.ShootingMath;
import org.firstinspires.ftc.teamcode.utils.Tracking;

public class ShooterBotRR {

    RoadRunnerMecanumDrive drive;

    GoalLift goalLift;
    RingShooter ringShooter;
    private final double FLY_WHEEL_RADIUS = 4; //in inches

    public ShooterBotRR(HardwareMap hw) {
        drive = new RoadRunnerMecanumDrive(hw);
        goalLift = new GoalLift(hw);
        ringShooter = new RingShooter(hw, FLY_WHEEL_RADIUS);
    }

    /**
     * Shoots a ring at a specified target
     * @param target target at which to shoot a ring at
     * @param turnPower power at which to turn
     */
    public void shootAtTarget(OpenGLMatrix target, double turnPower) {
        //rotate towards target
        drive.turn(ShootingMath.getAngleToTarget(FieldMap.RobotInfo.robotLocation.toVector(), target.toVector()));
        //assuming we are now lined up for the shot
        //shoot using velocity required to hit the target
        ringShooter.launchRingVelocity(ShootingMath.getVelocityToTarget(FieldMap.RobotInfo.getRingLaunchPointPosition().toVector(), target.toVector(), ringShooter.getLaunchAngle()), DistanceUnit.MM);
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

    public void driveTo(Pose2d... poses) {
        TrajectoryBuilder trajectoryBuilder = trajectoryBuilder();
        for(Pose2d pose : poses) {
            trajectoryBuilder = trajectoryBuilder.splineToLinearHeading(pose, 0);
        }
        drive(trajectoryBuilder.build());
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return drive.trajectoryBuilder(FieldMap.toPose2d(FieldMap.RobotInfo.robotLocation));
    }

    public void drive(Trajectory trajectory) {
        drive.followTrajectory(trajectory);
    }

    public void driveAsync(Trajectory trajectory) {
        drive.followTrajectoryAsync(trajectory);
    }


}
