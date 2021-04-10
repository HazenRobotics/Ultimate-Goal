package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveTechnicolor;
import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.mechanisms.RingShooter;
import org.firstinspires.ftc.teamcode.utils.FieldMap;
import org.firstinspires.ftc.teamcode.utils.SoundLibrary;
import org.firstinspires.ftc.teamcode.utils.TensorFlowUtil;
import org.firstinspires.ftc.teamcode.utils.Vuforia;
import org.firstinspires.ftc.teamcode.utils.VuforiaLocalization;
import org.jetbrains.annotations.NotNull;

public class RobotTechnicolorRR {

    public RRMecanumDriveTechnicolor drive;

    public GoalLift goalLift;
    public RingShooter ringShooter;
    public TensorFlowUtil tfod;

    private RevBlinkinLedDriver lights;
    private VoltageSensor batteryVoltageSensor;

    private final double FLY_WHEEL_RADIUS = 4; //in inches

    public static double PUSHED_POSTITION = 0.2 ;
    public static double RETRACTED_POSTITION = 0.0;

    public static double OPEN_POSTITION = 0.0;
    public static double CLOSED_POSTITION = 1.0;

    public static boolean REVERSE_LIFT_DIRECTION = false ;
    public static boolean REVERSE_SHOOTER_DIRECTION = false;

    public RobotTechnicolorRR(HardwareMap hw, OpMode op) {
        drive = new RRMecanumDriveTechnicolor(hw);
        goalLift = new GoalLift(hw, OPEN_POSTITION, CLOSED_POSTITION, REVERSE_LIFT_DIRECTION);
        ringShooter = new RingShooter(hw, FLY_WHEEL_RADIUS, PUSHED_POSTITION, RETRACTED_POSTITION, REVERSE_SHOOTER_DIRECTION);
        tfod = new TensorFlowUtil(hw, op);

        final String VUFORIA_KEY = hw.appContext.getResources().getString(R.string.vuforiakey);
        Vuforia.getInstance().setParameters(VUFORIA_KEY, "webcam", true, hw);

        batteryVoltageSensor = hw.voltageSensor.iterator().next();
        new SoundLibrary(hw);
        //lights = hw.get(RevBlinkinLedDriver.class, "lights");
    }



    public void sleepRobot(long delay) {
        long setTime = System.currentTimeMillis();
        while( (System.currentTimeMillis() - setTime)*1000 < (delay) );
    }

    /**
     * Shoots a ring at a specified target
     * @param target target at which to shoot a ring at
     */
    public void shootAtTarget(OpenGLMatrix target, boolean setSpeedZero, boolean speedUpTime) {
        ringShooter.setFlyWheelPID(new PIDFCoefficients(5, 0, 2, 12.5 * 12 / batteryVoltageSensor.getVoltage()));
        //rotate towards target
        //drive.turn(ShootingMath.getAngleToTarget(drive.getPoseEstimate(), FieldMap.toInches(FieldMap.toVectorF(target))));
        //assuming we are now lined up for the shot
        //shoot using velocity required to hit the target
        // backup shoot using power ringShooter.launchRingPower(0.85);
        ringShooter.launchRingAngularVelocity( 9.3, setSpeedZero, speedUpTime );
        //ringShooter.launchRingVelocity(ShootingMath.getVelocityToTarget(FieldMap.RobotInfo.getRingLaunchPointPosition().toVector(), target.toVector(), ringShooter.getLaunchAngle()), DistanceUnit.MM);
    }

    /**
     * Sets the position of the lift
     * @param liftPosition position at which to move the lift to
     * @param liftPower power at which to move the lift
     */
    public void setLiftPosition(GoalLift.LiftPosition liftPosition, double liftPower) {
        goalLift.setGoalLiftPosition(liftPosition, liftPower, 1000);
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
        return drive.trajectoryBuilder(drive.getPoseEstimate());
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

    public void teleopDrive(double forwardPower, double strafePower, double turnPower) {
        // You might have to play with the + or - depending on how your motors are installed
        double frontLeftPower  = forwardPower + strafePower - turnPower;
        double backLeftPower   = forwardPower - strafePower - turnPower;
        double frontRightPower = forwardPower - strafePower + turnPower;
        double backRightPower  = forwardPower + strafePower + turnPower;

        drive.setMotorPowers( frontLeftPower, backLeftPower, backRightPower, frontRightPower );
    }

}
