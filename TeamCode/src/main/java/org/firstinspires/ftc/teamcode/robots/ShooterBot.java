package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.mechanisms.RingShooter;
import org.firstinspires.ftc.teamcode.utils.FieldMap;
import org.firstinspires.ftc.teamcode.utils.ShootingMath;

public class ShooterBot extends Robot {

    public MecanumDrive mecanumDrive;

    public GoalLift goalLift;
    public RingShooter ringShooter;

    public static double PUSHED_POSTITION = 0.5;
    public static double RETRACTED_POSTITION = 0.25;

    public static double OPEN_POSTITION = 1.0;
    public static double CLOSED_POSTITION = 0.0;

    public static boolean REVERSE_MOTOR_DIRECTION = true;

    //public Tracking tracker;

    private final double FLY_WHEEL_RADIUS = 0;
    private final double tangentalVelocityMultiplier = 1; //calculated tangental velocity / actual velocity

    /**
     * Creates a Robot
     *
     * @param hw robot's hardware map
     * @param op
     */
    public ShooterBot(HardwareMap hw, OpMode op) {
        super(hw, op);

        super.driveTrain = new MecanumDrive(hw);
        mecanumDrive = (MecanumDrive) driveTrain;
        //tracker = new Tracking(mecanumDrive, hw);
        goalLift = new GoalLift(hw, OPEN_POSTITION, CLOSED_POSTITION);
        ringShooter = new RingShooter(hw, FLY_WHEEL_RADIUS, PUSHED_POSTITION, RETRACTED_POSTITION, REVERSE_MOTOR_DIRECTION);
    }

    /**
     * Shoots a ring at a specified target
     * @param target target at which to shoot a ring at
     * @param turnPower power at which to turn
     */
    public void shootAtTarget(OpenGLMatrix target, double turnPower) {
        //rotate towards target
        //rotateDegrees(ShootingMath.getAngleToTarget(FieldMap.RobotInfo.robotLocation.toVector(), target.toVector()), turnPower);
        //assuming we are now lined up for the shot
        //shoot using velocity required to hit the target
        ringShooter.launchRingVelocity(ShootingMath.getVelocityToTarget(FieldMap.RobotInfo.getRingLaunchPointPosition().toVector(), target.toVector(), ringShooter.getLaunchAngle()), DistanceUnit.MM);
    }

    /**
     * Sets the power of the ring intake
     * @param power power at which to set the intake power
     */
    public void setIntakePower(double power) {
        ringShooter.setIntakeMotorPower(power);
    }

    /**
     * Sets the position of the lift
     * @param liftPosition position at which to move the lift to
     * @param liftPower power at which to move the lift
     */
    public void setLiftPosition(GoalLift.LiftPosition liftPosition, double liftPower) {
        goalLift.setGoalLiftPosition(liftPosition, liftPower, 1000);
    }

    public void setClawPosition(GoalLift.ClawPosition clawPosition) {
        goalLift.setClawPosition(clawPosition);
    }

    /**
     *
     * @param degrees forward is zero, turning right is positive, limit: 359 degrees
     * @param power positive will turn right, negative turns left
     */
    /*public void rotateDegrees( double degrees, double power ) {

        mecanumDrive.drive( 0, 0, power );

        double initialDegrees = tracker.getGyroHeading();

        if( degrees > 0 )
            while( tracker.getGyroHeading() - initialDegrees < degrees && opModeIsActive())
                mecanumDrive.drive( 0, 0, power );
        else if( degrees < 0 )
            while( tracker.getGyroHeading() - initialDegrees > degrees && opModeIsActive())
                mecanumDrive.drive( 0, 0, power );

        //sets all power to zero afterwords
        mecanumDrive.drive( 0, 0, 0 );


    }*/
    public void drive(double forwardPower, double strafePower, double turnPower) {
        mecanumDrive.drive(-forwardPower, strafePower, turnPower);
    }
}
