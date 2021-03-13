package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.mechanisms.RingShooter;
import org.firstinspires.ftc.teamcode.utils.*;

public class RobotWood extends Robot {

    public MecanumDrive mecanumDrive;

    public GoalLift goalLift;

    public RingShooter ringShooter;

    public Tracking tracker;

    public TensorFlowUtil tensorFlowUtil;

    public static double MIN_POWER = 0.1;

    public static double FLY_WHEEL_RADIUS = 0;

    public static double PUSHED_POSTITION = 0.4;
    public static double RETRACTED_POSTITION = 0.05;

    public static double OPEN_POSTITION = 0.5;
    public static double CLOSED_POSTITION = 0.1;

    public RobotWood(HardwareMap hw, OpMode op){
        super(hw, op);

        super.driveTrain = new MecanumDrive(hw);
        mecanumDrive = (MecanumDrive) driveTrain;
        tracker = new Tracking(mecanumDrive, hw);
        goalLift = new GoalLift(hw, OPEN_POSTITION, CLOSED_POSTITION );
        ringShooter = new RingShooter(hw, FLY_WHEEL_RADIUS, PUSHED_POSTITION, RETRACTED_POSTITION );
        tensorFlowUtil = new TensorFlowUtil(hw, op);
    }

    public void dropOffGoal() { Robot.writeToDefaultFile("dropOffGOal()", true, true );

        TensorFlowUtil.Stack stackPos = tensorFlowUtil.getStack();

        switch( stackPos ) {
            case NONE:
                driveToZoneA();
                break;
            case SINGLE:
                driveToZoneB();
                break;
            case QUAD:
                driveToZoneC();
                break;
        }
    }

    public void driveToZoneA() { // 0 rings :: target zone A

        Robot.writeToDefaultFile( "driveToZoneA()", true, true);

        telemetry.addLine( "driveToZoneA()" );
        telemetry.update();

        driveDistance(12, -0.75, true);

        // drive backwards a bit
        // strafe right 4 feet
        // drop goal

    }

    public void driveToZoneB() { // 1 ring :: target zone B

        Robot.writeToDefaultFile( "driveToZoneB()", true, true);

        telemetry.addLine( "driveToZoneB()" );
        telemetry.update();

        strafeDistance(12, -0.75, true);

        // strafe right 2 feet
        // drive forward
        // drop goal

    }

    public void driveToZoneC() { // 4 rings :: target zone C

        Robot.writeToDefaultFile( "driveToZoneC()", true, true);

        telemetry.addLine( "driveToZoneC()" );
        telemetry.update();

        driveDistance(12, 0.75, true);

        // strafe right 4 feet
        // drive forward 4 feet
        // drop goal

    }

    public void drive(double drivePower, double strafePower, double rotatePower) {
        mecanumDrive.drive(drivePower, strafePower, rotatePower);
    }

    public void driveDistance( double distance, double power, boolean setPowerZero ) {

        mecanumDrive.drive( power, 0, 0 );

        int ticksToTravel = mecanumDrive.convertDistTicks(distance);
        int initialXPos = tracker.getLateralPosition();
        int initialYPos = tracker.getLongitudinalPosition();
        double percent = 0.5;

        mecanumDrive.drive( power, 0, 0 );
        while( tracker.getLateralPosition() - initialXPos < ticksToTravel && opModeIsActive() ) {

            /*
            double m = (power-(Math.signum(power)*MIN_POWER))/(-distance*(1-distance));
            double x = mecanumDrive.convertDistTicks(tracker.getLateralPosition() - ticksToTravel);
            double b = (Math.signum(power)*MIN_POWER)-m*distance;

            if( tracker.getLateralPosition() - initialXPos > ticksToTravel*percent )
                power = m*x + b;

             */

            mecanumDrive.drive( power, 0, 0 );
        }

        //sets all power to zero afterwords
        if(setPowerZero)
            mecanumDrive.drive( 0, 0, 0 );
    }

    public void strafeDistance( double distance, double power, boolean setPowerZero ) {

        mecanumDrive.drive( 0, power, 0 );

        int ticksToTravel = mecanumDrive.convertDistTicks(distance);
        int initialXPos = tracker.getLateralPosition();
        int initialYPos = tracker.getLongitudinalPosition();
        double percent = 0.5;

        mecanumDrive.drive( 0, power, 0 );
        while( tracker.getLongitudinalPosition() - initialYPos < ticksToTravel && opModeIsActive()) {

            double m = (power-(Math.signum(power)*MIN_POWER))/(-distance*(1-distance));
            double x = mecanumDrive.convertDistTicks(tracker.getLateralPosition() - ticksToTravel);
            double b = (Math.signum(power)*MIN_POWER)-m*distance;

            if( tracker.getLongitudinalPosition() - initialXPos > ticksToTravel*percent )
                power = m*x + b;

            mecanumDrive.drive( 0, power, 0 );
        }

        //sets all power to zero afterwords
        if(setPowerZero)
            mecanumDrive.drive( 0, 0, 0 );
    }

    /**
     *
     * @param degrees forward is zero, turning right is positive, limit: 359 degrees
     * @param power positive will turn right, negative turns left
     * @param setPowerZero if true, will set the power to zero once finished
     */
    public void rotateDegrees( double degrees, double power, boolean setPowerZero ) {

        mecanumDrive.drive( 0, 0, power );

        double initialDegrees = tracker.getGyroHeading();

        if( degrees > 0 )
            while( tracker.getGyroHeading() - initialDegrees < degrees && opModeIsActive())
                mecanumDrive.drive( 0, 0, power );
        else if( degrees < 0 )
            while( tracker.getGyroHeading() - initialDegrees > degrees && opModeIsActive())
                mecanumDrive.drive( 0, 0, power );

        //sets all power to zero afterwords
        if(setPowerZero) {
            mecanumDrive.drive( 0, 0, 0 );
        }


    }

    /**
     *
     * @param degrees forward is zero, turning right is positive, limit: 359 degrees
     * @param power positive will turn right, negative turns left
     */
    public void rotateDegrees( double degrees, double power ) throws InterruptedException {

        //mecanumDrive.drive( 0, 0, power );

        double initialDegrees = tracker.get360GyroHeading();
        double percent = 0.5;
        double variable = 5*power;

        if (power > 0) {
            mecanumDrive.drive( 0, 0, power );
            sleep( 250 );
            while( opModeIsActive() && tracker.get360GyroHeading() - initialDegrees < degrees - variable )
            {
                telemetry.addData("While:", tracker.get360GyroHeading() - initialDegrees < degrees)
                        .addData("While:", tracker.get360GyroHeading() - initialDegrees > degrees)
                        .addData("Heading", tracker.get360GyroHeading());
                telemetry.update();

                double m = (power-(Math.signum(power)*MIN_POWER))/(-degrees*(1-percent));
                double x = tracker.get360GyroHeading() - initialDegrees;
                double b = (Math.signum(power)*MIN_POWER)-m*degrees;

                if( tracker.get360GyroHeading() - initialDegrees > degrees*percent )
                    power = m*x + b;
                mecanumDrive.drive( 0, 0, power );
            }
            mecanumDrive.drive( 0, 0, 0 );
        }
        else if ( power != 0 )
        {
            mecanumDrive.drive( 0, 0, power );
            while( opModeIsActive() && tracker.get360GyroHeading() - initialDegrees > degrees + variable )
            {
                telemetry.addData("While:", tracker.get360GyroHeading() - initialDegrees < degrees)
                        .addData("While:", tracker.get360GyroHeading() - initialDegrees > degrees)
                        .addData("Heading", tracker.get360GyroHeading());
                telemetry.update();

                double m = (power-(Math.signum(power)*MIN_POWER))/(-degrees*(1-percent));
                double x = tracker.get360GyroHeading() - initialDegrees;
                double b = (Math.signum(power)*MIN_POWER)-m*degrees;

                if( tracker.get360GyroHeading() - initialDegrees > degrees*percent )
                    power = m*x + b;
                mecanumDrive.drive( 0, 0, power );
                mecanumDrive.drive( 0, 0, power );
            }
            mecanumDrive.drive( 0, 0, 0 );
        }

        /*
        while( opModeIsActive())
        {
            telemetry.addData("getGyroHeading", tracker.getGyroHeading())
                    .addData("getNewGyroHeading", tracker.getNewGyroHeading());
            telemetry.update();
        }*/


    }

    /**
     * @param drivePower - sets power to drive - negative power is backwards
     * @param strafePower - sets power to strafe - negative power is left
     * @param time - amount of time to run the motors in MILLISECONDS
     */
    public void omniTime(double drivePower, double strafePower, long time, boolean setPowerZero)
    {
        //gyro.resetZAxisIntegrator();
        //set power to 'drive' motors

        mecanumDrive.drive(drivePower, strafePower, 0);

        //wait for certain amount of time while motors are running
        //robotMecanum.wait(time);
        long setTime = System.currentTimeMillis();
        previousTime = opMode.getRuntime();

        while(System.currentTimeMillis() - setTime < (time) && opModeIsActive())
        {
            mecanumDrive.drive(drivePower, strafePower, /*gyroPID(180, opMode.getRuntime() - previousTime)*/0);
        }

        //sets all power to zero afterwords
        if(setPowerZero) {
            mecanumDrive.drive( 0, 0, 0 );
        }
    }


}
