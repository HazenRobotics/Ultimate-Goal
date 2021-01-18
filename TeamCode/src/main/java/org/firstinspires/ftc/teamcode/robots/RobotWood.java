package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.*;

public class RobotWood extends Robot {

    MecanumDrive mecanumDrive;
    public Tracking tracker;

    public static double MIN_POWER = 0.4;

    public RobotWood(HardwareMap hw, OpMode op){
        super(hw, op);
        super.driveTrain = new MecanumDrive(hw);
        mecanumDrive = (MecanumDrive) driveTrain;
        tracker = new Tracking( mecanumDrive, hw );

    }

    public void driveDistance( double distance, double maxPower, boolean setPowerZero ) {

        int ticksToTravel = mecanumDrive.convertDistTicks(distance);
        int initialXPos = tracker.getLongitudinalPosition();
        double percent = 0.25;
        //int initialYPos = tracker.getLateralPosition();

        double curPower;

        curPower = powerSlopeCalculations( maxPower, ticksToTravel, initialXPos, percent );
        mecanumDrive.drive( curPower, 0, 0 );

        while( tracker.getLongitudinalPosition() - initialXPos < ticksToTravel && opModeIsActive()) {

            curPower = powerSlopeCalculations( maxPower, ticksToTravel, initialXPos, percent );

            mecanumDrive.drive( curPower, 0, 0 );
        }

        //sets all power to zero afterwords
        if(setPowerZero) {
            mecanumDrive.drive( 0, 0, 0 );
        }


    }

    public void strafeDistance( double distance, double maxPower, boolean setPowerZero ) {

        int ticksToTravel = mecanumDrive.convertDistTicks(distance);
        int initialXPos = tracker.getLongitudinalPosition();
        int initialYPos = tracker.getLateralPosition();
        double percent = 0.25;

        double curPower;

        curPower = powerSlopeCalculations( maxPower, ticksToTravel, initialXPos, percent );
        mecanumDrive.drive( 0, curPower, 0 );

        while( tracker.getLateralPosition() - initialYPos < ticksToTravel && opModeIsActive()) {

            curPower = powerSlopeCalculations( maxPower, ticksToTravel, initialYPos, percent );

            mecanumDrive.drive(0, curPower, 0);
        }

        //sets all power to zero afterwords
        if(setPowerZero) {
            mecanumDrive.drive( 0, 0, 0 );
        }


    }

    /**
     *
     * @param degrees forward is zero, turning right is positive, limit: 359 degrees
     * @param maxPower positive will turn right, negative turns left
     * @param setPowerZero if true, will set the power to zero once finished
     */
    public void rotateDegrees( double degrees, double maxPower, boolean setPowerZero ) {

        mecanumDrive.drive( 0, 0, maxPower );

        double initialDegrees = tracker.getGyroHeading();
        double percent = 0.25;

        double a, m, x, b, curPower;

        if( degrees > 0 ) {
            while (tracker.getGyroHeading() - initialDegrees < degrees && opModeIsActive()) {

                /*
                a = (MIN_POWER - maxPower)/Math.pow(degrees - percent * degrees, 2);
                x = tracker.getNewGyroHeading() - initialDegrees;

                curPower = a * Math.pow(x - percent * degrees, 2) + maxPower;

                telemetry.addLine("long :" + tracker.getLongitudinalPosition() );
                telemetry.addLine("lat  :" + tracker.getLateralPosition() );
                telemetry.addLine("-----------------");

                telemetry.addLine("a :: " + a );
                telemetry.addLine("x :: " + x );
                telemetry.addLine("c :: " + curPower );
                telemetry.update();
                 */

                curPower = powerSlopeCalculations( maxPower, degrees, initialDegrees, percent );

                mecanumDrive.drive(0, 0, maxPower);
            }
        } else if( degrees < 0 ) {
            while (tracker.getGyroHeading() - initialDegrees > degrees && opModeIsActive()) {

                /*
                a = (MIN_POWER - maxPower)/Math.pow(degrees - percent * degrees, 2);
                x = tracker.getNewGyroHeading() - initialDegrees;

                curPower = a * Math.pow(x - percent * degrees, 2) + maxPower;

                telemetry.addLine("long :" + tracker.getLongitudinalPosition() );
                telemetry.addLine("lat  :" + tracker.getLateralPosition() );
                telemetry.addLine("-----------------");

                telemetry.addLine("a :: " + a );
                telemetry.addLine("x :: " + x );
                telemetry.addLine("c :: " + curPower );
                telemetry.update();
                 */

                curPower = powerSlopeCalculations( maxPower, degrees, initialDegrees, percent );

                mecanumDrive.drive(0, 0, maxPower);
            }
        }

        //sets all power to zero afterwords
        if(setPowerZero) {
            mecanumDrive.drive( 0, 0, 0 );
        }


    }

    public double powerSlopeCalculations( double maxPower, double movement, double initialMovement, double percent ) {

        double a, x, curPower;

        a = (MIN_POWER - maxPower)/Math.pow( movement - percent * movement, 2);
        x = tracker.getNewGyroHeading() - initialMovement;

        curPower = a * Math.pow(x - percent * movement, 2) + maxPower;

        telemetry.addLine("long :" + tracker.getLongitudinalPosition() );
        telemetry.addLine("lat  :" + tracker.getLateralPosition() );
        telemetry.addLine("-----------------");

        telemetry.addLine("a :: " + a );
        telemetry.addLine("x :: " + x );
        telemetry.addLine("c :: " + curPower );
        telemetry.update();

        return curPower;
    }

    /**
     *
     * @param degrees forward is zero, turning right is positive, limit: 359 degrees
     * @param maxPower positive will turn right, negative turns left
     */
    public void rotateDegrees( double degrees, double maxPower ) throws InterruptedException {

        mecanumDrive.drive( 0, 0, maxPower );

        double initialDegrees = tracker.getNewGyroHeading();
        double variable = 5*maxPower;
        double percent = 0.25;

        double a, m, x, b, curPower;

        if (maxPower > 0) {
            while( opModeIsActive() && tracker.getNewGyroHeading() - initialDegrees < degrees - variable )
            {
                telemetry.addData("While:", tracker.getNewGyroHeading() - initialDegrees < degrees)
                        .addData("While:", tracker.getNewGyroHeading() - initialDegrees > degrees)
                        .addData("Heading", tracker.getNewGyroHeading());

                /*
                a = (MIN_POWER - maxPower)/Math.pow(degrees - percent * degrees, 2);
                x = tracker.getNewGyroHeading() - initialDegrees;

                curPower = a * Math.pow(x - percent * degrees, 2) + maxPower;

                telemetry.addLine("long :" + tracker.getLongitudinalPosition() );
                telemetry.addLine("lat  :" + tracker.getLateralPosition() );
                telemetry.addLine("-----------------");

                telemetry.addLine("a :: " + a );
                telemetry.addLine("x :: " + x );
                telemetry.addLine("c :: " + curPower );
                telemetry.update();
                */

                curPower = powerSlopeCalculations( maxPower, degrees, initialDegrees, percent );

                mecanumDrive.drive( 0, 0, curPower );
            }
            mecanumDrive.drive( 0, 0, 0 );
        }
        else if ( maxPower != 0 )
        {
            while( opModeIsActive() && tracker.getNewGyroHeading() - initialDegrees > degrees + variable )
            {
                telemetry.addData("While:", tracker.getNewGyroHeading() - initialDegrees < degrees)
                        .addData("While:", tracker.getNewGyroHeading() - initialDegrees > degrees)
                        .addData("Heading", tracker.getNewGyroHeading());

                /*
                a = (MIN_POWER - maxPower)/Math.pow(degrees - percent * degrees, 2);
                x = tracker.getNewGyroHeading() - initialDegrees;

                curPower = a * Math.pow(x - percent * degrees, 2) + maxPower;

                telemetry.addLine("long :" + tracker.getLongitudinalPosition() );
                telemetry.addLine("lat  :" + tracker.getLateralPosition() );
                telemetry.addLine("-----------------");

                telemetry.addLine("a :: " + a );
                telemetry.addLine("x :: " + x );
                telemetry.addLine("c :: " + curPower );
                telemetry.update();
                */

                curPower = powerSlopeCalculations( maxPower, degrees, initialDegrees, percent );

                mecanumDrive.drive( 0, 0, curPower );
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
     *
     * @param time - time to move the robot
     * @param power - power the wheels to move the robot
     * @param setPowerZero - (boolean) set power to zero after moving
     */
    public void driveTime( long time, double power, boolean setPowerZero ) {

        mecanumDrive.drive( power, 0, 0 );

        //wait for certain amount of time while motors are running
        long setTime = System.currentTimeMillis();
        previousTime = opMode.getRuntime();

        while(System.currentTimeMillis() - setTime < (time) && opModeIsActive()) {
            mecanumDrive.drive( power, 0, 0 );
        }

        //sets all power to zero afterwords
        if(setPowerZero) {
            mecanumDrive.drive( 0, 0, 0 );
        }
    }

    /**
     *
     * @param time - time to strafe the robot
     * @param power - power for the wheels to strafe the robot
     * @param setPowerZero - (boolean) set power to zero after strafing
     */
    public void strafeTime( long time, double power, boolean setPowerZero ) {

        mecanumDrive.drive( 0, power, 0 );

        //wait for certain amount of time while motors are running
        long setTime = System.currentTimeMillis();
        previousTime = opMode.getRuntime();

        while(System.currentTimeMillis() - setTime < (time) && opModeIsActive()) {
            mecanumDrive.drive( 0, power, 0 );
        }

        //sets all power to zero afterwords
        if(setPowerZero) {
            mecanumDrive.drive( 0, 0, 0 );
        }
    }

    /**
     *
     * @param time - time to strafe the robot
     * @param power - power for the wheels to strafe the robot
     * @param setPowerZero - (boolean) set power to zero after strafing
     */
    public void rotateTime( long time, double power, boolean setPowerZero ) {

        mecanumDrive.drive( 0, 0, power );

        //wait for certain amount of time while motors are running
        long setTime = System.currentTimeMillis();
        previousTime = opMode.getRuntime();

        while(System.currentTimeMillis() - setTime < (time) && opModeIsActive()) {
            mecanumDrive.drive( 0, 0, power );
        }

        //sets all power to zero afterwords
        if(setPowerZero) {
            mecanumDrive.drive( 0, 0, 0 );
        }
    }

    /**
     *
     * @param time - time to rotate the robot in milliseconds
     * @param power - power for the wheels to rotate the robot
     * @param setPowerZero - (boolean) set power to zero after rotating
     */
    public void turnTime( long time, double power, boolean setPowerZero ) {

        mecanumDrive.drive( 0, 0, power );

        //wait for certain amount of time while motors are running
        long setTime = System.currentTimeMillis();
        previousTime = opMode.getRuntime();

        while(System.currentTimeMillis() - setTime < (time) && opModeIsActive()) {
            mecanumDrive.drive( 0, 0, power );
        }

        //sets all power to zero afterwords
        if(setPowerZero) {
            mecanumDrive.drive( 0, 0, 0 );
        }

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
