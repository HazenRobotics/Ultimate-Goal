package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.*;

public class RobotWood extends Robot {

    MecanumDrive mecanumDrive;
    Tracking tracker;

    public RobotWood(HardwareMap hw, OpMode op){
        super(hw, op);
        super.driveTrain = new MecanumDrive(hw);
        mecanumDrive = (MecanumDrive) driveTrain;
        tracker = new Tracking( mecanumDrive, hw );

    }

    public void driveDistance( double distance, double power, boolean setPowerZero ) {

        mecanumDrive.drive( power, 0, 0 );

        int ticksToTravel = mecanumDrive.convertDistTicks(distance);
        int initialXPos = tracker.getLongitudinalPosition();
        int initialYPos = tracker.getLateralPosition();

        while( tracker.getLongitudinalPosition() - initialXPos < ticksToTravel && opModeIsActive()) {
            mecanumDrive.drive( power, 0, 0 );
            /*
            if( tracker.getLateralPosition() < initialYPos && opModeIsActive() ) {
                strafeDistance( mecanumDrive.convertTicksDist(distance), power, false )
            if( tracker.getLateralPosition() > initialYPos && opModeIsActive() ) {
                strafeDistance( mecanumDrive.convertTicksDist(distance), -power, false )
            }*/
        }

        //sets all power to zero afterwords
        if(setPowerZero) {
            mecanumDrive.drive( 0, 0, 0 );
        }


    }

    public void strafeDistance( double distance, double power, boolean setPowerZero ) {

        mecanumDrive.drive( 0, power, 0 );

        int ticksToTravel = mecanumDrive.convertDistTicks(distance);
        int initialPosition = tracker.getLateralPosition();

        while( tracker.getLateralPosition() - ticksToTravel < ticksToTravel && opModeIsActive()) {
            mecanumDrive.drive( 0, power, 0 );
        }

        //sets all power to zero afterwords
        if(setPowerZero) {
            mecanumDrive.drive( 0, 0, 0 );
        }


    }

    /*
    public void rotateDegrees( double degrees, double power, boolean setPowerZero ) {

        mecanumDrive.drive( 0, 0, power );

        double initialDegrees = tracker.getGyroDegrees();

        while( tracker.getGyroDegrees() - initialDegrees < degrees && opModeIsActive()) {
            mecanumDrive.drive( 0, 0, power );
        }

        //sets all power to zero afterwords
        if(setPowerZero) {
            mecanumDrive.drive( 0, 0, 0 );
        }


    }
*/

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
     * @param time - time to rotate the robot in milliseconds
     * @param power - power for the wheels to rotate the robot
     * @param setPowerZero - (boolean) set power to zero after rotating
     */
    public void turnTime( long time, double power, boolean setPowerZero ) {

        mecanumDrive.drive( 0, 0, power );

        //wait for certain amount of time while motors are running
        //robotMecanum.wait(time);
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
