package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;

public class RobotWood extends Robot {

    public RobotWood(HardwareMap hw, OpMode op){
        super(hw, op);
        super.drive = new MecanumDrive(hw);

    }


    /**
     *
     * @param time - time to move the robot
     * @param power - power the wheels to move the robot
     * @param setPowerZero - (boolean) set power to zero after moving
     */
    public void driveTime( long time, double power, boolean setPowerZero ) {

        // troublshoot: drive.driveOmni( power, 0, 0 );
        //drive.driveOmni( power, 0, 0 );

        //wait for certain amount of time while motors are running
        long setTime = System.currentTimeMillis();
        previousTime = opMode.getRuntime();

        while(System.currentTimeMillis() - setTime < (time) && opModeIsActive()) {
            //drive.driveOmni( power, 0, 0 );
        }

        //sets all power to zero afterwords
        if(setPowerZero) {
            //drive.driveOmni( 0, 0, 0 );
        }
    }

    /**
     *
     * @param time - time to strafe the robot
     * @param power - power for the wheels to strafe the robot
     * @param setPowerZero - (boolean) set power to zero after strafing
     */
    public void strafeTime( long time, double power, boolean setPowerZero ) {

        // troublshoot: drive.driveOmni( power, 0, 0 );
        //drive.driveOmni( 0, power, 0 );

        //wait for certain amount of time while motors are running
        long setTime = System.currentTimeMillis();
        previousTime = opMode.getRuntime();

        while(System.currentTimeMillis() - setTime < (time) && opModeIsActive()) {
            //drive.driveOmni( 0, power, 0 );
        }

        //sets all power to zero afterwords
        if(setPowerZero) {
            //drive.driveOmni( 0, 0, 0 );
        }
    }

    /**
     *
     * @param time - time to rotate the robot
     * @param power - power for the wheels to rotate the robot
     * @param setPowerZero - (boolean) set power to zero after rotating
     */
    public void turnTime( long time, double power, boolean setPowerZero ) {

        // troublshoot: drive.driveOmni( power, 0, 0 );
        //drive.driveOmni( 0, 0, power );

        //wait for certain amount of time while motors are running
        //robotMecanum.wait(time);
        long setTime = System.currentTimeMillis();
        previousTime = opMode.getRuntime();

        while(System.currentTimeMillis() - setTime < (time) && opModeIsActive()) {
            //drive.driveOmni( 0, 0, power );
        }

        //sets all power to zero afterwords
        if(setPowerZero) {
            //drive.driveOmni( 0, 0, 0 );
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

        //drive.driveOmni(drivePower, strafePower, 0);

        //wait for certain amount of time while motors are running
        //robotMecanum.wait(time);
        long setTime = System.currentTimeMillis();
        previousTime = opMode.getRuntime();

        while(System.currentTimeMillis() - setTime < (time) && opModeIsActive())
        {
            //drive.driveOmni(drivePower, strafePower, /*gyroPID(180, opMode.getRuntime() - previousTime)*/0);
        }

        //sets all power to zero afterwords
        if(setPowerZero) {
            //drive.driveOmni( 0, 0, 0 );
        }
    }


}
