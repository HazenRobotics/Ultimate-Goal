package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "AutoDemonstration", group = "autonomous")
public class AutoDemonstration extends LinearOpMode {

    public static String CLASS_NAME = "AutoDemonstration";

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE );
        backRight.setDirection(DcMotorSimple.Direction.REVERSE );

        telemetry.addLine( CLASS_NAME + " : finished init" );
        telemetry.update();

        waitForStart();

        double globalPower = 0.5;

        setPowers( globalPower, globalPower, globalPower, globalPower );

        long startTime = System.currentTimeMillis();
        while( startTime + 2*1000 > System.currentTimeMillis() );

        rotateClockwise(globalPower, globalPower, globalPower, globalPower );
        while( startTime + 4.5*1000 > System.currentTimeMillis() );
        strafeRight(globalPower, globalPower, globalPower, globalPower );
        while(startTime + 7.5*1000 > System.currentTimeMillis())

        /*setPowers( -globalPower, -globalPower, -globalPower, -globalPower );

        while( startTime + 4*1000 > System.currentTimeMillis() );

        rotateCounterClockwise(globalPower,globalPower,globalPower,globalPower);

        while( startTime + 4.56*1000 > System.currentTimeMillis() );

        strafeRight(globalPower,globalPower,globalPower,globalPower);

        while( startTime + 6.56*1000 > System.currentTimeMillis() );

        strafeLeft(globalPower,globalPower,globalPower,globalPower);

        while( startTime + 8.56*1000 > System.currentTimeMillis() );

        rotateClockwise(globalPower,globalPower,globalPower,globalPower);

        while( startTime + 9.125*1000 > System.currentTimeMillis() );*/

        globalPower = 0;
        setPowers( globalPower, globalPower, globalPower, globalPower );

    }

    public void rotateCounterClockwise( double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower ) {
        frontLeft.setPower( frontLeftPower );
        frontRight.setPower( frontRightPower );
        backLeft.setPower( -backLeftPower );
        backRight.setPower( -backRightPower );

        telemetry.addLine( "FL: " + frontLeftPower + ", FR: " + frontRightPower );
        telemetry.update();
    }

    public void rotateClockwise( double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower ) {
        frontLeft.setPower( -frontLeftPower );
        frontRight.setPower( -frontRightPower );
        backLeft.setPower( backLeftPower );
        backRight.setPower( backRightPower );

        telemetry.addLine( "FL: " + frontLeftPower + ", FR: " + frontRightPower );
        telemetry.update();
    }

    public void strafeLeft( double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower ) {
        frontLeft.setPower( -frontLeftPower );
        frontRight.setPower( frontRightPower );
        backLeft.setPower( backLeftPower );
        backRight.setPower( -backRightPower );

        telemetry.addLine( "FL: " + frontLeftPower + ", FR: " + frontRightPower );
        telemetry.update();
    }

    public void strafeRight( double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower ) {
        frontLeft.setPower( frontLeftPower );
        frontRight.setPower( -frontRightPower );
        backLeft.setPower( -backLeftPower );
        backRight.setPower( backRightPower );

        telemetry.addLine( "FL: " + frontLeftPower + ", FR: " + frontRightPower );
        telemetry.update();
    }

    public void setPowers( double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower ) {

        frontLeft.setPower( frontLeftPower );
        frontRight.setPower( frontRightPower );
        backLeft.setPower( backLeftPower );
        backRight.setPower( backRightPower );

        telemetry.addLine( "FL: " + frontLeftPower + ", FR: " + frontRightPower );
        telemetry.update();

    }

}
