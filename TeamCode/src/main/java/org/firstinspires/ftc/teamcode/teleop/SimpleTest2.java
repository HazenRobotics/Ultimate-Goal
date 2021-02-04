package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp(name="SimpleTest2", group="SimpleTests") // uncomment this for it to appear in the DS app
public class SimpleTest2 extends OpMode
{
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;

    BNO055IMU gyro;

    @Override
    public void init()
    {
        //Wheels
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        backRightMotor = hardwareMap.dcMotor.get("backRight");

        gyro = hardwareMap.get( BNO055IMU.class, "imu" );
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        gyro.initialize( parameters );

        // make sure the imu gyro is calibrated before continuing.
        while ( !gyro.isGyroCalibrated() )
        {
            long startTime = System.currentTimeMillis();
            while( System.currentTimeMillis() < startTime + 50 /* milliseconds to wait */ );
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", gyro.getCalibrationStatus().toString());
        telemetry.update();
    }

    @Override
    public void loop()
    {

        telemetry.addData("              Controls", "   ");
        telemetry.addData("Drive", "Gp1: left stick y (axis)");
        telemetry.addData("Strafe", "Gp1: left stick x (axis)");
        telemetry.addData("Rotate", "Gp1: right stick x (axis)");
        telemetry.addLine();



        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
        //driving

        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        frontLeftPower = drive + strafe + rotate;
        frontRightPower = -drive + strafe - rotate;
        backLeftPower = drive - strafe - rotate;
        backRightPower = -drive - strafe + rotate;

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);

        telemetry.addData("left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("left_stick_x", gamepad1.left_stick_x);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);

        telemetry.addLine("--------------");

        telemetry.addData("Front Left Power", frontLeftMotor.getPower() );
        telemetry.addData("Front Right Power", frontRightMotor.getPower() );
        telemetry.addData("Back Left Power", backLeftMotor.getPower() );
        telemetry.addData("Back Right Power", backRightMotor.getPower() );

        telemetry.addLine();

        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

        telemetry.addData("Front Left Position", frontLeftMotor.getCurrentPosition() );
        telemetry.addData("Back Left Position", backLeftMotor.getCurrentPosition() );

        telemetry.addLine();

        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

        telemetry.addData("Gyro x", gyro.getPosition().x );
        telemetry.addData("Gyro y", gyro.getPosition().y );
        telemetry.addData("Gyro z", gyro.getPosition().z );
        telemetry.addData("Gyro unit", gyro.getPosition().unit );
        telemetry.addData("Gyro unit", gyro.getPosition().unit.toString() );
        telemetry.addData("Gyro", gyro.readCalibrationData() );






        telemetry.update();

    }
}