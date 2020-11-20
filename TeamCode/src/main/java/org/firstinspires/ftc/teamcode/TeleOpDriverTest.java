package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOpDriverTest", group="teleop")
public class TeleOpDriverTest extends OpMode
{
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;

    @Override
    public void init()
    {
        //Wheels
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftWheel");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightWheel");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftWheel");
        backRightMotor = hardwareMap.dcMotor.get("backRightWheel");
        //leftMotor.setDirection(DcMotor.Direction.REVERSE);

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
        telemetry.addData("Front Right Position", frontRightMotor.getCurrentPosition() );
        telemetry.addData("Back Left Position", backLeftMotor.getCurrentPosition() );
        telemetry.addData("Back Right Position", backRightMotor.getCurrentPosition() );

        telemetry.update();

    }
}