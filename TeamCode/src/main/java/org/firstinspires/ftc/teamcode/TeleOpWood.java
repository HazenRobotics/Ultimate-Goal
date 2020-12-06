package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;

// TeleOp class for the new wooden robot
@TeleOp(name="teleopWood", group="teleop")
public class TeleOpWood extends OpMode {

    //Robot robot;
    RobotWood robotWood;
    MecanumDrive mecanumDrive;

    @Override
    public void init() {
        //robot = new RobotWood(hardwareMap, this);
        //robot.driveTrain = new MecanumDrive(hardwareMap);
        robotWood = new RobotWood(hardwareMap, this);
        robotWood.driveTrain = new MecanumDrive(hardwareMap);
        mecanumDrive = ((MecanumDrive) robotWood.driveTrain);

        telemetry.addLine("init finished" );

        telemetry.update();
    }

    @Override
    public void loop() {

        telemetry.addData("              Controls", "   ");
        telemetry.addData("Drive", "Gp1: left stick y (axis)");
        telemetry.addData("Strafe", "Gp1: left stick x (axis)");
        telemetry.addData("Rotate", "Gp1: right stick x (axis)");
        telemetry.addLine();

        // moves the robot • left stick; moves forwards/backwards (y axis), strafing left/right (x axis) • right stick; rotating left/right ()x axis)
        mecanumDrive.drive( -gamepad1.left_stick_y/2, gamepad1.left_stick_x/2, gamepad1.right_stick_x/2 );

        telemetry.addData("left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("left_stick_x", gamepad1.left_stick_x);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);

        telemetry.addLine("--------------");

        telemetry.addData("Front Left Power", mecanumDrive.getFrontLeftPower() );
        telemetry.addData("Front Right Power", mecanumDrive.getFrontRightPower() );
        telemetry.addData("Back Left Power", mecanumDrive.getBackLeftPower() );
        telemetry.addData("Back Right Power", mecanumDrive.getBackRightPower() );

        telemetry.addLine();

        telemetry.addLine("longitudinal position = " + robotWood.tracker.getLongitudinalPosition() );
        telemetry.addLine("lateral position = " + robotWood.tracker.getLateralPosition() );

        telemetry.addLine();


        telemetry.addData("Move 17 inches", mecanumDrive.convertDistTicks( 6.5) );

        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

        //telemetry.addData("Gyro", gyro.getPosition() );

        telemetry.update();

    }
}
