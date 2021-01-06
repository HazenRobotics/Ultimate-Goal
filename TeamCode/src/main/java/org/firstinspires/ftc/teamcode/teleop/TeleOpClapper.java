package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.robots.RobotClapper;

// TeleOp class for the old clapper robot
@TeleOp(name="teleopClapper", group="teleop")
public class TeleOpClapper extends OpMode {

    Robot robot;

    @Override
    public void init() {
        robot = new RobotClapper(hardwareMap, this);
        robot.driveTrain = new MecanumDrive(hardwareMap);
    }

    @Override
    public void loop() {

        telemetry.addData("              Controls", "   ");
        telemetry.addData("Drive", "Gp1: left stick y (axis)");
        telemetry.addData("Strafe", "Gp1: left stick x (axis)");
        telemetry.addData("Rotate", "Gp1: right stick x (axis)");
        telemetry.addLine();

        // moves the robot • left stick; moves forwards/backwards (y axis), strafing left/right (x axis) • right stick; rotating left/right ()x axis)
        ( (MecanumDrive)robot.driveTrain).drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );

        telemetry.addData("left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("left_stick_x", gamepad1.left_stick_x);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);

        telemetry.update();

    }
}
