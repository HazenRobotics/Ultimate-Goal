package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.GoalLiftWood;
import org.firstinspires.ftc.teamcode.robots.RobotWood;

// TeleOp class for the new wooden robot
@TeleOp(name="teleopWood", group="teleop")
public class TeleOpWood extends OpMode {

    //Robot robot;
    RobotWood robot;

    @Override
    public void init() {
        robot = new RobotWood(hardwareMap, this);
    }

    @Override
    public void loop() {


        telemetry.addData("              Controls", "   ")
                .addData("Drive", "Gp1: left stick y (axis)")
                .addData("Strafe", "Gp1: left stick x (axis)")
                .addData("Rotate", "Gp1: right stick x (axis)");
        telemetry.addLine();

        // moves the robot • left stick; moves forwards/backwards (y axis), strafing left/right (x axis) • right stick; rotating left/right ()x axis)
        double slow = 1/2;
        robot.mecanumDrive.drive( -gamepad1.left_stick_y*slow, gamepad1.left_stick_x*slow, gamepad1.right_stick_x*slow );

        double maxDrivePower = 1.0;
        robot.mecanumDrive.setMotorPower( gamepad1.left_trigger, gamepad1.right_trigger, (gamepad1.left_bumper) ? maxDrivePower : 0, (gamepad1.right_bumper) ? maxDrivePower : 0 );


        if(gamepad1.a)
            robot.setClawPosition(robot.goalLift.getCurrentClawPosition() == GoalLiftWood.ClawPosition.OPEN ? GoalLiftWood.ClawPosition.CLOSED : GoalLiftWood.ClawPosition.OPEN);

        double maxLiftPower = 0.5;
        robot.setLiftPower( gamepad1.x ? maxLiftPower : 0 );



        telemetry.addData("left_stick_y", gamepad1.left_stick_y)
                .addData("left_stick_x", gamepad1.left_stick_x)
                .addData("right_stick_x", gamepad1.right_stick_x);

        telemetry.addLine("--------------");

        telemetry.addData("Front Left Power", robot.mecanumDrive.getFrontLeftPower() )
                .addData("Front Right Power", robot.mecanumDrive.getFrontRightPower() )
                .addData("Back Left Power", robot.mecanumDrive.getBackLeftPower() )
                .addData("Back Right Power", robot.mecanumDrive.getBackRightPower() );

        telemetry.addLine();

        telemetry.addLine("longitudinal position = " + robot.tracker.getLongitudinalPosition() );
        telemetry.addLine("lateral position = " + robot.tracker.getLateralPosition() );

        telemetry.addLine();

        /*
        for (int travelDistance = 0; travelDistance <= 20; travelDistance++) {
            String data = "Move " + travelDistance + " inch" + (travelDistance == 1 ? "    " : "es") + (travelDistance <= 9 ? "   " : "");
            telemetry.addData(data, robot.mecanumDrive.convertDistTicks(travelDistance) );
        }

        telemetry.addLine();

        telemetry.addData("Move 20 inches", robot.mecanumDrive.convertDistTicks( 20) ); // 4255(.30058379) with 250 ppr, 3404(.24046704) with 200 ppr
        String data = "Move " + robot.mecanumDrive.convertDistTicks( 20) + " ticks";
        telemetry.addData(data, robot.mecanumDrive.convertTicksDist( mecanumDrive.convertDistTicks( 20) )  );
        */

        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

        telemetry.addData("getNewGyroHeading", robot.tracker.getNewGyroHeading())
                .addData("getGyroHeading", robot.tracker.getGyroHeading() )
                .addData("getGyroRoll", robot.tracker.getGyroRoll() )
                .addData("getGyroPitch", robot.tracker.getGyroPitch() );

        telemetry.addData("getGyroXVelocity", robot.tracker.getGyroXVelocity())
                .addData("getGyroYVelocity", robot.tracker.getGyroYVelocity() )
                .addData("getGyroZVelocity", robot.tracker.getGyroZVelocity() );

        telemetry.update();

    }
}
