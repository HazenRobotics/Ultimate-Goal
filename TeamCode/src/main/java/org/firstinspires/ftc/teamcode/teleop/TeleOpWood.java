package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.RobotWood;
import org.firstinspires.ftc.teamcode.mechanisms.GoalLiftWood;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Tracking;

// TeleOp class for the new wooden robot
@TeleOp(name="teleopWood", group="teleop")
public class TeleOpWood extends OpMode {

    //Robot robot;
    RobotWood robot;

    @Override
    public void init() {
        robot = new RobotWood(hardwareMap, this);

        //robot.setClawPosition( GoalLiftWood.ClawPosition.OPEN );

        telemetry.addLine("init finished");
        telemetry.update();
    }

    @Override
    public void loop() {

        telemetry.addData("              Controls", "   ")
                .addData("Drive", "Gp1: left stick y (axis)")
                .addData("Strafe", "Gp1: left stick x (axis)")
                .addData("Rotate", "Gp1: right stick x (axis)");
        telemetry.addLine();

        double pC = ( gamepad1.left_stick_button ? 1.0 : 0.5 ); //powerChange
        robot.mecanumDrive.drive( -gamepad1.left_stick_y*pC, gamepad1.left_stick_x*pC, gamepad1.right_stick_x*pC );

        if(gamepad1.b)
            robot.setClawPosition( GoalLiftWood.ClawPosition.CLOSED );
        if(gamepad1.x)
            robot.setClawPosition( GoalLiftWood.ClawPosition.OPEN);

        double maxLiftPower = 0.5;
        robot.setLiftPower( gamepad1.a ? maxLiftPower : ( gamepad1.y ? -maxLiftPower : 0 ) );

        telemetry.addData("left_stick_y", gamepad1.left_stick_y)
                .addData("left_stick_x", gamepad1.left_stick_x)
                .addData("right_stick_x", gamepad1.right_stick_x);

        telemetry.addLine();

        telemetry.addLine("longitudinal position = " + robot.tracker.getLongitudinalPosition() + " (ticks), "
                + robot.mecanumDrive.convertTicksDist( robot.tracker.getLongitudinalPosition()) + " (in)" );
        telemetry.addLine("lateral position = " + robot.tracker.getLateralPosition() + " (ticks), "
                + robot.mecanumDrive.convertTicksDist( robot.tracker.getLateralPosition()) + " (in)" );

        telemetry.addLine();

        telemetry.addLine( "Servo Position = " + robot.goalLift.claw.getPosition() + " :: "
                + robot.goalLift.getCurrentClawPosition() + " " + robot.goalLift.getCurrentClawPosition().name());
        telemetry.addLine( "Lift Position = " /*+ robot.goalLift.getCurrentLiftPosition()*/ );

        telemetry.addLine();

        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

        /*
        telemetry.addData("getNewGyroHeading", robot.tracker.getNewGyroHeading() )
                .addData("getGyroHeading", robot.tracker.getGyroHeading() );
        telemetry.addData("getGyroRoll", robot.tracker.getGyroRoll() )
                .addData("getGyroPitch", robot.tracker.getGyroPitch() );

        telemetry.addData("getGyroXVelocity", robot.tracker.getGyroXVelocity())
                .addData("getGyroYVelocity", robot.tracker.getGyroYVelocity() )
                .addData("getGyroZVelocity", robot.tracker.getGyroZVelocity() );
         */

        telemetry.update();

    }
}
