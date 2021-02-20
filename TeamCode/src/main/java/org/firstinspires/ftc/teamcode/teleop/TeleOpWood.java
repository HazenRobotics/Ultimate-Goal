package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.RobotWood;
import org.firstinspires.ftc.teamcode.mechanisms.GoalLiftWood;

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

        double pC = ( gamepad1.left_stick_button ? 1.0 : 0.4 ); //powerChange
        robot.mecanumDrive.drive( -gamepad1.left_stick_y*pC, gamepad1.left_stick_x*pC, gamepad1.right_stick_x*pC );

        if(gamepad1.b)
            robot.goalLift.setClawPosition( GoalLiftWood.ClawPosition.CLOSED );
        if(gamepad1.x)
            robot.goalLift.setClawPosition( GoalLiftWood.ClawPosition.OPEN);

        double liftPower = 0.5;
        if( gamepad1.y )
            robot.goalLift.setGoalLiftPosition( GoalLiftWood.LiftPosition.LIFTED, liftPower );
        if( gamepad1.a )
            robot.goalLift.setGoalLiftPosition( GoalLiftWood.LiftPosition.LOWERED, liftPower );

        telemetry.addData("left_stick_y", gamepad1.left_stick_y)
                .addData("left_stick_x", gamepad1.left_stick_x)
                .addData("right_stick_x", gamepad1.right_stick_x);

        telemetry.addLine();

        telemetry.addLine("longitudinal position = " + robot.tracker.getLongitudinalPosition() + " (ticks), "
                + robot.mecanumDrive.convertTicksDist( robot.tracker.getLongitudinalPosition()) + " (in)" );
        telemetry.addLine("lateral position = " + robot.tracker.getLateralPosition() + " (ticks), "
                + robot.mecanumDrive.convertTicksDist( robot.tracker.getLateralPosition()) + " (in)" );

        telemetry.addLine();

        telemetry.addLine( "Servo Position = " + robot.goalLift.getCurrentClawPosition() + " :: "  + robot.goalLift.getClawPosition() );
        telemetry.addLine( "Lift Position = " + robot.goalLift.getCurrentLiftPosition() + " :: " + robot.goalLift.getLiftPower() );

        telemetry.addLine();

        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||


        telemetry.addData("getNewGyroHeading", robot.tracker.get360GyroHeading() )
                .addData("getGyroHeading", robot.tracker.getGyroHeading() );

        /*
        telemetry.addData("getGyroRoll", robot.tracker.getGyroRoll() )
                .addData("getGyroPitch", robot.tracker.getGyroPitch() );

        telemetry.addData("getGyroXVelocity", robot.tracker.getGyroXVelocity())
                .addData("getGyroYVelocity", robot.tracker.getGyroYVelocity() )
                .addData("getGyroZVelocity", robot.tracker.getGyroZVelocity() );
         */

        telemetry.update();

    }
}
