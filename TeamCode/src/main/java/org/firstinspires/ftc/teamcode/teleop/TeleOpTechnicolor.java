package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;

@TeleOp (name = "Technicolor", group = "Competition")
public class TeleOpTechnicolor extends OpMode {

    RobotTechnicolorRR robot;

    final double LIFT_POWER = 0.5;
    final double SHOOTER_POWER = 0.85;
    final double INTAKE_POWER = 0.9;

    @Override
    public void init() {
        robot = new RobotTechnicolorRR(hardwareMap, this);
    }

    @Override
    public void loop() {
        robot.drive.setWeightedDrivePower(new Pose2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        ));
        /*if(gamepad1.a) {
            robot.setClawPosition(robot.goalLift.getCurrentClawPosition() == GoalLift.ClawPosition.OPEN ? GoalLift.ClawPosition.CLOSED : GoalLift.ClawPosition.OPEN);
        }
        if(gamepad1.x) {
            robot.setLiftPosition(robot.goalLift.getCurrentLiftPosition() == GoalLift.LiftPosition.LIFTED ? GoalLift.LiftPosition.LOWERED : GoalLift.LiftPosition.LIFTED, 0.5);
        }*/
        /*if(gamepad1.y) {
            robot.ringShooter.launchRingPower(1.0);
        }*/
        // goal lift = gamepad1.y and gamepad1.a
        if( gamepad1.y )
            robot.goalLift.setGoalLiftPosition( GoalLift.LiftPosition.LIFTED, LIFT_POWER, 1000 );
        if( gamepad1.a )
            robot.goalLift.setGoalLiftPosition( GoalLift.LiftPosition.LOWERED, LIFT_POWER, 1000 );

        // ring shooter = gamepad1.right_trigger

        robot.ringShooter.setFlyWheelMotorPower( gamepad1.right_trigger*SHOOTER_POWER );

        // ring pusher (servo) = gamepad1.left_bumper
        if( gamepad1.left_bumper )
            robot.ringShooter.pushRing();

        // intake = gamepad1.left_trigger
        robot.ringShooter.setIntakeMotorPower( gamepad1.left_trigger*INTAKE_POWER );

        robot.drive.update();
        telemetry.update();
    }
}
