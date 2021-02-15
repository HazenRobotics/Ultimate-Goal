package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.robots.ShooterBot;
import org.firstinspires.ftc.teamcode.robots.ShooterBotRR;

@TeleOp (name = "Shooter Bot")
public class TeleOpShooter extends OpMode {

    ShooterBotRR robot;

    @Override
    public void init() {
        robot = new ShooterBotRR(hardwareMap);
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
        if(gamepad1.y) {
            robot.ringShooter.launchRingPower(0.8);
        }
        if(gamepad1.b) {
            robot.setIntakePower(robot.ringShooter.getCurrentIntakePower() == 0 ? 1 : 0);
        }
        /*if(gamepad1.y) {
            robot.ringShooter.setFlyWheelMotorPower(1);
        }
        if(gamepad1.x) {
            robot.ringShooter.setFlyWheelMotorPower(0);
        }*/
        robot.drive.update();
    }
}
