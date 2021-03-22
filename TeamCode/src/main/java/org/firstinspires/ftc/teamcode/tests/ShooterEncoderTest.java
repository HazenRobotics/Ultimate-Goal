package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.road_runner.util.Encoder;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;

@TeleOp(name="Shooter Encoder Test", group="Test")
public class ShooterEncoderTest extends OpMode {
    RobotTechnicolorRR robot;
    @Override
    public void init() {
        robot = new RobotTechnicolorRR(hardwareMap, this);
        robot.ringShooter.leftFlyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ringShooter.rightFlyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        robot.ringShooter.setFlyWheelMotorPower(gamepad1.right_trigger);
        telemetry.addData("Left Flywheel Position", robot.ringShooter.leftFlyWheelMotor.getCurrentPosition());
        telemetry.addData("Right Flywheel Position", robot.ringShooter.rightFlyWheelMotor.getCurrentPosition());
        telemetry.update();

    }
}
