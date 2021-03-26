package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.road_runner.util.Encoder;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;

@TeleOp(name="Shooter Encoder Test", group="Test")
public class ShooterEncoderTest extends OpMode {
    RobotTechnicolorRR robot;
    @Override
    public void init() {
        robot = new RobotTechnicolorRR(hardwareMap, this);
        robot.ringShooter.leftFlyWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ringShooter.rightFlyWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ringShooter.leftFlyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ringShooter.rightFlyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ringShooter.setFlyWheelPID(new PIDFCoefficients(2, 0, 2, 8.5));
    }

    @Override
    public void loop() {
        robot.ringShooter.setFlyWheelMotorVelocity(5, AngleUnit.RADIANS);
        telemetry.addData("Left Flywheel Position", robot.ringShooter.leftFlyWheelMotor.getCurrentPosition());
        telemetry.addData("Left Flywheel Velocity", robot.ringShooter.leftFlyWheelMotor.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("Right Flywheel Position", robot.ringShooter.rightFlyWheelMotor.getCurrentPosition());
        telemetry.addData("Right Flywheel Velocity", robot.ringShooter.rightFlyWheelMotor.getVelocity(AngleUnit.RADIANS));
        telemetry.update();
    }
}
