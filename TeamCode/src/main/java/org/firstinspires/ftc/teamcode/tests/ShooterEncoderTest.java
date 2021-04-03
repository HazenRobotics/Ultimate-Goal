package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.road_runner.util.Encoder;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;

@TeleOp(name="Shooter Encoder Test", group="Test")
//@Disabled
public class ShooterEncoderTest extends LinearOpMode {
    RobotTechnicolorRR robot;
    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotTechnicolorRR(hardwareMap, this);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        robot.ringShooter.leftFlyWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ringShooter.rightFlyWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ringShooter.leftFlyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ringShooter.rightFlyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ringShooter.setFlyWheelPID(new PIDFCoefficients(6, 0, 3, 12 * 12 / batteryVoltageSensor.getVoltage()));

        waitForStart();
        robot.ringShooter.setFlyWheelMotorVelocity(3, AngleUnit.RADIANS);
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Left Flywheel Position", robot.ringShooter.leftFlyWheelMotor.getCurrentPosition());
            telemetry.addData("Left Flywheel Velocity", robot.ringShooter.leftFlyWheelMotor.getVelocity(AngleUnit.RADIANS));
            telemetry.addData("Right Flywheel Position", robot.ringShooter.rightFlyWheelMotor.getCurrentPosition());
            telemetry.addData("Right Flywheel Velocity", robot.ringShooter.rightFlyWheelMotor.getVelocity(AngleUnit.RADIANS));
            telemetry.update();
        }
    }
}
