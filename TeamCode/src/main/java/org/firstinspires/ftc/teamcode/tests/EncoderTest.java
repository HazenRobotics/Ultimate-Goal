package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.road_runner.util.Encoder;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;

@TeleOp(name="Localization Encoder Test", group="Test")
public class EncoderTest extends OpMode {
    RobotTechnicolorRR robot;
    Encoder parallelEncoder;
    Encoder perpendicularEncoder;
    @Override
    public void init() {
        robot = new RobotTechnicolorRR(hardwareMap, this);
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "goalLift"));
    }

    @Override
    public void loop() {
        robot.drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );
        telemetry.addData("Parallel Encoder", parallelEncoder.getCurrentPosition());
        telemetry.addData("Perpendicular Encoder", perpendicularEncoder.getCurrentPosition());
        telemetry.update();

    }
}
