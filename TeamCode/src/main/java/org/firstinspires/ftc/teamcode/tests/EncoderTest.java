package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.road_runner.util.Encoder;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;
import org.firstinspires.ftc.teamcode.utils.AveragedGyro;

@TeleOp(name="Localization Encoder Test", group="Test")
public class EncoderTest extends OpMode {
    RobotTechnicolorRR robot;
    Encoder parallelEncoder;
    Encoder perpendicularEncoder;
    AveragedGyro imu;
    @Override
    public void init() {

        //Robot.createDefaultMatchLogFileName( this.getClass().getName() );

        robot = new RobotTechnicolorRR(hardwareMap, this);
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "goalLift"));
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = new AveragedGyro(hardwareMap, "imu", "imu2", params);
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
        telemetry.addData("Drive Motor Encoders", robot.drive.getWheelPositions());
        telemetry.addData("Angle", imu.getAngularHeading());
        telemetry.update();

    }
}
