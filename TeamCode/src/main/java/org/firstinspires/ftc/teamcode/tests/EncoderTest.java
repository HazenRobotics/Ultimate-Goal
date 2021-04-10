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

import java.util.List;

@TeleOp(name="Localization Encoder Test", group="Test")
public class EncoderTest extends OpMode {
    RobotTechnicolorRR robot;
    Encoder parallelEncoder;
    Encoder perpendicularEncoder;
    AveragedGyro imu;
    @Override
    public void init() {

        Robot.createDefaultMatchLogFileName( this.getClass().getSimpleName() );

        robot = new RobotTechnicolorRR(hardwareMap, this);
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "goalLift"));
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = new AveragedGyro(hardwareMap, "imu", "imu2", params);
    
        Robot.writeToMatchDefaultFile( "Init Finished", true );
    }

    @Override
    public void loop() {

        double rotatePercent = 0.5;

        robot.drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y*1,
                        -gamepad1.left_stick_x*1,
                        -gamepad1.right_stick_x*rotatePercent
                )
        );

        telemetry.addData("Parallel Encoder", parallelEncoder.getCurrentPosition());
        telemetry.addData("Perpendicular Encoder", perpendicularEncoder.getCurrentPosition());
        List<Double> motors = robot.drive.getWheelPositions();
        telemetry.addData("Front Left Encoder", motors.get(0));
        telemetry.addData("Back Left Encoder", motors.get(1));
        telemetry.addData("Back Right Encoder", motors.get(2));
        telemetry.addData("Front Right Encoder", motors.get(3));
        telemetry.addData("Angle", imu.getAngularHeading());
        telemetry.update();

    }
}
