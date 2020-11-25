package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotClapper;
import org.firstinspires.ftc.teamcode.RobotWood;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;

// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor

@Autonomous(name="AutonomousTest", group="autonomous")
//@disabled
public class AutonomousTest extends LinearOpMode
{
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new RobotWood(hardwareMap, this);
        robot.driveTrain = new MecanumDrive(hardwareMap);

        telemetry.addLine("init finished");
        telemetry.update();

        waitForStart();

        //==========================================================================================
        //Official Start








    }


}
