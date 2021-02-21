package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.RobotWood;

import java.text.DecimalFormat;

// TeleOp class for the new wooden robot
@TeleOp(name="TeleOpPIDTesting", group="teleop")
public class TeleOpPIDTesting extends OpMode {

    //Robot robot;
    RobotWood robot;

    double P = 0.06;
    double I = 0.0;
    double D = 0.0;
    double iLimit = 1;

    double target = 0;
    double integral = 0;
    double previousTime = 0;
    double previousError = 0;
    double initialPos;

    DecimalFormat df = new DecimalFormat( "##0.000000" );

    @Override
    public void init() {
        robot = new RobotWood(hardwareMap, this);

        integral = 0;
        previousError = 0;
        previousTime = System.currentTimeMillis();
        initialPos = robot.mecanumDrive.convertTicksDist( robot.tracker.getLateralPosition() );

        telemetry.addLine("init finished");
        telemetry.update();
    }



    @Override
    public void loop() {

        robot.mecanumDrive.drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );

        // get sensor position
        double curInput = robot.mecanumDrive.convertTicksDist( robot.tracker.getLateralPosition() );


        double PChange = 0.001;
        double IChange = 0.0001;
        double DChange = 0.0001;

        if( gamepad1.b ) {
            if( (gamepad1.left_trigger + gamepad1.right_trigger) / 2 > 0.1 )
                P += PChange;
            if( gamepad1.left_bumper )
                I += IChange;
            if( gamepad1.right_bumper )
                D += DChange;
        } else if( gamepad1.x ) {
            if( (gamepad1.left_trigger + gamepad1.right_trigger) / 2 > 0.1 )
                P -= PChange;
            if( gamepad1.left_bumper )
                I -= IChange;
            if( gamepad1.right_bumper )
                D -= DChange;
        }

        telemetry.addLine( "left trigger :: " + gamepad1.left_trigger );


        int targetDistance = 24;

        // get joystick command
        if ( gamepad1.y )
            target = initialPos + targetDistance;
        else if ( gamepad1.a )
            target = initialPos;


        // calculations
        double error = target - curInput;
        double time = System.currentTimeMillis() - previousTime;

        if (Math.abs(error) < iLimit)
            integral += error * time;

        double derivative = (error - previousError) / time;

        double outputSpeed = P * error + I * integral + D * derivative;

        // output to motors
        robot.drive( outputSpeed, 0, 0 );

        // update last- variables
        previousTime = System.currentTimeMillis();
        previousError = error;


        // telemetry
        telemetry.addLine( "----------------------" );
        telemetry.addLine( "P :: " + df.format(P) );
        telemetry.addLine( "I :: " + df.format(I) );
        telemetry.addLine( "D :: " + df.format(D) );
        telemetry.addLine( "----------------------" );
        telemetry.addLine( "P * error :: " + df.format(P * error) );
        telemetry.addLine( "I * integral :: " + df.format(I * integral) );
        telemetry.addLine( "D * derivative :: " + df.format(D * derivative) );
        telemetry.addLine( "----------------------" );
        telemetry.addLine( "outputSpeed :: " + df.format(outputSpeed) );

        telemetry.update();

    }
}
