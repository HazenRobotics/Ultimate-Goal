
package org.firstinspires.ftc.newteamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot extends LinearOpMode
{
    //======================================================
    DcMotor leftMotor;
    DcMotor rightMotor;

    //======================================================
    DcMotor lift;
    final double MAX_LIFT_SPEED = 0.8;

    //======================================================
    Servo leftHook;
    Servo rightHook;
    final double LEFT_HOOK_HOME = 0.75;
    final double RIGHT_HOOK_HOME = 0.2;
    final double LEFT_HOOK_EXTENDED = 0;
    final double RIGHT_HOOK_EXTENDED = 1;
    double leftHookPosition = LEFT_HOOK_HOME;
    double rightHookPosition =  RIGHT_HOOK_HOME;

    //======================================================
    Servo leftClapper;
    Servo rightClapper;
    final double LEFT_CLAPPER_HOME = 0.0;
    final double RIGHT_CLAPPER_HOME = 1.0;
    final double LEFT_CLAPPER_EXTENDED = 0.38;
    final double RIGHT_CLAPPER_EXTENDED = 0.61;
    double leftClapperPosition = LEFT_CLAPPER_HOME;
    double rightClapperPosition = RIGHT_CLAPPER_HOME;

    //======================================================
    GyroSensor gyro;

    //======================================================
    final int tickPerRevlolution = 1440;
    final double linearWheelDistance = (Math.PI) * 4;
    final double linearSpoolDistance = (Math.PI) * 1.5748;

    //======================================================
    enum Position{none,left,right};
    Position skystonePosition;

    int noneTally;
    int leftTally;
    int rightTally;
    int totalTally;

    final int IMAGE_CHECK_ITERATIONS = 2;

    //======================================================
    int shuffleCount = 0;

    //==============================================================================================

    @Override
    public void runOpMode() throws InterruptedException { }

    //=========================================================================================
    //Lift method
    public void setlift(double liftPower)
    {
        telemetry.addData("setLift", "running");
        telemetry.update();

        convertDistTicks(5.5, linearSpoolDistance);
    }

    //==========================================================================================
    //clapper method
    public void clapper(boolean clappersHome)
    {
        telemetry.addData("clappers", "running");
        telemetry.update();

        //set clappers position to their positions
        if(clappersHome)
        {
            leftClapperPosition = LEFT_CLAPPER_HOME;
            rightClapperPosition = RIGHT_CLAPPER_HOME;
        }
        else
        {
            leftClapperPosition = LEFT_CLAPPER_EXTENDED;
            rightClapperPosition = RIGHT_CLAPPER_EXTENDED;
        }
        leftClapper.setPosition(leftClapperPosition);
        rightClapper.setPosition(rightClapperPosition);
    }


    //==========================================================================================
    //hook methd
    public void hooks(boolean hooksHome)
    {
        telemetry.addData("hooks", "running");
        telemetry.update();

        //set hooks positions to positions
        if(hooksHome)
        {
            leftHookPosition = LEFT_HOOK_HOME;
            rightHookPosition = RIGHT_HOOK_HOME;
        }
        else
        {
            leftHookPosition = LEFT_HOOK_EXTENDED;
            rightHookPosition = RIGHT_HOOK_EXTENDED;
        }
        leftHook.setPosition(leftHookPosition);
        rightHook.setPosition(rightHookPosition);
    }

    //==============================================================================================

    //method takes in 2nd parameter for circumfrence of spinning object
    public int convertDistTicks(double distanceToTravel, double circumfrence)
    {
        //1440 revolutions = 1 rotation
        //1 rotation = 4

        double revolutions = distanceToTravel / circumfrence;
        int totalTicks = (int) Math.round(revolutions * tickPerRevlolution);

        return totalTicks;
    }

    public void move(double distanceToTravel,double power, boolean isForward)
    {
        telemetry.addData("move", "running");
        telemetry.update();

        // reset encoder count kept by left motor.
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (isForward)
        {
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
            leftMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        else
        {
            rightMotor.setDirection(DcMotor.Direction.FORWARD);
            leftMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        // set left motor to run to 5000 encoder counts.
        leftMotor.setTargetPosition(convertDistTicks(distanceToTravel, linearWheelDistance));
        rightMotor.setTargetPosition(convertDistTicks(distanceToTravel, linearWheelDistance));

        // set both motors to 25% power. Movement will start.
        leftMotor.setPower(power);
        rightMotor.setPower(power);

        // set left motor to run to target encoder position and stop with brakes on.
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // wait while opmode is active and left motor is busy running to position.
        while (opModeIsActive() && leftMotor.isBusy())
        {
            telemetry.addData("encoder-fwd", leftMotor.getCurrentPosition() + "  busy=" + leftMotor.isBusy());
            telemetry.update();
            idle();
        }
        while (opModeIsActive() && rightMotor.isBusy())
        {
            telemetry.addData("encoder-fwd", rightMotor.getCurrentPosition() + "  busy=" + rightMotor.isBusy());
            telemetry.update();
            idle();
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }



    public void turnOnSpot(double turningDegrees, double power, boolean turnLeft)
    {
        telemetry.addData("turnOnSpot", "running");
        telemetry.update();

        double turningNumber = (turningDegrees/180) * 16.4 * (Math.PI);
        double onSpotTurningNumber = turningNumber/2;

        // reset encoder count kept by left motor.
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //forward direction switching both wheels for turning in spot
        if(turnLeft)
        {
            rightMotor.setDirection(DcMotor.Direction.FORWARD);
            leftMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        else
        {
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
            leftMotor.setDirection(DcMotor.Direction.REVERSE);
        }
    }


    //turning method
    public void turn(double turningDegrees, double power, boolean isForward, boolean leftWheel)
    {
        telemetry.addData("turn", "running");
        telemetry.update();

        // calculations from degrees to motor distance

        // 90* arc length = (radius/2) * pi
        // angle/180 * radius * pi
        // (angle/180) * 16.5 * (Math.PI)
        // (turningDegrees/180) * 16.5 * (Math.PI)

        double turningNumber = (turningDegrees/180) * 16.4 * (Math.PI);
        double onSpotTurningNumber = turningNumber/2;

        if (isForward)
        {
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
            leftMotor.setDirection(DcMotor.Direction.FORWARD);

        }
        else
        {
            rightMotor.setDirection(DcMotor.Direction.FORWARD);
            leftMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        //if wheel is left:
        if(leftWheel)
        {
            // reset encoder count kept by left motor.
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set left motor to run to 5000 encoder counts.
            leftMotor.setTargetPosition(convertDistTicks(turningNumber,linearWheelDistance));

            // set both motors to 25% power. Movement will start.
            leftMotor.setPower(power);

            // set left motor to run to target encoder position and stop with brakes on.
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // wait while opmode is active and left motor is busy running to position.
            while (opModeIsActive() && leftMotor.isBusy()) {
                telemetry.addData("encoder-fwd", leftMotor.getCurrentPosition() + "  busy=" + leftMotor.isBusy());
                telemetry.update();
                idle();
            }

            // set motor power to zero to turn off motors. The motors stop on their own but
            // power is still applied so we turn off the power.
            leftMotor.setPower(0.0);
        }

        //if wheel is right:
        else
        {
            // reset encoder count kept by left motor.
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set left motor to run to 5000 encoder counts.
            rightMotor.setTargetPosition(convertDistTicks(turningNumber,linearWheelDistance));

            // set both motors to 25% power. Movement will start.
            rightMotor.setPower(power);

            // set left motor to run to target encoder position and stop with brakes on.
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // wait while opmode is active and left motor is busy running to position.
            while (opModeIsActive() && rightMotor.isBusy())
            {
                telemetry.addData("encoder-fwd", rightMotor.getCurrentPosition() + "  busy=" + rightMotor.isBusy());
                telemetry.update();
                idle();
            }

            // set motor power to zero to turn off motors. The motors stop on their own but
            // power is still applied so we turn off the power.
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);
        }
    }

    public void turnGyro(double turningDegrees, double power, boolean turnRight)
    {
        telemetry.addData("turnGyro", "running");
        telemetry.update();

        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gyro.resetZAxisIntegrator();
        if(turnRight)
        {
            rightMotor.setDirection(DcMotor.Direction.FORWARD);
            leftMotor.setDirection(DcMotor.Direction.FORWARD);

            rightMotor.setPower(power);
            leftMotor.setPower(power);

            while(gyro.getHeading() + 180 < 180 - turningDegrees) {}
        }
        else
        {
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
            leftMotor.setDirection(DcMotor.Direction.REVERSE);

            rightMotor.setPower(power);
            leftMotor.setPower(power);

            while(gyro.getHeading() + 180 < 180 + turningDegrees) {}
        }

        rightMotor.setPower(0);
        leftMotor.setPower(0);

        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

