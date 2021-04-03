package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.SoundLibrary;


@TeleOp (name = "AudioTest", group = "Test")
public class AudioTest extends LinearOpMode {



    GamepadEvents gamepad1;
    SoundLibrary library;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepad1 = new GamepadEvents(super.gamepad1);
        library = new SoundLibrary( hardwareMap );

        waitForStart();







    }



}
