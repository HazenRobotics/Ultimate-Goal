package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.SoundLibrary;


@TeleOp (name = "AudioTest", group = "Test")
//@Disabled
public class AudioTest extends OpMode {

    GamepadEvents gamepad1;
    SoundLibrary library;

    boolean showAudios = true;

    @Override
    public void init() {

        gamepad1 = new GamepadEvents(super.gamepad1);
        library = new SoundLibrary( hardwareMap );

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {

        if( gamepad1.a.onPress() )
            telemetry.addLine( library.playAudio("pewdefault") );

        if( gamepad1.b.onPress() )
            telemetry.addLine( library.playRandomPew() );

        if( gamepad1.x.onPress() )
            telemetry.addLine( library.playRandomSound() );

        if( gamepad1.y.onPress() )
            telemetry.addLine( library.playAudio("psstartup") );

        if( gamepad1.right_bumper.onPress() )
            showAudios = !showAudios;

        if( showAudios )
            telemetry.addLine( library.getAudios() );

        telemetry.update();
        gamepad1.update();
    }
}
