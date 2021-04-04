package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.Robot;

public class Audio {

    private boolean audioFound = false;
    private boolean isPlaying = false;

    private int audioID = 0;

    private String audioName = "";

    private HardwareMap hardwareMap;

    private Thread playAudio;

    public Audio(HardwareMap hw) {

        hardwareMap = hw;
    }

    public Audio(String name, HardwareMap hw) {

        audioName = name;

        hardwareMap = hw;

        initSound();
    }

    public void initSound( ) {

        // Determine Resource IDs for sounds built into the RC application.
        audioID = hardwareMap.appContext.getResources().getIdentifier( audioName, "raw", hardwareMap.appContext.getPackageName() );

        Robot.writeToDefaultFile( "" + audioID, true, true );

        // Determine if sound resources are found.
        // Note: Preloading is NOT required, but it's a good way to verify all your sounds are available before you run.
        if (audioID != 0)
            audioFound = SoundPlayer.getInstance().preload( hardwareMap.appContext, audioID );

        playAudio = new Thread(() -> {
            SoundPlayer.getInstance().startPlaying( hardwareMap.appContext, audioID );
        });
    }

    public void play() {
        String textToWrite = (audioFound ? "Successfully played" : "Failed to find & play") + " audio " + audioName;
        Robot.writeToDefaultFile( textToWrite, true, true);

        //if(audioFound)
            //SoundPlayer.getInstance().startPlaying( hardwareMap.appContext, audioID );
        if(playAudio.isAlive())
            playAudio.interrupt();
        else
            playAudio.start();

        // playAudio.isAlive()) ? playAudio.interrupt() : playAudio.start();
    }

    public String getName() {
        return audioName;
    }

    public int getID() {
        return audioID;
    }

    public boolean exists() {
        return audioFound;
    }




}
