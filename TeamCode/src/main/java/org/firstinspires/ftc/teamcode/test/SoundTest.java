package org.firstinspires.ftc.teamcode.test;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.R;

/**
 * Created by robotics on 2/10/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Sounds", group = "FunTest")
public class SoundTest extends OpMode {

    MediaPlayer player;
    MediaPlayer.OnPreparedListener listener;

    @Override
    public void init() {
        player = MediaPlayer.create(hardwareMap.appContext, R.raw.airhorn);
    }

    @Override
    public void loop() {
        if(!player.isPlaying()) {
            player.start();
        }
    }
}
