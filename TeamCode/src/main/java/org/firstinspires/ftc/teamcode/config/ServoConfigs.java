package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.teamcode.actuators.ServoFTCConfig;

public class ServoConfigs {
    public static ServoFTCConfig CodeBot(String name) {
        ServoFTCConfig config = null;

        switch (name) {
            case "BOOPER-LEFT":
                config = new ServoFTCConfig("booper-left", false, 0.0, 0.75);
                break;
            case "BOOPER-RIGHT":
                config = new ServoFTCConfig("booper-right", true, 0.0, 0.75);
                break;
            case "BLOCKER":
                config = new ServoFTCConfig("blocker", true, 0.0, 0.98);
                break;
        }

        return config;
    }

    public static ServoFTCConfig FinalBot(String name) {
        ServoFTCConfig config = null;

        switch (name) {
            case "BOOPER-LEFT":
                config = new ServoFTCConfig("booper-left", false, 0.0, 0.75);
                break;
            case "BOOPER-RIGHT":
                config = new ServoFTCConfig("booper-right", true, 0.0, 0.75);
                break;
            case "BLOCKER":
                config = new ServoFTCConfig("blocker", true, 0.0, 0.98);
                break;
        }

        return config;
    }
}
