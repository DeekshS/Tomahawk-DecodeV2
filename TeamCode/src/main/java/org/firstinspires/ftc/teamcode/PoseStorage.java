package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {
    public static Pose2d endPose = new Pose2d(0,0,0);
    public static SIDE side;

    public static double goalX = 72;
    public static double goalY;

    public static double hpX;
    public static double hpY;
    public static double hpHeading;
    public static Action setEndPose;

    public enum SIDE {
        RED, BLUE
    }

    public static Action setEndPose(Pose2d pose) {

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                endPose = pose;
                return false;
            }
        };
    }
}
