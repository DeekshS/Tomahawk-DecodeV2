package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {
    public static Pose2d endPose;
    public static SIDE side;

    public static double goalX = 72;
    public static double goalY;

    public static double hpX;
    public static double hpY;
    public static double hpHeading;

    public enum SIDE {
        RED, BLUE
    }
}
