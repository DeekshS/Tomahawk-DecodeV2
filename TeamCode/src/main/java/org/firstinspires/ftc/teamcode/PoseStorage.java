package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {
    public static Pose2d endPose;
    public static SIDE side;
    public static Pose2d blueHP = new Pose2d(-70,-70,0);
    public static Pose2d redHP = new Pose2d(-70,70,0);

    public static double goalX;
    public static double goalY;

    public enum SIDE {
        RED, BLUE
    }
}
