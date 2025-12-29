package org.firstinspires.ftc.teamcode.drive.PoseTransfer;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {
    public static Pose2d endPose;
    public static SIDE side;

    private static final double RED_GOAL_X = 72;
    private static final double RED_GOAL_Y = 72;
    private static final double BLUE_GOAL_X = 72;
    private static final double BLUE_GOAL_Y = -72;

    private static boolean red = PoseStorage.side.equals(PoseStorage.SIDE.RED);

    public static double goalX = red ? RED_GOAL_X : BLUE_GOAL_X;
    public static double goalY = red ? RED_GOAL_Y : BLUE_GOAL_Y;
    public enum SIDE {
        RED, BLUE
    }
}
