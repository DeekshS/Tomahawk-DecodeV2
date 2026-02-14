package org.firstinspires.ftc.teamcode.autonomous.autos;
//
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//
//
//public interface FCV2 {
//
//
//
//
//    double intake = 0;
//    double shoot = 2;
//
//
//    //-------------------------------Angles------------------------------------------
//
//
//    //RED
//    double RED_CLOSE_ANGLE = Math.toRadians(-45);
//    double RED_FAR_ANGLE = Math.toRadians(-20);
//    double RED_ARTIFACT_ANGLE = Math.toRadians(-90);
//
//
//    //BLUE
//    double BLUE_CLOSE_ANGLE = Math.toRadians(45);
//    double BLUE_FAR_ANGLE = Math.toRadians(20);
//    double BLUE_ARTIFACT_ANGLE = Math.toRadians(90);
//
//
//
//
//    //-------------------------------Coordinates------------------------------------------
//
//
//    // ======Artifact Length======
//    double ARTIFACT_DIST = 29;
//    double HP_ARTIFACT_DIST = 62;
//    // ======RED ARTIFACTS======
//    Vector2d GPP_RED_ARTIFACT = new Vector2d(-36, -17);
//    Vector2d PGP_RED_ARTIFACT = new Vector2d(-12, -24);
//    Vector2d PPG_RED_ARTIFACT = new Vector2d(12, -24);
//    Vector2d HP_RED_ARTIFACT = new Vector2d(-66,-62);
//
//
//
//
//    // ======RED Shooting Locations======
//    Vector2d RED_CLOSE_SHOOT = new Vector2d(12, -12);
//    Vector2d RED_FAR_SHOOT = new Vector2d(-58, -12);
//
//
//    // ======BLUE ARTIFACTS======
//    Vector2d GPP_BLUE_ARTIFACT = new Vector2d(-36, 17);
//    Vector2d PGP_BLUE_ARTIFACT = new Vector2d(-14, 24);
//    Vector2d PPG_BLUE_ARTIFACT = new Vector2d(12, 24);
//    Vector2d HP_BLUE_ARTIFACT = new Vector2d(-66,62);
//
//
//    // ====== BLUE Shooting Locations======
//    Vector2d BLUE_CLOSE_SHOOT = new Vector2d(12, 8);
//    Vector2d BLUE_FAR_SHOOT = new Vector2d(-58, 12);
//
//
//
//
//    // ======Gates======
//    Vector2d BLUE_GATE = new Vector2d(0, 56);
//    Vector2d BLUE_GATE_INTAKE = new Vector2d(-24, 62);
//    Vector2d BLUE_GATE_INTAKE_TWO = new Vector2d(-25, 62);
//    double BLUE_GATE_INTAKE_TWO_ANGLE = Math.toRadians(0);
//    double BLUE_GATE_INTAKE_ANGLE = Math.toRadians(64);
//    Vector2d RED_GATE = new Vector2d(0, -50);
//    Vector2d RED_GATE_INTAKE = new Vector2d(-24, -62);
//    double RED_GATE_INTAKE_ANGLE = Math.toRadians(235);
//
//
//    // ======STARTING POSITIONS======
//    Pose2d RED_FAR_START = new Pose2d(-61.25, -5, Math.toRadians(0));
//
//
//    Pose2d RED_CLOSE_START = new Pose2d(55.5, -43.5, Math.toRadians(-53));
//
//
//    Pose2d BLUE_FAR_START = new Pose2d(-61.25, 24, Math.toRadians(0));
//
//
//    Pose2d BLUE_CLOSE_START = new Pose2d(55.5, 43.5, Math.toRadians(53));
//
//
//    Pose2d BLUE_GOAL_START = new Pose2d(-47.5, 51.5,  Math.toRadians(323));
//
//
//    //======MISC=======
//    Vector2d BLUE_FAR_PARK = new Vector2d(-24, 24);
//
//
//}
//



import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;


public interface FCV2 {




    double intake = 0;
    double shoot = 2;


    //-------------------------------Angles------------------------------------------


    //RED
    double RED_CLOSE_ANGLE = Math.toRadians(-45);
    double RED_FAR_ANGLE = Math.toRadians(-26);
    double RED_ARTIFACT_ANGLE = Math.toRadians(-90);


    //BLUE
    double BLUE_CLOSE_ANGLE = Math.toRadians(45);
    double BLUE_FAR_ANGLE = Math.toRadians(26);
    double BLUE_ARTIFACT_ANGLE = Math.toRadians(90);




    //-------------------------------Coordinates------------------------------------------


    // ======Artifact Length======
    double ARTIFACT_DIST = 33;
    double HP_ARTIFACT_DIST = 62;
    // ======RED ARTIFACTS======
    Vector2d GPP_RED_ARTIFACT = new Vector2d(-42, -18);
    Vector2d PGP_RED_ARTIFACT = new Vector2d(-15, -24);
    Vector2d PPG_RED_ARTIFACT = new Vector2d(10, -24);
    Vector2d HP_RED_ARTIFACT = new Vector2d(-65,-59);




    // ======RED Shooting Locations======
    Vector2d RED_CLOSE_SHOOT = new Vector2d(12, -8);
    Vector2d RED_FAR_SHOOT = new Vector2d(-58, -12);


    // ======BLUE ARTIFACTS======
    Vector2d GPP_BLUE_ARTIFACT = new Vector2d(-26, 18);
    Vector2d PGP_BLUE_ARTIFACT = new Vector2d(-13, 24);
    Vector2d PPG_BLUE_ARTIFACT = new Vector2d(10, 24);
    Vector2d HP_BLUE_ARTIFACT = new Vector2d(-54,71);
    Vector2d HP_SIDE_BLUE_ARTIFACT = new Vector2d(-61,50);



    // ====== BLUE Shooting Locations======
    Vector2d BLUE_CLOSE_SHOOT = new Vector2d(12, 8);
    Vector2d BLUE_FAR_SHOOT = new Vector2d(-55, 17);




    // ======Gates======
    Vector2d BLUE_GATE = new Vector2d(0, 54);
    Vector2d BLUE_GATE_INTAKE = new Vector2d(-24, 62);
    Vector2d BLUE_GATE_INTAKE_TWO = new Vector2d(-25, 62);
    double BLUE_GATE_INTAKE_TWO_ANGLE = Math.toRadians(0);
    double BLUE_GATE_INTAKE_ANGLE = Math.toRadians(64);
    Vector2d RED_GATE = new Vector2d(0, -54);
    Vector2d RED_GATE_INTAKE = new Vector2d(-24, -62);
    double RED_GATE_INTAKE_ANGLE = Math.toRadians(235);


    // ======STARTING POSITIONS======
    Pose2d RED_FAR_START = new Pose2d(-61.25, -24, Math.toRadians(0));


    //    Pose2d RED_CLOSE_START = new Pose2d(55.5, -43.5, Math.toRadians(-47));
    Pose2d RED_CLOSE_START = new Pose2d(62, -46, Math.toRadians(-47));


    Pose2d BLUE_FAR_START = new Pose2d(-61.25, 24, Math.toRadians(0));


    Pose2d BLUE_CLOSE_START = new Pose2d(55.5, 43.5, Math.toRadians(47));


    Pose2d BLUE_GOAL_START = new Pose2d(-47.5, 51.5,  Math.toRadians(323));


    //======MISC=======
    Vector2d BLUE_FAR_PARK = new Vector2d(-24, 24);


}