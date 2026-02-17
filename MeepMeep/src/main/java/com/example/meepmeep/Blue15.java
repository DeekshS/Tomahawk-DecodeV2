package com.example.meepmeep;

import static com.example.meepmeep.FCV2.ARTIFACT_DIST;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

//import org.firstinspires.ftc.teamcode.autonomous.autos.FCV2;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;


public class Blue15 {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity d = new DefaultBotBuilder(meepMeep)
            .setConstraints(80, 80, 2.5, 3, 18)
            .setDimensions(16.53, 18)
            .build();

        DriveShim drive = d.getDrive();

        Action preload = drive.actionBuilder(FCV2.RED_CLOSE_START)
            .strafeToLinearHeading(FCV2.RED_CLOSE_SHOOT, FCV2.RED_CLOSE_ANGLE+Math.toRadians(13))
            .build();

        Action artifact1 = drive.actionBuilder(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE))
            .strafeToLinearHeading(new Vector2d(FCV2.PPG_RED_ARTIFACT.x, FCV2.PPG_RED_ARTIFACT.y), FCV2.RED_ARTIFACT_ANGLE)
            .lineToY(FCV2.PPG_RED_ARTIFACT.y - ARTIFACT_DIST - 1)
//            .strafeToLinearHeading(FCV2.RED_GATE, 0)
            .build();


        Action artifact1_return = drive.actionBuilder(new Pose2d(FCV2.RED_GATE.x, FCV2.RED_GATE.y, 0))
            .strafeToLinearHeading(FCV2.RED_CLOSE_SHOOT, FCV2.RED_CLOSE_ANGLE)
            .build();


        Action artifact2 = drive.actionBuilder(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE))
            .strafeToLinearHeading(FCV2.PGP_RED_ARTIFACT, FCV2.RED_ARTIFACT_ANGLE)

            .setTangent(FCV2.RED_ARTIFACT_ANGLE)

            .lineToY(FCV2.PGP_RED_ARTIFACT.y - ARTIFACT_DIST - 4)
//            .setReversed(true)
//            .splineToLinearHeading(new Pose2d(FCV2.PGP_RED_ARTIFACT.x, FCV2.PGP_RED_ARTIFACT.y+FCV2.ARTIFACT_DIST, FCV2.RED_ARTIFACT_ANGLE), -Math.PI/2.2)

            .build();

        Action artifact2_return = drive.actionBuilder(new Pose2d(FCV2.PGP_RED_ARTIFACT.x, FCV2.PGP_RED_ARTIFACT.y - ARTIFACT_DIST - 2, FCV2.RED_ARTIFACT_ANGLE))
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE), Math.PI/8)

            .build();



        Action artifact3 = drive.actionBuilder(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE))
            .strafeToLinearHeading(FCV2.GPP_RED_ARTIFACT_CLOSE, FCV2.RED_ARTIFACT_ANGLE)
            .strafeToConstantHeading(new Vector2d(FCV2.GPP_RED_ARTIFACT_CLOSE.x, FCV2.GPP_RED_ARTIFACT_CLOSE.y - ARTIFACT_DIST - 2))
//                .strafeToLinearHeading(FCV2.RED_GATE, 0)
            .build();

        Action artifact3_return = drive.actionBuilder(new Pose2d(FCV2.GPP_RED_ARTIFACT_CLOSE.x, FCV2.GPP_RED_ARTIFACT_CLOSE.y - ARTIFACT_DIST, FCV2.RED_ARTIFACT_ANGLE))

            //                .setReversed(true)
            .strafeToLinearHeading(FCV2.RED_CLOSE_SHOOT, FCV2.RED_CLOSE_ANGLE - Math.toRadians(4))

            .build();

        Action park = drive.actionBuilder(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE - 4))
            .strafeTo(new Vector2d(FCV2.PGP_RED_ARTIFACT.x, FCV2.PGP_RED_ARTIFACT.y - 5))
            .build();

        Action hp = drive.actionBuilder(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE))
            .strafeToLinearHeading(new Vector2d(FCV2.HP_RED_ARTIFACT.x + 14, FCV2.HP_RED_ARTIFACT.y), Math.toRadians(-180))
            .strafeTo(new Vector2d(FCV2.HP_RED_ARTIFACT.x, FCV2.HP_RED_ARTIFACT.y))
            .build();

        Action hp_return = drive.actionBuilder(new Pose2d(FCV2.HP_RED_ARTIFACT.x, FCV2.HP_RED_ARTIFACT.y, Math.toRadians(-180)))
//                .strafeToLinearHeading(FCV2.RED_CLOSE_SHOOT, FCV2.RED_CLOSE_ANGLE)
            .setTangent(Math.toRadians(0))
            .splineToLinearHeading(new Pose2d(FCV2.RED_CLOSE_SHOOT, FCV2.RED_CLOSE_ANGLE), Math.toRadians(-180))
            .build();

        d.runAction(
//            drive.getDrive().actionBuilder(new Pose2d(FCV2.HP_BLUE_ARTIFACT.x, FCV2.HP_BLUE_ARTIFACT.y, Math.toRadians(180)))
//                .setTangent(Math.toRadians(270))
//                .splineToLinearHeading(new Pose2d(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE), Math.toRadians(0))
//                .build()

            new SequentialAction(
                preload,
                artifact1,
//                botActions.preload_parallel_blue(preload),
//
//                botActions.shoot_parallel(),
//
//                robot.outtake.stopAction(),
//
//
//
//
//
//
//
//                botActions.intake_parallel(artifact1),
//
//                robot.intake.stop(),

                new ParallelAction(
                    artifact1_return
                    //                                subsystems.intake.intakeReverse(0.5),

//                    new SequentialAction(
//
////                        robot.outtake.reverseAction(.5),
////                        robot.outtake.shootVelocityAction(CLOSE_VELOCITY)
//
//                    )
                ),

//                robot.outtake.stopAction(),


//                new SequentialAction(
//                    //                                subsystems.outtake.shoot_close(),
//                    new ParallelAction(
////                        robot.outtake.shootVelocityTimeAction(CLOSE_VELOCITY, SHOOTER_TIME),
////                        robot.intake.intakeTimeAction(SHOOTER_TIME)
//                    )
//
//                ),

//                robot.outtake.stopAction(),





                // ARTIFACT 2

                new ParallelAction(
                    artifact2
//                    robot.intake.intakeTimeAction(INTAKE_WAIT_TIME),
//                    robot.outtake.shootVelocityTimeAction(CLOSE_VELOCITY, INTAKE_WAIT_TIME)
                ),
//                robot.intake.stop(),

                new ParallelAction(
                    artifact2_return
                    //                                subsystems.intake.intakeReverse(0.5),

//                    new SequentialAction(
//
////                        robot.outtake.reverseTimeAction(.5),
////                        robot.outtake.shootVelocityAction(CLOSE_VELOCITY)
//
//                    )
                ),

//                new SequentialAction(
//                    //                                subsystems.outtake.shoot_close(),
//                    new ParallelAction(
////                        robot.outtake.shootVelocityTimeAction(CLOSE_VELOCITY, SHOOTER_TIME),
////                        robot.intake.intakeTimeAction(SHOOTER_TIME)
//                    )
//
//                ),
//                robot.outtake.stopAction(),





                // ARTIFACT 3

                new ParallelAction(
                    artifact3
//                    robot.intake.intakeTimeAction(INTAKE_WAIT_TIME+.5),
//                    robot.outtake.reverseTimeAction(INTAKE_WAIT_TIME+.5)
                ),
//                robot.intake.stop(),

                new ParallelAction(
                    artifact3_return
                    //                                subsystems.intake.intakeReverse(0.5),

//                    new SequentialAction(
//                        robot.outtake.shootVelocityAction(CLOSE_VELOCITY)
//
//                    )
                ),
                hp

//                new SequentialAction(
//                    robot.intake.intakeTimeAction(SHOOTER_TIME)
//                )
            )

        );

        BufferedImage bg = null;
        try {
            bg = ImageIO.read(new File("MeepMeep/src/main/java/com/example/meepmeep/DECODE.png"));
        } catch (IOException e) {
            e.printStackTrace();
        }
        meepMeep.setBackground(bg)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(d)
            .start();


    }
}