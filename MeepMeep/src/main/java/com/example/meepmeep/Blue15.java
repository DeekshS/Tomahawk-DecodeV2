package com.example.meepmeep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

//import org.firstinspires.ftc.teamcode.autonomous.autos.FCV2;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;


public class Blue15 {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity drive = new DefaultBotBuilder(meepMeep)
            .setConstraints(80, 80, 2.5, 3, 18)
            .setDimensions(16.53, 18)
            .build();

        Action preload = drive.getDrive().actionBuilder(FCV2.BLUE_CLOSE_START)
                .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE)
                .build();

        Action artifact1 = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
                .strafeToLinearHeading(new Vector2d(FCV2.PPG_BLUE_ARTIFACT.x, FCV2.PPG_BLUE_ARTIFACT.y), FCV2.BLUE_ARTIFACT_ANGLE)
                .lineToY(FCV2.PPG_BLUE_ARTIFACT.y + 16)
                .strafeToLinearHeading(FCV2.BLUE_GATE, 0)
                .build();


        Action artifact1_return = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_GATE.x, FCV2.BLUE_GATE.y, 0))
                .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE)
                .build();


        Action artifact2 = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
                .strafeToLinearHeading(FCV2.PGP_BLUE_ARTIFACT, FCV2.BLUE_ARTIFACT_ANGLE)

                .setTangent(FCV2.BLUE_ARTIFACT_ANGLE)
                //
                .lineToY(FCV2.PGP_BLUE_ARTIFACT.y+FCV2.ARTIFACT_DIST)
//            .setReversed(true)
//            .splineToLinearHeading(new Pose2d(FCV2.PGP_BLUE_ARTIFACT.x, FCV2.PGP_BLUE_ARTIFACT.y+FCV2.ARTIFACT_DIST, FCV2.BLUE_ARTIFACT_ANGLE), -Math.PI/2.2)

                .build();

        Action artifact2_return = drive.getDrive().actionBuilder(new Pose2d(FCV2.PGP_BLUE_ARTIFACT.x, FCV2.PGP_BLUE_ARTIFACT.y+FCV2.ARTIFACT_DIST, FCV2.BLUE_ARTIFACT_ANGLE))

//            .strafeTo(FCV2.PGP_BLUE_ARTIFACT)
//            .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE-Math.toRadians(5-2))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE), Math.PI/8)

//                            .splineToLinearHeading(new Pose2d(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE))
//            .setReversed(true)
//            .strafeTo(FCV2.BLUE_CLOSE_SHOOT)

                .build();



        Action artifact3 = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
                .strafeToLinearHeading(FCV2.GPP_BLUE_ARTIFACT, FCV2.BLUE_ARTIFACT_ANGLE)

//            .setTangent(0)
//            .splineToConstantHeading(FCV2.GPP_BLUE_ARTIFACT, -0.75*Math.PI)
                .setTangent(FCV2.BLUE_ARTIFACT_ANGLE)
                .lineToY(FCV2.GPP_BLUE_ARTIFACT.y+FCV2.ARTIFACT_DIST)

                .build();

        Action artifact3_return = drive.getDrive().actionBuilder(new Pose2d(FCV2.GPP_BLUE_ARTIFACT.x, FCV2.GPP_BLUE_ARTIFACT.y + FCV2.ARTIFACT_DIST, FCV2.BLUE_ARTIFACT_ANGLE))

                //                .setReversed(true)
                .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE)

                .build();

        Action park = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
                .strafeTo(new Vector2d(FCV2.PGP_BLUE_ARTIFACT.x, FCV2.PGP_BLUE_ARTIFACT.y-5))
                .build();

        Action hp_artifact = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
                .strafeToLinearHeading(FCV2.HP_BLUE_ARTIFACT, Math.toRadians(180))
                .strafeTo(new Vector2d(FCV2.HP_BLUE_ARTIFACT.x-20, FCV2.HP_BLUE_ARTIFACT.y))
                .build();

        drive.runAction(
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
                hp_artifact

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
            .addEntity(drive)
            .start();


    }
}