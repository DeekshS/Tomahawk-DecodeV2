package org.firstinspires.ftc.teamcode.autonomous.autos.Blue.Close;

import org.firstinspires.ftc.teamcode.autonomous.autos.FieldConstants;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.autonomous.autos.BotActions;

@Autonomous
public class BlueCloseAuto extends LinearOpMode implements FieldConstants {

    public static double INTAKE_WAIT_TIME = 4;
    public static double SHOOTER_TIME = 2;

    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this);
        BotActions botActions = new BotActions(robot);

        MecanumDrive drive = new MecanumDrive(hardwareMap, BLUE_CLOSE_START);

        Action preload = drive.actionBuilder(BLUE_CLOSE_START)
                .strafeToLinearHeading(BLUE_CLOSE_SHOOT, BLUE_CLOSE_ANGLE)
                .build();

        Action artifact1 = drive.actionBuilder(new Pose2d(BLUE_CLOSE_SHOOT.x, BLUE_CLOSE_SHOOT.y, BLUE_CLOSE_ANGLE))
                .strafeToLinearHeading(PPG_BLUE_ARTIFACT, BLUE_ARTIFACT_ANGLE)

                .build();

        Action artifact1_return = drive.actionBuilder(new Pose2d(PPG_BLUE_ARTIFACT.x, PPG_BLUE_ARTIFACT.y, BLUE_ARTIFACT_ANGLE))

                .strafeToLinearHeading(BLUE_CLOSE_SHOOT, BLUE_CLOSE_ANGLE)
//                .waitSeconds(0.85)

                .build();


        Action artifact2 = drive.actionBuilder(new Pose2d(BLUE_CLOSE_SHOOT.x, BLUE_CLOSE_SHOOT.y, BLUE_CLOSE_ANGLE))
                .strafeToLinearHeading(PGP_BLUE_ARTIFACT, BLUE_ARTIFACT_ANGLE)

                .setTangent(BLUE_ARTIFACT_ANGLE)
//
                .lineToY(PGP_BLUE_ARTIFACT.y+ARTIFACT_DIST+6)

                .build();

        Action artifact2_return = drive.actionBuilder(new Pose2d(PGP_BLUE_ARTIFACT.x, PGP_BLUE_ARTIFACT.y+ARTIFACT_DIST, BLUE_ARTIFACT_ANGLE))

//                .strafeTo(PGP_BLUE_ARTIFACT)
//                .strafeToLinearHeading(BLUE_CLOSE_SHOOT, BLUE_CLOSE_ANGLE+Math.toRadians(5-2))
//
//                .splineToLinearHeading(new Pose2d(BLUE_CLOSE_SHOOT))
                .setReversed(true)
                .splineToConstantHeading(BLUE_CLOSE_SHOOT, 0)

                .build();



        Action artifact3 = drive.actionBuilder(new Pose2d(BLUE_CLOSE_SHOOT.x, BLUE_CLOSE_SHOOT.y, BLUE_CLOSE_ANGLE))
//                .strafeToLinearHeading(GPP_BLUE_ARTIFACT, BLUE_ARTIFACT_ANGLE)

                .setTangent(Math.PI)
                .splineToConstantHeading(GPP_BLUE_ARTIFACT, Math.PI/2)
//                .waitSeconds(.2)
                .lineToY(GPP_BLUE_ARTIFACT.y+ARTIFACT_DIST+7)

                .build();

        Action artifact3_return = drive.actionBuilder(new Pose2d(GPP_BLUE_ARTIFACT.x, GPP_BLUE_ARTIFACT.y+ARTIFACT_DIST+7, BLUE_ARTIFACT_ANGLE))

//                .setReversed(true)
                .strafeToLinearHeading(BLUE_CLOSE_SHOOT, BLUE_CLOSE_ANGLE)

                .build();

        waitForStart();
        if (isStopRequested()) return;




        Actions.runBlocking(
                new SequentialAction(
                        botActions.preload_parallel_blue(preload),

                        botActions.shoot_parallel(),

                        robot.outtake.shoot_stop(),







                        botActions.intake_parallel(artifact1),

                        robot.intake.stop(),

                        new ParallelAction(
                                artifact1_return,
//                                subsystems.intake.intakeReverse(0.5),

                                new SequentialAction(

                                        robot.outtake.shoot_reverse(.5),
                                        robot.outtake.shoot_close()

                                )
                        ),
//                        subsystems.outtake.shoot_stop(),


                        new SequentialAction(
//                                subsystems.outtake.shoot_close(),
                                new ParallelAction(
                                        robot.outtake.shoot_close_time(SHOOTER_TIME),
                                        robot.intake.intake(SHOOTER_TIME)
                                )

                        ),

                        robot.outtake.shoot_stop(),





                        // ARTIFACT 2

                        new ParallelAction(
                                artifact2,
                                robot.intake.intake(INTAKE_WAIT_TIME),
                                robot.outtake.shoot_reverse(INTAKE_WAIT_TIME)
                        ),
                        robot.intake.stop(),

                        new ParallelAction(
                                artifact2_return,
//                                subsystems.intake.intakeReverse(0.5),

                                new SequentialAction(

                                        robot.outtake.shoot_reverse(.5),
                                        robot.outtake.shoot_close()

                                )
                        ),

                        new SequentialAction(
//                                subsystems.outtake.shoot_close(),
                                new ParallelAction(
                                        robot.outtake.shoot_close_time(SHOOTER_TIME),
                                        robot.intake.intake(SHOOTER_TIME)
                                )

                        ),
                        robot.outtake.shoot_stop(),





                        // ARTIFACT 3

                        new ParallelAction(
                                artifact3,
                                robot.intake.intake(INTAKE_WAIT_TIME-.5),
                                robot.outtake.shoot_reverse(INTAKE_WAIT_TIME-.5)
                        ),
                        robot.intake.stop(),

                        new ParallelAction(
                                artifact3_return,
//                                subsystems.intake.intakeReverse(0.5),

                                new SequentialAction(

                                        robot.outtake.shoot_reverse(.5),
                                        robot.outtake.shoot_close()

                                )
                        ),

                        new SequentialAction(
                                robot.outtake.shoot_close(),
                                robot.intake.intake(SHOOTER_TIME)
                        )
                )

        );




    }

}