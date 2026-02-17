package org.firstinspires.ftc.teamcode.autonomous.autos.Blue.Close;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.OuttakeConstants;
import org.firstinspires.ftc.teamcode.autonomous.autos.BotActions;
import org.firstinspires.ftc.teamcode.autonomous.autos.FCV2;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
@Config
public class Blue12Alliance extends LinearOpMode implements FCV2 {

    public static double INTAKE_WAIT_TIME = 2.75;
    public static double SHOOTER_TIME = 1.25;



    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);

        Robot.drive = new MecanumDrive(hardwareMap, FCV2.BLUE_CLOSE_START);

        MecanumDrive drive = Robot.drive;

        Action preload = drive.actionBuilder(FCV2.BLUE_CLOSE_START)
            .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE)
            .build();

        Action artifact1 = drive.actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
            .strafeToLinearHeading(new Vector2d(FCV2.PPG_BLUE_ARTIFACT.x, FCV2.PPG_BLUE_ARTIFACT.y), FCV2.BLUE_ARTIFACT_ANGLE)
            .lineToY(FCV2.PPG_BLUE_ARTIFACT.y + ARTIFACT_DIST-12)
            .strafeToLinearHeading(FCV2.BLUE_GATE, 0)
            .build();


        Action artifact1_return = drive.actionBuilder(new Pose2d(FCV2.BLUE_GATE.x, FCV2.BLUE_GATE.y, 0))
            .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE)
            .build();


        Action artifact2 = drive.actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
            .strafeToLinearHeading(FCV2.PGP_BLUE_ARTIFACT, FCV2.BLUE_ARTIFACT_ANGLE)

            .setTangent(FCV2.BLUE_ARTIFACT_ANGLE)

            .lineToY(FCV2.PGP_BLUE_ARTIFACT.y+FCV2.ARTIFACT_DIST+2)
//            .setReversed(true)
//            .splineToLinearHeading(new Pose2d(FCV2.PGP_BLUE_ARTIFACT.x, FCV2.PGP_BLUE_ARTIFACT.y+FCV2.ARTIFACT_DIST, FCV2.BLUE_ARTIFACT_ANGLE), -Math.PI/2.2)

            .build();

        Action artifact2_return = drive.actionBuilder(new Pose2d(FCV2.PGP_BLUE_ARTIFACT.x, FCV2.PGP_BLUE_ARTIFACT.y+FCV2.ARTIFACT_DIST+2, FCV2.BLUE_ARTIFACT_ANGLE))
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE), Math.PI/8)

            .build();



        Action artifact3 = drive.actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
            .strafeToLinearHeading(FCV2.GPP_BLUE_ARTIFACT_CLOSE, FCV2.BLUE_ARTIFACT_ANGLE)
            .strafeToConstantHeading(new Vector2d(FCV2.GPP_BLUE_ARTIFACT_CLOSE.x, FCV2.GPP_BLUE_ARTIFACT_CLOSE.y+ARTIFACT_DIST+2))
//                .strafeToLinearHeading(FCV2.BLUE_GATE, 0)
            .build();

        Action artifact3_return = drive.actionBuilder(new Pose2d(FCV2.GPP_BLUE_ARTIFACT_CLOSE.x, FCV2.GPP_BLUE_ARTIFACT_CLOSE.y + FCV2.ARTIFACT_DIST, FCV2.BLUE_ARTIFACT_ANGLE))

            //                .setReversed(true)
            .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE)

            .build();

        Action park = drive.actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
            .strafeTo(new Vector2d(FCV2.PGP_BLUE_ARTIFACT.x, FCV2.PGP_BLUE_ARTIFACT.y-5))
            .build();

//        Action hp = drive.actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
//            .strafeToLinearHeading(new Vector2d(FCV2.HP_BLUE_ARTIFACT.x + 14, FCV2.HP_BLUE_ARTIFACT.y), Math.toRadians(180))
//            .strafeTo(new Vector2d(FCV2.HP_BLUE_ARTIFACT.x, FCV2.HP_BLUE_ARTIFACT.y))
//            .build();
//
//        Action hp_return = drive.actionBuilder(new Pose2d(FCV2.HP_BLUE_ARTIFACT.x, FCV2.HP_BLUE_ARTIFACT.y, Math.toRadians(180)))
////                .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE)
//            .setTangent(Math.toRadians(0))
//            .splineToLinearHeading(new Pose2d(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE), Math.toRadians(180))
//            .build();
//
//        Action hp2 = drive.actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
////            .strafeToLinearHeading(FCV2.HP_BLUE_ARTIFACT, Math.toRadians(180))
//            .strafeToLinearHeading(new Vector2d(FCV2.HP_BLUE_ARTIFACT.x + 14, FCV2.HP_BLUE_ARTIFACT.y - 4), Math.toRadians(180))
//            .strafeTo(new Vector2d(FCV2.HP_BLUE_ARTIFACT.x, FCV2.HP_BLUE_ARTIFACT.y - 4))
//            .build();
//
//        Action hp_return2 = drive.actionBuilder(new Pose2d(FCV2.HP_BLUE_ARTIFACT.x, FCV2.HP_BLUE_ARTIFACT.y - 4, Math.toRadians(180)))
////                .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE)
//            .setTangent(Math.toRadians(270))
//            .splineToLinearHeading(new Pose2d(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE), Math.toRadians(0))
//            .build();


        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
            new ParallelAction(
                Robot.intake.intakeTimeAction(29.9),
                new SequentialAction(
                    new ParallelAction(
                        preload,
                        Robot.outtake.shootCloseAction()
                    ),
//
                    new ParallelAction(
                        Robot.outtake.shootCloseAction(),
//                        Robot.transfer.intakeTransferTimeAction(SHOOTER_TIME),
                        BotActions.transferHold(SHOOTER_TIME),
                        new SleepAction(SHOOTER_TIME)

                    ),
                    Robot.transfer.transferStopAction(),
                    //FIRST SPIKE
                    new ParallelAction(
                        artifact2,
                        Robot.intake.intakeTimeAction(INTAKE_WAIT_TIME)
                    ),
//
                    Robot.intake.stop(),


                    new ParallelAction(
                        artifact2_return,
                        Robot.outtake.shootCloseAction()
                    ),

                    new ParallelAction(
                        Robot.outtake.shootCloseAction(),
//                        Robot.transfer.intakeTransferTimeAction(SHOOTER_TIME),
                        BotActions.transferHold(SHOOTER_TIME),
                        new SleepAction(SHOOTER_TIME)

                    ),

                    Robot.transfer.transferStopAction(),
//
                    //SECOND SPIKE
                    new ParallelAction(
                        artifact1,
                        Robot.intake.intakeTimeAction(INTAKE_WAIT_TIME)

                    ),
//
                    Robot.intake.stop(),

                    new ParallelAction(
                        artifact1_return,
                        Robot.outtake.shootCloseAction()
                    ),
//
                    new ParallelAction(
                        Robot.outtake.shootCloseAction(),
//                        Robot.transfer.intakeTransferTimeAction(SHOOTER_TIME),
                        BotActions.transferHold(SHOOTER_TIME),
                        new SleepAction(SHOOTER_TIME)
                    ),
                    //THIRD SPIKE
                    Robot.transfer.transferStopAction(),
                    new ParallelAction(
                        artifact3,
                        Robot.intake.intakeTimeAction(INTAKE_WAIT_TIME)
                    ),

                    Robot.intake.stop(),
                    new ParallelAction(
                        artifact3_return,
                        Robot.outtake.shootCloseAction()
                    ),
                    new ParallelAction(
                        Robot.outtake.shootCloseAction(),
//                        Robot.transfer.intakeTransferTimeAction(SHOOTER_TIME),
                        BotActions.transferHold(SHOOTER_TIME),
                        new SleepAction(SHOOTER_TIME)

                    ),
                    Robot.transfer.transferStopAction(),
//                    new ParallelAction(
//                        hp,
//                        Robot.intake.intakeTimeAction(INTAKE_WAIT_TIME)
//                    ),
//
//                    Robot.intake.stop(),
//                    new ParallelAction(
//                        hp_return
////                                Robot.outtake.shootCloseAction(Robot)
//                    ),
//                    new ParallelAction(
//                        Robot.outtake.shootCloseAction(Robot),
//                        Robot.transfer.intakeTransferTimeAction(SHOOTER_TIME),
//                        new SleepAction(SHOOTER_TIME)
//
//                    ),
//                    Robot.transfer.transferStopAction(),
//                    new ParallelAction(
//                        hp2,
//                        Robot.intake.intakeTimeAction(INTAKE_WAIT_TIME)
//                    ),
//
//                    Robot.intake.stop(),
////                        new ParallelAction(
////                            hp_return2
////    //                                Robot.outtake.shootCloseAction(Robot)
////                        ),
////                        new ParallelAction(
////                            Robot.outtake.shootCloseAction(Robot),
////                            Robot.transfer.intakeTransferTimeAction(SHOOTER_TIME),
////                            new SleepAction(SHOOTER_TIME)
////
////                        ),
                    Robot.intake.stop(),
                    park,
                    PoseStorage.setEndPose(new Pose2d(new Vector2d(FCV2.PGP_BLUE_ARTIFACT.x, FCV2.PGP_BLUE_ARTIFACT.y-5), BLUE_CLOSE_ANGLE))
//
                )

            )
        );
    }

}