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
public class Blue15New extends LinearOpMode implements FCV2 {

    public static double INTAKE_WAIT_TIME = 3.5;
    public static double SHOOTER_TIME = 2.5;



    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this);
        BotActions botActions = new BotActions(robot);

        MecanumDrive drive = new MecanumDrive(hardwareMap, BLUE_CLOSE_START);


        Action preload = drive.actionBuilder(FCV2.BLUE_CLOSE_START)
                .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE)
                .build();

        Action artifact1 = drive.actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
                .strafeToLinearHeading(new Vector2d(FCV2.PPG_BLUE_ARTIFACT.x, FCV2.PPG_BLUE_ARTIFACT.y), FCV2.BLUE_ARTIFACT_ANGLE)
                .lineToY(FCV2.PPG_BLUE_ARTIFACT.y + 16)
                .strafeToLinearHeading(FCV2.BLUE_GATE, 0)
                .build();


        Action artifact1_return = drive.actionBuilder(new Pose2d(FCV2.BLUE_GATE.x, FCV2.BLUE_GATE.y, 0))
                .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE)
                .build();


        Action artifact2 = drive.actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
                .strafeToLinearHeading(FCV2.PGP_BLUE_ARTIFACT, FCV2.BLUE_ARTIFACT_ANGLE)

                .setTangent(FCV2.BLUE_ARTIFACT_ANGLE)
                //
                .lineToY(FCV2.PGP_BLUE_ARTIFACT.y+FCV2.ARTIFACT_DIST)
//            .setReversed(true)
//            .splineToLinearHeading(new Pose2d(FCV2.PGP_BLUE_ARTIFACT.x, FCV2.PGP_BLUE_ARTIFACT.y+FCV2.ARTIFACT_DIST, FCV2.BLUE_ARTIFACT_ANGLE), -Math.PI/2.2)

                .build();

        Action artifact2_return = drive.actionBuilder(new Pose2d(FCV2.PGP_BLUE_ARTIFACT.x, FCV2.PGP_BLUE_ARTIFACT.y+FCV2.ARTIFACT_DIST, FCV2.BLUE_ARTIFACT_ANGLE))

//            .strafeTo(FCV2.PGP_BLUE_ARTIFACT)
//            .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE-Math.toRadians(5-2))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE), Math.PI/8)

//                            .splineToLinearHeading(new Pose2d(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE))
//            .setReversed(true)
//            .strafeTo(FCV2.BLUE_CLOSE_SHOOT)

                .build();



        Action artifact3 = drive.actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
                .strafeToLinearHeading(FCV2.GPP_BLUE_ARTIFACT, FCV2.BLUE_ARTIFACT_ANGLE)

//            .setTangent(0)
//            .splineToConstantHeading(FCV2.GPP_BLUE_ARTIFACT, -0.75*Math.PI)
                .setTangent(FCV2.BLUE_ARTIFACT_ANGLE)
                .lineToY(FCV2.GPP_BLUE_ARTIFACT.y+FCV2.ARTIFACT_DIST)

                .build();

        Action artifact3_return = drive.actionBuilder(new Pose2d(FCV2.GPP_BLUE_ARTIFACT.x, FCV2.GPP_BLUE_ARTIFACT.y + FCV2.ARTIFACT_DIST, FCV2.BLUE_ARTIFACT_ANGLE))

                //                .setReversed(true)
                .strafeToLinearHeading(FCV2.BLUE_CLOSE_SHOOT, FCV2.BLUE_CLOSE_ANGLE)

                .build();

        Action park = drive.actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
                .strafeTo(new Vector2d(FCV2.PGP_BLUE_ARTIFACT.x, FCV2.PGP_BLUE_ARTIFACT.y-5))
                .build();

        Action hp_artifact = drive.actionBuilder(new Pose2d(FCV2.BLUE_CLOSE_SHOOT.x, FCV2.BLUE_CLOSE_SHOOT.y, FCV2.BLUE_CLOSE_ANGLE))
                .strafeToLinearHeading(FCV2.HP_BLUE_ARTIFACT, Math.toRadians(180))
                .strafeTo(new Vector2d(FCV2.HP_BLUE_ARTIFACT.x-20, FCV2.HP_BLUE_ARTIFACT.y))
                .build();


        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                preload,
                                robot.outtake.shootCloseAction()
                        ),
//
                        new ParallelAction(
                                robot.outtake.shootCloseAction(),
                                robot.intake.intakeTransferTimeAction(SHOOTER_TIME),
                                new SleepAction(SHOOTER_TIME)

                        ),

                        //FIRST SPIKE
                        new ParallelAction(
                                artifact1,
                                robot.intake.intakeTimeAction(INTAKE_WAIT_TIME)
                        ),
//
                        robot.intake.stop(),


                        new ParallelAction(
                                artifact1_return,
                                robot.outtake.shootCloseAction()
                        ),

                        new ParallelAction(
                                robot.outtake.shootVelocityTimeAction(1130, SHOOTER_TIME),
                                robot.intake.intakeTransferTimeAction(SHOOTER_TIME)
                        ),
//
                        //SECOND SPIKE
                        new ParallelAction(
                                artifact2,
                                robot.intake.intakeTimeAction(SHOOTER_TIME)

                        ),
//
                        robot.intake.stop(),

                        new ParallelAction(
                                artifact2_return,
                                robot.outtake.shootCloseAction()
                        ),
//
                        new ParallelAction(
                                robot.outtake.shootCloseAction(),
                                robot.intake.intakeTransferTimeAction(SHOOTER_TIME)
                        ),
//
                        //THIRD SPIKE

                        new ParallelAction(
                                artifact3,
                                robot.intake.intakeTimeAction(INTAKE_WAIT_TIME)
                        ),

                        robot.intake.stop(),
                        new ParallelAction(
                                artifact3_return,
                                robot.outtake.shootCloseAction()
                        ),
                        new ParallelAction(
                                robot.outtake.shootCloseAction(),
                                robot.intake.intakeTransferTimeAction(SHOOTER_TIME),
                                new SleepAction(SHOOTER_TIME)
                        ),
                        park
//
//
                )

        );
        robot.drive.localizer.update();
        PoseStorage.endPose = robot.drive.localizer.getPose();
        PoseStorage.side = PoseStorage.SIDE.BLUE;
    }

}