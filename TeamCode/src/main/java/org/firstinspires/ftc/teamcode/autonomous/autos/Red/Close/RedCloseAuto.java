package org.firstinspires.ftc.teamcode.autonomous.autos.Red.Close;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.autos.BotActions;
import org.firstinspires.ftc.teamcode.autonomous.autos.FieldConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.PoseTransfer.PoseStorage;

@Autonomous
@Config
public class RedCloseAuto extends LinearOpMode implements FieldConstants {

    public static double INTAKE_WAIT_TIME = 3;
    public static double SHOOTER_TIME = 2.5;

    public static int ARTIFACT_SHOOT_VEL = 1765;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, null);
        BotActions botActions = new BotActions(robot);

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_CLOSE_START);

        // --- Trajectories ---
        Action preload = drive.actionBuilder(RED_CLOSE_START)
                .strafeToLinearHeading(RED_CLOSE_SHOOT, RED_CLOSE_ANGLE)
                .build();

        Action spike1 = drive.actionBuilder(new Pose2d(RED_CLOSE_SHOOT.x, RED_CLOSE_SHOOT.y, RED_CLOSE_ANGLE))
                .strafeToLinearHeading(PPG_RED_ARTIFACT, RED_ARTIFACT_ANGLE)
                .build();

        Action spike1_return = drive.actionBuilder(new Pose2d(PPG_RED_ARTIFACT.x, PPG_RED_ARTIFACT.y, RED_ARTIFACT_ANGLE))
                .strafeToLinearHeading(RED_CLOSE_SHOOT, RED_CLOSE_ANGLE)
                .build();

        Action spike2 = drive.actionBuilder(new Pose2d(RED_CLOSE_SHOOT.x, RED_CLOSE_SHOOT.y, RED_CLOSE_ANGLE))
                .strafeToLinearHeading(PGP_RED_ARTIFACT, RED_ARTIFACT_ANGLE)
                .build();

        Action spike2_return = drive.actionBuilder(new Pose2d(PGP_RED_ARTIFACT.x, PGP_RED_ARTIFACT.y, RED_ARTIFACT_ANGLE))
                .strafeToLinearHeading(RED_CLOSE_SHOOT, RED_CLOSE_ANGLE)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        // --- Autonomous Sequence ---
        Actions.runBlocking(
                new SequentialAction(

                        // Start outtake once, keep it running
                        robot.outtake.shootVelocityAction(ARTIFACT_SHOOT_VEL),

                        // Step 1: Preload
                        botActions.preload_parallel_red(preload),
                        robot.intake.intakeTimeAction(SHOOTER_TIME),
                        robot.intake.stop(),

                        // Step 2: Spike 1 cycle
                        new ParallelAction(
                                spike1,
                                robot.intake.intakeTimeAction(INTAKE_WAIT_TIME)
                        ),
                        robot.intake.stop(),
                        spike1_return,
                        robot.intake.intakeTimeAction(SHOOTER_TIME),
                        robot.intake.stop(),

                        // Step 3: Spike 2 cycle
                        new ParallelAction(
                                spike2,
                                robot.intake.intakeTimeAction(INTAKE_WAIT_TIME)
                        ),
                        robot.intake.stop(),
                        spike2_return,
                        robot.intake.intakeTimeAction(SHOOTER_TIME),
                        robot.intake.stop()

                        // Outtake keeps running the whole time, no stopAction
                )
        );

        // Save final pose for teleop
        PoseStorage.currentPose = robot.pinpoint.getPose();
    }
}