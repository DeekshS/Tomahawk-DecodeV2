package org.firstinspires.ftc.teamcode.autonomous.autos;

import static org.firstinspires.ftc.teamcode.subsystems.Outtake.OuttakeConstants.CLOSE_VELOCITY2;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.autonomous.autos.BotActions;
import org.firstinspires.ftc.teamcode.autonomous.autos.FCV2;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
@Config
public class test extends LinearOpMode implements FCV2 {

    //TODO: maybe make intake run for the entire thing

    public static double INTAKE_WAIT_TIME = 4.6;
    public static double SHOOTER_TIME = 2.5;

    public static int ARTIFACT_SHOOT_VEL = 1050;


    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);

        BotActions botActions = new BotActions();

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_CLOSE_START);



        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
//                        new Action() {
//                            @Override
//                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                                telemetry.addData("velocity", robot.outtake.getVelocity());
//                                telemetry.addData("setpoint", 1130);
//                                telemetry.update();
//                                return false;
//                            }
//                        },
                    Robot.outtake.shootCloseAction(),
                    Robot.intake.intakeTimeAction(30),
                    BotActions.transferHold(30)
                )


        );
        Robot.drive.localizer.update();
        PoseStorage.endPose = Robot.drive.localizer.getPose();
        PoseStorage.side = PoseStorage.SIDE.RED;
    }

}