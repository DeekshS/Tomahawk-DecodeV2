package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.tele.SET_AS_BLUE;

@TeleOp
@Config
public class AutoLock extends LinearOpMode {
    public static double targetAngle = 0;
    public static boolean enabled = false;

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Turret turret = new Turret(this);
        MecanumDrive drive = new MecanumDrive(this.hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();


        while (opModeIsActive()) {
            drive.localizer.update();
            turret.autoAlign(drive.localizer.getPose());
            turret.update();
            telemetry.addData("Target Angle", turret.getTargetAngle());
            telemetry.addData("Rotation", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            telemetry.addData("X", drive.localizer.getPose().position.x);
            telemetry.addData("Y", drive.localizer.getPose().position.y);
            telemetry.update();
        }
    }
}