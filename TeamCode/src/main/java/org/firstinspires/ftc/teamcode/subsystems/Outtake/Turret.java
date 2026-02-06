package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Config
public class Turret {

    public static boolean enabled = true; // allow manual override
    public static double multiplier = 0.00319444;
    double targetAngle = 0;
    double initialAngle;
    public double calculatedAngle;

    Servo left;
    Servo right;
    AnalogInput encoder;
//    Limelight3A limelight;

    // Constructor
    public Turret(LinearOpMode mode) {
        left = mode.hardwareMap.get(Servo.class, "turretLeft");
        right = mode.hardwareMap.get(Servo.class, "turretRight");
        encoder = mode.hardwareMap.get(AnalogInput.class, "turretEncoderRight");
//        limelight = mode.hardwareMap.get(Limelight3A.class, "ll");
//        limelight.start();
        initialAngle = Constants.turretOffset;
    }

    // --------------- Auto-Align --------------



    // ---------------- Control ----------------
    public void setEnabled(boolean enabled) {
        Turret.enabled = enabled;
    }

    public void setTargetAngle(double a) {
        targetAngle = a;
    }

    public void changeTargetAngle(double a) {
        targetAngle += a;
    }


    public double getTargetAngle() {
        return targetAngle;
    }

    public double getPosition() {
        return targetAngle * multiplier;
    }

    public double getInitialAngle() {
        return initialAngle;
    }

    public boolean isAtTarget() {
        return true;
    }

    public double autoAlign(Pose2d pose) {
        double robotX = pose.position.x;
        double robotY = pose.position.y;

        double deltaX = PoseStorage.goalX - robotX;
        double deltaY = PoseStorage.goalY - robotY;
        calculatedAngle = Math.toDegrees(Math.atan2(deltaY, deltaX)) - Math.toDegrees(pose.heading.toDouble());
        setTargetAngle(calculatedAngle);
        return calculatedAngle;
    }

    public void update() {
//        targetAngle += 180;
//        targetAngle = (targetAngle > 360) ? targetAngle - 360 : targetAngle;
//        targetAngle -= limelight.getLatestResult().getTx();

        left.setPosition((Math.round(targetAngle) + 184) * multiplier);
        right.setPosition((Math.round(targetAngle) + 184) * multiplier);
    }

    public Action alignAction(double angle, double time) {
        return  new Action() {

            ElapsedTime t = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setTargetAngle(angle);
                update();

                return t.seconds() < time;
            }
        };
    }

    public Action autoAlignAction(MecanumDrive drive, PoseStorage.SIDE side, double time) {
        return  new Action() {

            ElapsedTime t = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                drive.localizer.update();
                PoseStorage.goalX = 72;
                PoseStorage.goalY = (side.equals(PoseStorage.SIDE.BLUE) ? 72 : -72);
                autoAlign(drive.localizer.getPose());
                update();

                return t.seconds() < time;
            }
        };
    }

}
