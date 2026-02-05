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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Config
public class CRTurret {

    public static boolean enabled = true; // allow manual override
    public static double TOLERANCE = 2;

    public static boolean gearOut = false;

    double power = 0;
    double error = 0;
    double currentAngle;
    double targetAngle = 0;
    double initialAngle;
    public double calculatedAngle;

    CRServo left;
    CRServo right;
    AnalogInput encoder;
    Limelight3A limelight;

    // Constructor
    public CRTurret(LinearOpMode mode) {
        left = mode.hardwareMap.get(CRServo.class, "turretLeft");
        right = mode.hardwareMap.get(CRServo.class, "turretRight");
        encoder = mode.hardwareMap.get(AnalogInput.class, "turretEncoderRight");
        limelight = mode.hardwareMap.get(Limelight3A.class, "ll");
        limelight.start();
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

    public double getCurrentAngle() {
        return currentAngle;
    }

    public double getError() {
        return error;
    }

    public double getPower() {
        return power;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getInitialAngle() {
        return initialAngle;
    }

    public boolean isAtTarget() {
        return Math.abs(error) < TOLERANCE;
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
        currentAngle = (((encoder.getVoltage() / 3.3 * 360) % 360) - 180) - initialAngle;
        targetAngle = (targetAngle > 180) ? targetAngle - 360 : targetAngle;
        targetAngle -= limelight.getLatestResult().getTx();

        error = (targetAngle - currentAngle) % 360;
        power = 0.1 * Math.log(1+Math.abs(error)) / Math.log(10);

        if (isAtTarget()) {
            power = 0;
        } else {
            if ((error < 0)) {
                power = -power;
            }
        }

        if (enabled) left.setPower(power); else left.setPower(0);
        if (enabled) right.setPower(power); else right.setPower(0);
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
