package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

public class Functions {
    public static boolean inBetween(int a, int max, int min) {
        return a < max && a > min;
    }

    public static boolean inBetween(double a, double max, double min) {
        return a < max && a > min;
    }

    public static boolean greaterThan(int a, int b) {
        return a > b;
    }

    public static boolean greaterThan(double a, double b) {
        return a > b;
    }

    public static boolean lessThan(int a, int b) {
        return a < b;
    }

    public static boolean lessThan(double a, double b) {
        return a < b;
    }

    public static boolean isInShootingZone(Pose2d pose) {
        return (pose.position.x > 0 && Math.abs(pose.position.y) <= pose.position.x) || (pose.position.x <= -48 && Math.abs(pose.position.y) >= pose.position.x);
    }

    public static void runBlocking(Action action) {
        FtcDashboard dash = FtcDashboard.getInstance();
        Canvas previewCanvas = new Canvas();
        action.preview(previewCanvas);

        boolean running = true;
        while (running && !Thread.currentThread().isInterrupted()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());

            running = action.run(packet);

            dash.sendTelemetryPacket(packet);
        }
    }

    public static Action driveUpdate(double time, MecanumDrive drive) {
        return  new Action() {
            ElapsedTime t = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                drive.localizer.update();

                return t.seconds() < time;
            }
        };
    }
}
