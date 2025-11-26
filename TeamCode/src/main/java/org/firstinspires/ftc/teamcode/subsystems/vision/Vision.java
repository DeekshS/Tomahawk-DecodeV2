package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Vision {

    AprilTagLibrary aprilTagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();

    AprilTagProcessor tagProcessor;

    VisionPortal visionPortal;

    List<AprilTagDetection> currentDetections;

    public Vision(HardwareMap hwmap) {

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                //.setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagLibrary(aprilTagLibrary)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hwmap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();
        visionPortal.setProcessorEnabled(tagProcessor, true);

    }

    public void update(Telemetry telemetry) {

        currentDetections = tagProcessor.getDetections();

        if (!currentDetections.isEmpty()) {
            for (AprilTagDetection tag : currentDetections) {
                if (tag.ftcPose != null) {

                    telemetry.addData("x: ", tag.ftcPose.x);
                    telemetry.addData("y: ", tag.ftcPose.y);
                    telemetry.addData("z: ", tag.ftcPose.z);
                    telemetry.addData("roll: ", tag.ftcPose.roll);
                    telemetry.addData("pitch: ", tag.ftcPose.pitch);
                    telemetry.addData("yaw: ", tag.ftcPose.yaw);
                }

                else {
                    telemetry.addLine("Null April Tag :(");
                }
                telemetry.addData("April Tag found with ID: ", tag.id);
            }
        }
        else {
            telemetry.addLine("No April Tag Detected");
        }

        telemetry.update();
    }

    public VisionPortal getVisionPortal() {return visionPortal;}

}
