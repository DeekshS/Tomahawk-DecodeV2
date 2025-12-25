package org.firstinspires.ftc.teamcode.fsm;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.gamepad.GamepadMappings;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.OuttakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.RTPAxon;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class EmergencyFSM {

    private Intake intake;
    private Outtake outtake;
    private RTPAxon turret;           // turret controlled by RTPAxon
    private Robot robot;
    private GamepadMappings controls;
    private Telemetry telemetry;
    private PinpointLocalizer pinpoint;
    private GazelleState gazelleState;
    private Intake transfer;

    // ---------------- Goal Positions ----------------
    private static final double RED_GOAL_X = 36.0;
    private static final double RED_GOAL_Y = 36.0;
    private static final double BLUE_GOAL_X = -36.0;
    private static final double BLUE_GOAL_Y = 36.0;

    // Current target color: true = red, false = blue
    private boolean targetIsRed = true;

    public EmergencyFSM(Telemetry telemetry, GamepadMappings controls, Robot robot, RTPAxon turret) {
        this.robot = robot;
        this.intake = robot.intake;
        this.outtake = robot.outtake;
        this.turret = turret;
        this.pinpoint = robot.pinpoint;
        this.controls = controls;
        this.telemetry = telemetry;
        this.transfer = robot.transfer;
        this.gazelleState = GazelleState.BASE_STATE;
    }

    // ---------------- Main update loop ----------------
    public void gazelleUpdate() {
        controls.update();
        robot.driveTrain.update();
        robot.drive.localizer.update();

        // ---------------- Toggle target color ----------------
        if (controls.turretRed.value()) {
            targetIsRed = !targetIsRed; // toggle target color
        }

        double goalX = targetIsRed ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = targetIsRed ? RED_GOAL_Y : BLUE_GOAL_Y;

        // ---------------- Outtake / Flywheel ----------------
        if (controls.flywheelClose.value()) {
            outtake.shootVelocity(OuttakeConstants.CLOSE_VELOCITY);
        } else if (controls.flywheelFar.value()) {
            outtake.shootVelocity(OuttakeConstants.FAR_VELOCITY);
        } else if (controls.autoVelo.value()) {
            // automatic velocity + hood based on robot position
            double robotX = robot.drive.localizer.getPose().position.x;
            double robotY = robot.drive.localizer.getPose().position.y;
            double distance = Math.hypot(goalX - robotX, goalY - robotY);

            // calculate automatic velocity using Outtake method
            int velocity = outtake.autoVelocity(robotX, robotY, goalX, goalY);
            outtake.shootVelocity(velocity);

            telemetry.addData("Auto Velocity", velocity);
            telemetry.addData("Distance", distance);
        } else {
            outtake.shootVelocity(OuttakeConstants.OFF_VELOCITY);
        }

        // ---------------- Intake / Transfer ----------------
        if (controls.intake.locked()) {
            intake.intake();
            transfer.setPower(0);
        } else if (controls.intakeReverse.locked()) {
            intake.intakeReverse();
            transfer.setPower(-1);
        } else if (controls.transfer.locked()) {
            transfer.setPower(1);
            intake.intake();
        } else {
            intake.intakeStop();
            transfer.setPower(0);
        }

        // ---------------- Turret control ----------------
        if (turret != null) {
            boolean autoAim = true; // always auto-aim at the current target color

            if (autoAim) {
                double robotX = robot.drive.localizer.getPose().position.x;
                double robotY = robot.drive.localizer.getPose().position.y;

                double deltaX = goalX - robotX;
                double deltaY = goalY - robotY;
                double targetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));
                if (targetAngle < 0) targetAngle += 360;

                turret.setTargetRotation(targetAngle);
                turret.setRtp(true);
            } else {
                // Manual control fallback
                double manualPower = 0;
                if (controls.turretLeft.locked()) manualPower = -0.2;
                else if (controls.turretRight.locked()) manualPower = 0.2;

                turret.setPower(manualPower);
                turret.setRtp(false);
            }

            turret.update(); // always update PID / rotation
        }

        // ---------------- FSM State Updates ----------------
        switch (gazelleState) {
            case BASE_STATE: break;
            case INTAKING: break;
            case TRANSFERRING: break;
        }

        telemetry.update();
    }

    // ---------------- Getters / Setters ----------------
    public GazelleState getState() { return gazelleState; }
    public void setState(GazelleState newState) { gazelleState = newState; }

    public enum GazelleState {
        BASE_STATE, INTAKING, TRANSFERRING
    }
}
