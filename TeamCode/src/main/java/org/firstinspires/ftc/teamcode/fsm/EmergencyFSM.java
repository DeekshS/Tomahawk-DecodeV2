package org.firstinspires.ftc.teamcode.fsm;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.gamepad.GamepadMappings;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.OuttakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
@Config
public class EmergencyFSM {

    private Intake intake;
    private Outtake outtake;
//    private RTPAxon turret;           // turret controlled by RTPAxon
    private final Turret turret;
    private final Robot robot;
    private final GamepadMappings controls;
    private final Telemetry telemetry;
    private GazelleState gazelleState;
    public static int velocity = 1580;
    public EmergencyFSM(Telemetry telemetry, GamepadMappings controls, Robot robot) {
        this.robot = robot;
        this.intake = robot.intake;
        this.turret = robot.turret;
        this.outtake = robot.outtake;
        this.controls = controls;
        this.telemetry = telemetry;
    }

    // ---------------- Main update loop ----------------
    public void gazelleUpdate() {
        controls.update();
        robot.driveTrain.update();
        robot.drive.localizer.update();

        // ---------------- Outtake / Flywheel ----------------
        if (controls.flywheelClose.value()) {
            outtake.shootVelocity(OuttakeConstants.CLOSE_VELOCITY);
            outtake.hood.setPosition(OuttakeConstants.CLOSE_HOOD);
            intake.transferIn(1);
        } else if (controls.flywheelFar.value()) {
            outtake.shootVelocity(OuttakeConstants.FAR_VELOCITY);
            intake.transferIn(1);
        } else if (controls.autoVelo.value()) {
            outtake.autoVelocity(robot.drive.localizer.getPose());
            intake.transferIn(1);
        } else {
            outtake.shootVelocity(OuttakeConstants.OFF_VELOCITY);
        }

        // ---------------------- Turret ----------------------
        if (controls.turretAuto.value() || controls.flywheelClose.value() || controls.flywheelFar.value() || controls.autoVelo.value()) {
            turret.update();
        } else {
            turret.setTargetAngle(0);
            turret.update();
        }

        // ---------------- Intake / Transfer ----------------
        if (controls.intake.locked() || controls.intake2.locked()) {
            intake.intake();
            intake.transferStop();
        } else if (controls.intakeReverse.locked()) {
            intake.intakeReverse();
            intake.transferOut(1);
        } else if (controls.transfer.locked()) {
            intake.transferIn(1);
            telemetry.addLine("Transfer Locked");
        } else {
            intake.intakeStop();
            intake.transferStop();
        }

        telemetry.update();
    }

    // ---------------- Getters / Setters ----------------
    public enum GazelleState {
        BASE_STATE, INTAKING, TRANSFERRING
    }
}
