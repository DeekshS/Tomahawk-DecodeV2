package org.firstinspires.ftc.teamcode.fsm;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PoseStorage;
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
    private Intake transfer;
    private Outtake outtake;
    private final Turret turret;
    private final Robot robot;
    private final GamepadMappings controls;
    private final Telemetry telemetry;
    private GazelleState gazelleState;
    public String telemetry1;
    public static int velocity = 1580;
    public EmergencyFSM(Telemetry telemetry, GamepadMappings controls, Robot robot) {
        this.robot = robot;
        this.intake = robot.intake;
        this.transfer = robot.transfer;
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

        //Drivetrain stuff
        if (controls.reset.value()) {
            robot.drive.localizer.setPose(new Pose2d(PoseStorage.hpX, PoseStorage.hpY, PoseStorage.hpHeading));
        }

        // ---------------- Outtake / Flywheel ----------------
        if (controls.flywheelClose.value()) {
            outtake.shootVelocity(OuttakeConstants.CLOSE_VELOCITY);
            outtake.hood.setPosition(OuttakeConstants.CLOSE_HOOD);
            outtake.currentVelocity = OuttakeConstants.CLOSE_VELOCITY;
        } else if (controls.flywheelFar.value()) {
            outtake.shootVelocity(OuttakeConstants.FAR_VELOCITY);
            outtake.hood.setPosition(OuttakeConstants.FAR_HOOD);
            outtake.currentVelocity = OuttakeConstants.FAR_VELOCITY;
        }
//        else if (controls.autoVelo.value()) {
//            outtake.autoVelocity(robot.drive.localizer.getPose());
//            intake.transferIn(1);
//        }
        else {
            outtake.shootVelocity(OuttakeConstants.OFF_VELOCITY);
            outtake.currentVelocity = 0;
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
            transfer.setPower(0);
        } else if (controls.intakeReverse.locked()) {
            intake.intakeReverse();
            transfer.setPower(-1);
        } else if (controls.transfer.locked()) {
            intake.intake();
            if (controls.flywheelClose.value() || controls.flywheelFar.value()) {
                if (outtake.getVelocity() <= outtake.currentVelocity - OuttakeConstants.velocityError) {
                    telemetry1 = "transferHold";
                    transfer.setPower(-0.8);
                } else {
                    intake.transferIn(1);
                    telemetry1 = "transfer";
                }
            }
            else {
                intake.transferIn(1);
            }
            transfer.setPower(1);
        } else if (!controls.transfer.locked() && !controls.intakeReverse.locked()) {
            intake.intakeStop();
            transfer.setPower(0);
        }

        telemetry.update();
    }

    // ---------------- Getters / Setters ----------------
    public enum GazelleState {
        BASE_STATE, INTAKING, TRANSFERRING
    }
}
