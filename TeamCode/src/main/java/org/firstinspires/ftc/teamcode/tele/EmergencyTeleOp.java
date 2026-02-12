package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.fsm.EmergencyFSM;
import org.firstinspires.ftc.teamcode.gamepad.GamepadMappings;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
public class EmergencyTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize controls first
        GamepadMappings controls = new GamepadMappings(gamepad1, gamepad2);

        // Initialize FSM with fully constructed Robot
        EmergencyFSM fsm = new EmergencyFSM(this, telemetry, controls);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Initialization complete");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            controls.update();
            fsm.gazelleUpdate();

            Robot.drive.localizer.update();
            telemetry.addData("Velocity", Robot.outtake.getVelocity());
            telemetry.addData("SetPoint", Robot.outtake.currentVelocity);
            telemetry.addData("TransferState", fsm.telemetry1);
            telemetry.addData("botX", Robot.drive.localizer.getPose().position.x);
            telemetry.addData("botY", Robot.drive.localizer.getPose().position.y);

            // Optional telemetry
            telemetry.update();
        }
    }
}
