package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fsm.EmergencyFSM;
import org.firstinspires.ftc.teamcode.gamepad.GamepadMappings;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
public class EmergencyTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize controls first
        GamepadMappings controls = new GamepadMappings(gamepad1, gamepad2);

        // Initialize Robot before FSM!
        Robot robot = new Robot(this);

        // Initialize FSM with fully constructed Robot
        EmergencyFSM fsm = new EmergencyFSM(telemetry, controls, robot);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Initialization complete");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            controls.update();
            fsm.gazelleUpdate();

            robot.drive.localizer.update();
            robot.turret.autoAlign(robot.drive.localizer.getPose());
            telemetry.addData("Velocity", robot.outtake.getVelocity());
            telemetry.addData("SetPoint", robot.outtake.currentVelocity);
            telemetry.addData("TransferState", fsm.telemetry1);
            // Optional telemetry
            telemetry.update();
        }
    }
}
