package org.firstinspires.ftc.teamcode.tele;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.gamepads.GamePad2;
import org.firstinspires.ftc.teamcode.gamepads.GamePad1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class ParentTeleOp extends LinearOpMode {
    public static HardwareMap globalHardwareMap;
    public static Telemetry globalTelemetry;
    public static Gamepad globalGamepad1;
    public static Gamepad globalGamepad2;
    private GamePad1 gp1;
    private GamePad2 gp2;
    DriveTrain.State centric;

    public ParentTeleOp(DriveTrain.State state) {
        centric = state;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        globalHardwareMap = hardwareMap;
        globalTelemetry = telemetry;
        globalGamepad1 = gamepad1;
        globalGamepad2 = gamepad2;
        gp1 = new GamePad1();
        gp2 = new GamePad2();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            gp1.update();
            gp2.update();
        }
    }
}