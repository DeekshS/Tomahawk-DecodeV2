package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PoseStorage;

@TeleOp
public class SET_AS_RED extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        if (isStopRequested()) return;

        set();
    }

    public static void set() {
        PoseStorage.side = PoseStorage.SIDE.RED;
        PoseStorage.goalX = 72;
        PoseStorage.goalY = -72;
        PoseStorage.hpX = -56.4375;
        PoseStorage.hpY = -54.6875;
        PoseStorage.hpHeading = Math.toRadians(270);
    }
}
