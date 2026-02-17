package org.firstinspires.ftc.teamcode.tuners;








import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;








import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.pid.MiniPID;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;








@TeleOp
@Config
public class AutoAlignTuner extends LinearOpMode {
    public static double pos = 0;
    DriveTrain driveTrain;
    MecanumDrive drive;








    public void runOpMode() {
        driveTrain = new DriveTrain(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());




        waitForStart();
        if (isStopRequested()) return;








        while (opModeIsActive()) {
            driveTrain.update(this.getRuntime());


            telemetry.addData("LL Result", driveTrain.result);
            telemetry.addData("error", driveTrain.error);
            telemetry.addData("text", driveTrain.text);
//            telemetry.addData("testing223", driveTrain.testingResult);
            telemetry.addData("pipeline", (PoseStorage.side.equals(PoseStorage.SIDE.BLUE) ? 1 : 0));
            telemetry.update();
        }








    }
}









