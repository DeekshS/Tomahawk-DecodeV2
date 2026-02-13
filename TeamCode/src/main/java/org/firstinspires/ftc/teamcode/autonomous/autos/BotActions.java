 package org.firstinspires.ftc.teamcode.autonomous.autos;

 import androidx.annotation.NonNull;

 import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
 import com.acmerobotics.roadrunner.Action;
 import com.acmerobotics.roadrunner.ParallelAction;
 import com.acmerobotics.roadrunner.SequentialAction;
 import com.qualcomm.robotcore.util.ElapsedTime;

 import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
 import org.firstinspires.ftc.teamcode.subsystems.Outtake.OuttakeConstants;
 import org.firstinspires.ftc.teamcode.subsystems.Robot;

 public class BotActions {


     public static double INTAKE_WAIT_TIME = 3;
     public static double SHOOTER_TIME = 2.5;
     public static double transferPower = 1;

     public Action preload_parallel_blue(Action path) {

         return new ParallelAction(
                 path,
                 Robot.outtake.shootVelocityAction(OuttakeConstants.CLOSE_VELOCITY2)
 //                subsystems.turret.blue_init()
         );
     }


     public Action preload_parallel_red(Action path) {

         return new ParallelAction(
                 path,
                 Robot.outtake.shootVelocityAction(OuttakeConstants.CLOSE_VELOCITY2)
 //                subsystems.turret.red_init()
         );
     }

     public Action shoot_parallel() {

         return new ParallelAction(
             Robot.intake.intakeTimeAction(2)
         );
     }

     public Action shoot_close_spin_up() {

         return new SequentialAction(
                Robot.outtake.shootVelocityAction(OuttakeConstants.CLOSE_VELOCITY2)
 //                subsystems.intake.intake(SHOOTER_TIME)
         );
     }



     public Action intake_parallel(Action path) {

         return new ParallelAction(
                 path,
                 Robot.intake.intakeTimeAction(INTAKE_WAIT_TIME)
                 //robot.outtake.reverseTimeAction(INTAKE_WAIT_TIME)
         );

     }


     public Action return_parallel(Action path) {

         return new ParallelAction(
                 path
                 //robot.outtake.reverseTimeAction(1)
         );

     }



     public static Action transferHold(double time) {
         return new Action() {
             private boolean init = false;
             ElapsedTime timer = new ElapsedTime();


             @Override
             public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                 if (!init) {
                     init = true;
                     timer.reset();
                 }

                 if (timer.seconds() < time) {
                     Robot.intake.intake();
                     if (Math.abs(Robot.outtake.getVelocity()) <= Math.abs(OuttakeConstants.CLOSE_VELOCITY2) - Math.abs(OuttakeConstants.autonVelocityError)) {
                         Robot.intake.transferOut(Math.min(transferPower - 0.2, 0.4));

                     } else {
                         Robot.intake.transferIn(transferPower);
                     }
                 } else {
                     Robot.intake.stop();
                 }


                 return timer.seconds() <= time;
             }
         };
     }



 }
