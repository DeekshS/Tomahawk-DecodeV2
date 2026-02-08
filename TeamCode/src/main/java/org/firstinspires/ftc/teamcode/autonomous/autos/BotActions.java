 package org.firstinspires.ftc.teamcode.autonomous.autos;

 import androidx.annotation.NonNull;

 import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
 import com.acmerobotics.roadrunner.Action;
 import com.acmerobotics.roadrunner.ParallelAction;
 import com.acmerobotics.roadrunner.SequentialAction;
 import com.qualcomm.robotcore.util.ElapsedTime;

 import org.firstinspires.ftc.teamcode.Constants;
 import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
 import org.firstinspires.ftc.teamcode.subsystems.Outtake.OuttakeConstants;
 import org.firstinspires.ftc.teamcode.subsystems.Robot;

 public class BotActions {

     Robot robot;

     public static double INTAKE_WAIT_TIME = 3;
     public static double SHOOTER_TIME = 2.5;
     public static double transferPower = 1;



     public BotActions(Robot robot) {
         this.robot = robot;
     }

     public Action preload_parallel_blue(Action path) {

         return new ParallelAction(
                 path,
                 robot.outtake.shootVelocityAction(FieldConstants.CLOSE_VELOCITY)
 //                subsystems.turret.blue_init()
         );
     }


     public Action preload_parallel_red(Action path) {

         return new ParallelAction(
                 path,
                 robot.outtake.shootVelocityAction(FieldConstants.CLOSE_VELOCITY)
 //                subsystems.turret.red_init()
         );
     }

     public Action shoot_parallel() {

         return new ParallelAction(
                 robot.intake.intakeTimeAction(2)
         );
     }

     public Action shoot_close_spin_up() {

         return new SequentialAction(
                 robot.outtake.shootVelocityAction(FieldConstants.CLOSE_VELOCITY)
 //                subsystems.intake.intake(SHOOTER_TIME)
         );
     }



     public Action intake_parallel(Action path) {

         return new ParallelAction(
                 path,
                 robot.intake.intakeTimeAction(INTAKE_WAIT_TIME)
                 //robot.outtake.reverseTimeAction(INTAKE_WAIT_TIME)
         );

     }


     public Action return_parallel(Action path) {

         return new ParallelAction(
                 path
                 //robot.outtake.reverseTimeAction(1)
         );

     }

//     public static Action transferHold(Robot robot, double time) {
//         return new Action() {
//             @Override
//             public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                 ElapsedTime timer = new ElapsedTime();
//                 while (timer.seconds() <= time) {
//
//                 }
//                 return false;
//             }
//         };
//     }

     public static Action transferHold(Robot robot, double time) {
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
                     if (Math.abs(robot.outtake.getVelocity()) <= Math.abs(OuttakeConstants.CLOSE_VELOCITY2) - Math.abs(OuttakeConstants.velocityError)) {
                         robot.intake.transferOut(Math.min(transferPower - 0.2, 0.4));
                     } else {
                         robot.intake.transferIn(transferPower);
                     }
                     return true;
                 } else {
                     robot.intake.stop();
                 }


                 return timer.seconds() <= time;
             }
         };
     }



 }
