// package org.firstinspires.ftc.teamcode.autonomous.autos;

// import com.acmerobotics.roadrunner.Action;
// import com.acmerobotics.roadrunner.ParallelAction;
// import com.acmerobotics.roadrunner.SequentialAction;

// import org.firstinspires.ftc.teamcode.subsystems.Robot;

// public class BotActions {

//     Robot robot;

//     public static double INTAKE_WAIT_TIME = 3;
//     public static double SHOOTER_TIME = 2.5;



//     public BotActions(Robot robot) {
//         this.robot = robot;
//     }

//     public Action preload_parallel_blue(Action path) {

//         return new ParallelAction(
//                 path,
//                 robot.outtake.shoot_close()
// //                subsystems.turret.blue_init()
//         );
//     }


//     public Action preload_parallel_red(Action path) {

//         return new ParallelAction(
//                 path,
//                 robot.outtake.shoot_close()
// //                subsystems.turret.red_init()
//         );
//     }

//     public Action shoot_parallel() {

//         return new ParallelAction(
//                 robot.intake.intakeTime(2)
//         );
//     }

//     public Action shoot_close_spin_up() {

//         return new SequentialAction(
//                 robot.outtake.shoot_close()
// //                subsystems.intake.intake(SHOOTER_TIME)
//         );
//     }



//     public Action intake_parallel(Action path) {

//         return new ParallelAction(
//                 path,
//                 robot.intake.intake(INTAKE_WAIT_TIME),
//                 robot.outtake.shoot_reverse(INTAKE_WAIT_TIME)
//         );

//     }


//     public Action return_parallel(Action path) {

//         return new ParallelAction(
//                 path,
//                 robot.outtake.shoot_reverse(1)
//         );

//     }



// }
