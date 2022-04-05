 /* Copyright (c) 2017 FIRST. All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted (subject to the limitations in the disclaimer below) provided that
  * the following conditions are met:
  *
  * Redistributions of source code must retain the above copyright notice, this list
  * of conditions and the following disclaimer.
  *
  * Redistributions in binary form must reproduce the above copyright notice, this
  * list of conditions and the following disclaimer in the documentation and/or
  * other materials provided with the distribution.
  *
  * Neither the name of FIRST nor the names of its contributors may be used to endorse or
  * promote products derived from this software without specific prior written permission.
  *
  * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
  * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  */

 package org.firstinspires.ftc.teamcode.teleop;


 import com.arcrobotics.ftclib.drivebase.MecanumDrive;
 import com.arcrobotics.ftclib.gamepad.GamepadEx;
 import com.arcrobotics.ftclib.gamepad.GamepadKeys;
 import com.qualcomm.robotcore.eventloop.opmode.OpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.util.ElapsedTime;


 import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
 import org.firstinspires.ftc.teamcode.subsystems.LiftArm;
 import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveTrain;

 @TeleOp(name="HerbergerDrive", group="Iterative Opmode")
 public class HerbergerDrive extends OpMode
 {
     // Declare OpMode members.
     private ElapsedTime runtime = new ElapsedTime();
     Robot robot;
     double driveSpeed = 0.5;
     double prevSpeed = 0.5;
     double strafeSpeed = 0.5;
     double prevStrafe = 0.5;


     GamepadEx driverOp = null;
     GamepadEx toolOp = null;
     boolean buttonX;



     /*
      * Code to run ONCE when the driver hits INIT
      */
     @Override
     public void init() {
         robot = robot.resetInstance();
         robot.init(hardwareMap, MecanumDriveTrain.DriveMode.MANUAL);

         //Gamepad Initialization
         driverOp = new GamepadEx(gamepad1);
         toolOp = new GamepadEx(gamepad2);

         robot.box.setInverted(true);



         // Tell the driver that initialization is complete.

     }

     /*
      * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
      */
     @Override
     public void init_loop() {
     }

     /*
      * Code to run ONCE when the driver hits PLAY
      */
     @Override
     public void start() {

         runtime.reset();
         robot.box.setPosition(0.3);
         robot.liftArm.setHeight(0);
     }


     /*
      * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
      */

     @Override
     public void loop() {


         brake();
         driveTrainController();
         robot.liftArm.liftController();
         liftArm();
         boxFlip();
         duckWheel();
         intake(0.9);
         changeSpeed();
     }

     public void changeSpeed()
     {

         if(driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT)){prevSpeed = 0.4; prevStrafe = 0.5;}
         else if (driverOp.getButton(GamepadKeys.Button.DPAD_DOWN)){prevSpeed = 0.5; prevStrafe = 0.6;}
         else if (driverOp.getButton(GamepadKeys.Button.DPAD_LEFT)){prevSpeed = 0.6; prevStrafe = 0.7;}
         else if (driverOp.getButton(GamepadKeys.Button.DPAD_UP)){prevSpeed = 0.7; prevStrafe = 0.8;}
         else if (driverOp.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)){prevSpeed = 1; prevStrafe = 1;}


         if(driverOp.getButton(GamepadKeys.Button.A)){driveSpeed = 0.35; strafeSpeed = 0.35;} else {driveSpeed = prevSpeed; strafeSpeed = prevStrafe;}

     }


     public void driveTrainController()
     {
         double forward;
         double strafe;
         double turn = driverOp.getLeftX() * driveSpeed;

         if(Math.abs(driverOp.getRightY()) > 0.1 ){forward = driverOp.getRightY() * -driveSpeed;}
         else{forward = driverOp.getLeftY() * driveSpeed;}



         strafe = driverOp.getRightX() * strafeSpeed;

         if(Math.abs(driverOp.getRightX()) < 0.1)
         {

         if(driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER)){strafe = strafeSpeed;}
         else if (driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER)){strafe = -strafeSpeed;}

         }

         if(buttonX){strafe = 0; forward = 0; turn = 0;}




         robot.mecanumDriveTrain.drive(strafe,forward, turn);
     }


     public void liftArm() {
         if(toolOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) robot.liftArm.liftHeight = LiftArm.LiftHeight.ZERO;
         else if(toolOp.getButton(GamepadKeys.Button.DPAD_DOWN)) robot.liftArm.liftHeight= LiftArm.LiftHeight.BOTTOM;
         else if(toolOp.getButton(GamepadKeys.Button.DPAD_LEFT)) robot.liftArm.liftHeight = LiftArm.LiftHeight.MIDDLE;
         else if(toolOp.getButton(GamepadKeys.Button.DPAD_UP)) robot.liftArm.liftHeight = LiftArm.LiftHeight.TOP;
         else if(driverOp.getButton(GamepadKeys.Button.Y)) robot.liftArm.liftHeight = LiftArm.LiftHeight.CAP;
     }

     public void boxFlip()
     {
         boolean aButton = toolOp.getButton(GamepadKeys.Button.A);
         boolean bButton = toolOp.getButton(GamepadKeys.Button.B);
         boolean xButton = toolOp.getButton(GamepadKeys.Button.X);
         boolean yButton = toolOp.getButton(GamepadKeys.Button.Y);
         double rightTrigger = toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

         if(aButton)
         {
             robot.box.setPosition(0.79);
         }
         if(bButton)
         {
             robot.box.setPosition(0.85);


         }
         if(xButton)
         {
             robot.box.setPosition(0.5);

         }
         if(yButton)
         {
             robot.box.setPosition(0.64);
         }
         if(rightTrigger > 0.05){robot.box.setPosition(0.32);}
     }


     public void duckWheel()
     {
         boolean rightBumper = toolOp.getButton(GamepadKeys.Button.RIGHT_BUMPER);
         boolean leftBumper = toolOp.getButton(GamepadKeys.Button.LEFT_BUMPER);


         if(rightBumper)
         {
             robot.duckWheel.runDuckOpposite();

         }else if(leftBumper)
             {
                 robot.duckWheel.runDuckWheel();
             }
         else {
             robot.duckWheel.stopDuck();

         }
     }

     public void intake(double speed)
     {
         double intakeValue = driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * speed;
         double spitOut = driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * speed;

         if(intakeValue > 0.05 )
         {
             robot.intake.set(intakeValue);

         }else if(spitOut > 0.05)
         {
             robot.intake.set(-spitOut);
         }else
             {
                 robot.intake.set(0);
             }
     }

     public void brake()
     {
         buttonX = driverOp.getButton(GamepadKeys.Button.X);
         if(buttonX){
             robot.mecanumDriveTrain.stop();
         }
     }




     /*
      * Code to run ONCE after the driver hits STOP
      */
     @Override
     public void stop() {


         robot.mecanumDriveTrain.stop();
         robot.lift.stopMotor();
         robot.duckWheel.stopDuck();


     }


 }