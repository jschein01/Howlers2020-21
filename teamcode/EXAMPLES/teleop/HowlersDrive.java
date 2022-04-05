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

package org.firstinspires.ftc.teamcode.EXAMPLES.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EXAMPLES.hardwaremaps.HowlersHardware;
@Disabled
@TeleOp(name="HowlersDrive", group="Iterative Opmode")

public class HowlersDrive extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HowlersHardware robot;

    //BasicDrive basicDrive;
    //ManualTurretController manualTurretController;

    GamepadEx driverOp = null;
    GamepadEx toolOp = null;

    double currentVelocity = 0;
    double setPoint = 0;

    boolean aButtonHeld = false;
    boolean triggerHeld = false;

    public enum AutoShooterState {
        MANUAL,
        ACTIVE,
        TAKE_CONTROL,
        START_FLYWHEEL,
        RELINQUISH_CONTROL,
    }

    public enum DriveMode {
        SLOW,
        FAST,
    }

    public DriveMode driveMode = DriveMode.FAST;

    public AutoShooterState autoShooterState = AutoShooterState.MANUAL;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = robot.resetInstance();

        robot.init(hardwareMap);

        //Gamepad Initialization
        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

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
        //CommandScheduler.getInstance().schedule(basicDrive);
        //CommandScheduler.getInstance().schedule(manualTurretController);
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        driveTrainController();
    }


    public void driveTrainController() {
        double speed = robot.driveTrain.getSpeed();

        boolean leftBumperState = driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER);
        boolean rightBumperState = driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER);

        double leftTrigger = driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rightTrigger = driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        if (leftBumperState) {
            if (!triggerHeld) {
                switch (driveMode) {
                    case FAST: {
                        robot.driveTrain.setSpeed(0.25);
                        driveMode = DriveMode.SLOW;
                    }
                    break;
                    case SLOW: {
                        robot.driveTrain.setSpeed(0.6);
                        driveMode = DriveMode.FAST;
                    }
                    break;
                }
                triggerHeld = true;
            }
        } else {
            triggerHeld = false;
        }

        double strafe = 0;
        double turn = driverOp.getLeftX() * speed;
        double forward = driverOp.getLeftY() * speed;

        if (leftTrigger > 0.2 && rightTrigger > 0.2) {
            strafe = 0;
        } else if (leftTrigger > 0.2) {
            strafe = leftTrigger * -1 * speed;
        } else if (rightTrigger > 0.2) {
            strafe = rightTrigger * 1 * speed;
        }


        robot.driveTrain.drive(forward, turn);
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }


}