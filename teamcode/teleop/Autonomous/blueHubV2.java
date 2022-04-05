package org.firstinspires.ftc.teamcode.teleop.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Camera.BarcodePosition;
import org.firstinspires.ftc.teamcode.Camera.Camera;
import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
import org.firstinspires.ftc.teamcode.subsystems.LiftArm;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveTrain;


@Autonomous(name="blueHubv2")
public class blueHubV2 extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    Robot robot;
    AutoRunState autoRunState;
    Scheduler scheduler;
    double distance = 500;
    double strafeDistance = 500;
    double turn = 90;


    @Override
    public void init()
    {
        robot = robot.resetInstance();
        robot.init(hardwareMap, MecanumDriveTrain.DriveMode.AUTONOMOUS);

        robot.camera = new Camera(hardwareMap);
        robot.liftArm.liftHeight = LiftArm.LiftHeight.ZERO;
        robot.mecanumDriveTrain.resetEncoders();

    }

    @Override
    public void init_loop()
    {
        robot.mecanumDriveTrain.dashBoard();
    }

    @Override
    public void start()
    {
        runtime.reset();

        autoRunState = AutoRunState.START;
        scheduler = Scheduler.FIRST;
        robot.mecanumDriveTrain.resetEncoders();
        robot.mecanumDriveTrain.setTolerance(10);
        robot.mecanumDriveTrain.turnTolerance(2);
        robot.mecanumDriveTrain.setSpeed(0.6);

    }

    @Override
    public void loop()
    {
        robot.mecanumDriveTrain.driveTrainController();
        robot.liftArm.liftController();
        BarcodePosition position = robot.camera.getPosition();

        switch(autoRunState)
        {
            case TEST:
                //keep this case intact to use for testing
                autoRunState = AutoRunState.STOP;
                break;

            case STOP:
                stop();
                break;

            case START:
                switch(position)
                {
                    case RIGHT:
                        robot.liftArm.liftHeight = LiftArm.LiftHeight.TOP;
                        break;
                    case CENTER:
                        robot.liftArm.liftHeight = LiftArm.LiftHeight.MIDDLE;
                        break;
                    case LEFT:
                        robot.liftArm.liftHeight = LiftArm.LiftHeight.BOTTOM;
                        break;
                }
                robot.camera.endCamera();
                robot.box.setPosition(0.35);
                distance = 500;
                autoRunState = AutoRunState.FORWARD;

                break;
            case DEPOSIT:
                robot.box.setPosition(0.76);
                autoRunState = AutoRunState.SCHEDULE;
                break;
            case TURN:
                robot.mecanumDriveTrain.autoTurn(turn);
                autoRunState = AutoRunState.SCHEDULE;
                break;
            case FORWARD:
                robot.mecanumDriveTrain.autoDrive(-distance);
                autoRunState = AutoRunState.SCHEDULE;
                break;
            case BACKWARD:
                robot.mecanumDriveTrain.autoDrive(distance);
                autoRunState = AutoRunState.SCHEDULE;
                break;
            case STRAFERIGHT:
                robot.mecanumDriveTrain.autoStrafe(-strafeDistance);
                autoRunState = AutoRunState.SCHEDULE;
                break;
            case STRAFELEFT:
                robot.mecanumDriveTrain.autoStrafe(strafeDistance);
                autoRunState = AutoRunState.SCHEDULE;
                break;
            case DIAGONAL:
                robot.mecanumDriveTrain.autoDiag(distance,strafeDistance);
                autoRunState = AutoRunState.SCHEDULE;
                break;


            case SCHEDULE:
                switch (scheduler)
                {
                    case FIRST:
                        if(robot.mecanumDriveTrain.YatSetPoint())
                        {
                            robot.mecanumDriveTrain.resetEncoders();
                            robot.mecanumDriveTrain.resetPID();
                            turn = 90;
                            autoRunState = AutoRunState.TURN;
                            scheduler = Scheduler.SECOND;



                        }

                        break;
                    case SECOND:
                        if(robot.mecanumDriveTrain.TurnatSetPoint())
                        {
                            robot.mecanumDriveTrain.resetEncoders();
                            robot.mecanumDriveTrain.resetPID();
                            robot.box.setPosition(0.64);
                            strafeDistance = 850;
                            autoRunState = AutoRunState.STRAFERIGHT;
                            scheduler = Scheduler.THIRD;

                        }
                        break;
                    case THIRD:
                        if(robot.mecanumDriveTrain.XatSetPoint())
                        {
                            robot.mecanumDriveTrain.resetEncoders();
                            robot.mecanumDriveTrain.resetPID();

                            autoRunState = AutoRunState.DEPOSIT;
                            scheduler = Scheduler.FOURTH;
                        }
                        break;
                    case FOURTH:
                        if(robot.mecanumDriveTrain.isFinished(0.8))
                        {
                            robot.mecanumDriveTrain.resetEncoders();
                            robot.mecanumDriveTrain.resetPID();
                            robot.mecanumDriveTrain.resetTimer();
                            strafeDistance = 2500;
                            autoRunState = AutoRunState.STRAFELEFT;
                            scheduler = Scheduler.FIFTH;
                        }
                        break;
                    case FIFTH:
                        if(robot.mecanumDriveTrain.XatSetPoint() || robot.mecanumDriveTrain.isFinished(3))
                        {
                            robot.mecanumDriveTrain.resetEncoders();
                            robot.mecanumDriveTrain.resetPID();
                            robot.mecanumDriveTrain.resetTimer();
                            robot.liftArm.liftHeight = LiftArm.LiftHeight.ZERO;
                            robot.mecanumDriveTrain.setSpeed(0.4);
                            distance = 1700;
                            autoRunState = AutoRunState.FORWARD;
                            scheduler = Scheduler.END;
                        }
                        break;
                    case END:
                        if(robot.mecanumDriveTrain.YatSetPoint())
                        {
                            autoRunState = AutoRunState.STOP;
                        }
                        break;


                }
                break;

        }

        robot.mecanumDriveTrain.dashBoard();
        telemetry.addData("autoRunState",autoRunState);
        telemetry.addData("schedule",scheduler);
        telemetry.update();

    }

    @Override
    public void stop()
    {

        robot.mecanumDriveTrain.stop();
        robot.liftArm.liftHeight = LiftArm.LiftHeight.ZERO;
        robot.lift.set(0);
        robot.intake.set(0);
    }
}