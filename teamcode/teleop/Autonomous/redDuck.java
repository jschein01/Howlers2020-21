package org.firstinspires.ftc.teamcode.teleop.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Camera.BarcodePosition;
import org.firstinspires.ftc.teamcode.Camera.CameraRed;
import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
import org.firstinspires.ftc.teamcode.subsystems.LiftArm;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveTrain;


@Autonomous(name="redDuck")
public class redDuck extends OpMode {

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

        robot.cameraRed = new CameraRed(hardwareMap);
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
        BarcodePosition position = robot.cameraRed.getPosition();

        switch(autoRunState)
        {
            case TEST:
                //keep this case intact to use for testing
                turn = 175;
                scheduler = Scheduler.END;
                break;

            case STOP:
                if(robot.mecanumDriveTrain.YatSetPoint()){stop();}
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
                robot.cameraRed.endCamera();
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
                            robot.box.setPosition(0.64);
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
                            strafeDistance = 1600;
                            autoRunState = AutoRunState.STRAFERIGHT;
                            scheduler = Scheduler.THIRD;

                        }
                        break;
                    case THIRD:
                        if(robot.mecanumDriveTrain.XatSetPoint())
                        {
                            robot.mecanumDriveTrain.resetEncoders();
                            robot.mecanumDriveTrain.resetPID();
                            distance = 200;
                            autoRunState = AutoRunState.BACKWARD;

                            if(robot.mecanumDriveTrain.YatSetPoint())
                            {
                                robot.box.setPosition(0.76);
                                scheduler = Scheduler.FOURTH;
                            }

                        }

                        break;
                    case FOURTH:
                        if(robot.mecanumDriveTrain.isFinished(0.7))
                        {

                            robot.mecanumDriveTrain.resetEncoders();
                            robot.mecanumDriveTrain.resetPID();
                            robot.mecanumDriveTrain.resetTimer();
                            distance = 500;
                            autoRunState = AutoRunState.FORWARD;
                            scheduler = Scheduler.FIFTH;

                        }
                        break;
                    case FIFTH:
                        if(robot.mecanumDriveTrain.YatSetPoint())
                        {
                            robot.mecanumDriveTrain.resetEncoders();
                            robot.mecanumDriveTrain.resetPID();
                            robot.liftArm.liftHeight = LiftArm.LiftHeight.ZERO;
                            turn = 150;
                            autoRunState = AutoRunState.TURN;
                            scheduler = Scheduler.SEVENTH;
                        }
                        break;
                    case SIXTH:
                        if(robot.mecanumDriveTrain.TurnatSetPoint())
                        {
                            robot.mecanumDriveTrain.resetEncoders();
                            robot.mecanumDriveTrain.resetPID();
                            strafeDistance = 250;
                            autoRunState = AutoRunState.STRAFELEFT;
                            scheduler = Scheduler.SEVENTH;
                            robot.mecanumDriveTrain.resetTimer();

                        }
                        break;
                    case SEVENTH:
                        if(robot.mecanumDriveTrain.TurnatSetPoint() || robot.mecanumDriveTrain.isFinished(3))
                        {
                            robot.mecanumDriveTrain.resetEncoders();
                            robot.mecanumDriveTrain.resetPID();
                            robot.mecanumDriveTrain.resetTimer();
                            distance = 2000;
                            autoRunState = AutoRunState.FORWARD;
                            scheduler = Scheduler.EIGHTH;

                        }
                        break;
                    case EIGHTH:
                        if(robot.mecanumDriveTrain.YatSetPoint() || robot.mecanumDriveTrain.isFinished(3)) {
                            robot.mecanumDriveTrain.resetEncoders();
                            robot.mecanumDriveTrain.resetPID();
                            robot.duckWheelMotor.set(-0.5);
                            robot.mecanumDriveTrain.disablePID();
                            if (robot.mecanumDriveTrain.isFinished(6)) {
                                robot.mecanumDriveTrain.resetTimer();

                                scheduler = Scheduler.NINTH; // ending here
                            }
                        }
                        break;
                    case NINTH:
                    {
                        robot.duckWheelMotor.set(0);
                        distance = 200;
                        autoRunState = AutoRunState.BACKWARD;
                        scheduler = Scheduler.TENTH;

                    }
                    break;
                    case TENTH:
                        if(robot.mecanumDriveTrain.YatSetPoint())
                        {
                            robot.mecanumDriveTrain.resetEncoders();
                            robot.mecanumDriveTrain.resetPID();

                            turn = 180;
                            autoRunState = AutoRunState.TURN;
                            scheduler = Scheduler.ELEVENTH;
                        }
                        break;
                    case ELEVENTH:
                        if(robot.mecanumDriveTrain.TurnatSetPoint())
                        {
                            robot.mecanumDriveTrain.resetEncoders();
                            robot.mecanumDriveTrain.resetPID();
                            robot.mecanumDriveTrain.resetTimer();
                            distance = 575;
                            autoRunState = AutoRunState.BACKWARD;
                            scheduler = Scheduler.TWELTH;
                        }
                        break;
                    case TWELTH:
                        if(robot.mecanumDriveTrain.YatSetPoint() || robot.mecanumDriveTrain.isFinished(2))
                        {
                            robot.mecanumDriveTrain.resetEncoders();
                            robot.mecanumDriveTrain.resetPID();
                            robot.mecanumDriveTrain.resetTimer();
                            strafeDistance = 500;
                            autoRunState = AutoRunState.STRAFERIGHT;
                            scheduler = Scheduler.END;


                        }
                        break;
                    case END:
                        if(robot.mecanumDriveTrain.XatSetPoint() || robot.mecanumDriveTrain.isFinished(2))
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