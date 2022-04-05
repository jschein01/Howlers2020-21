package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
import org.firstinspires.ftc.teamcode.hardwaremaps.motors.HerbergerMotor;
import org.firstinspires.ftc.teamcode.teleop.Autonomous.AutoRunState;
import org.firstinspires.ftc.teamcode.teleop.Autonomous.Scheduler;

public class MecanumDriveTrain extends SubsystemBase
{
    private MecanumDrive mecanumDrive;

    private double speed = 0.6;

    public void setSpeed(double speed) { this.speed = speed; }

    public double getSpeed() { return speed; }

    public PIDFController driveTrainXPID;
    public PIDFController driveTrainYPID;
    public PIDFController driveTrainTurnPID;

    public double yCalculation;
    public double xCalculation;
    public double turnCalculation;

    public double turnAngle;
    public double turnTol;

    Orientation angles;

    DriveState driveState;

    private ElapsedTime timer = new ElapsedTime();
    private boolean start = true;

    public double currentHeading()
    {
        Robot robot = Robot.getInstance();
        double angle;
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if(angles.firstAngle < 0){angle = 360 - Math.abs(angles.firstAngle);} else angle = angles.firstAngle;
        if(angle >(360 - turnTol) || angle < turnTol){angle = 0;}
        return  angle;
    }



    public enum DriveMode
    {
        AUTONOMOUS,
        MANUAL,
    }

    public enum DriveState
    {
        Y,
        X,
        TURN,
        DIAG,
        ZERO,
    }

    public MecanumDriveTrain(final HardwareMap hwMap, MecanumDriveTrain.DriveMode driveMode) {
        Robot robot = Robot.getInstance();


        robot.rightBack = new HerbergerMotor(hwMap, "rightBack", 134.4);
        robot.rightFront = new HerbergerMotor(hwMap, "rightFront", 134.4);
        robot.leftBack = new HerbergerMotor(hwMap, "leftBack", 134.4);
        robot.leftFront = new HerbergerMotor(hwMap, "leftFront", 134.4);


        if(driveMode == DriveMode.MANUAL)
        {
            robot.rightFront.setInverted(true);
            robot.leftFront.setInverted(true);
            robot.leftBack.setInverted(true);
            robot.rightBack.setInverted(true);
        }


        robot.leftFront.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mecanumDrive = new MecanumDrive(robot.leftFront, robot.rightFront, robot.leftBack, robot.rightBack);

        driveTrainXPID = new PIDFController(0.00115,0.1,0.0005,0.00011);
        driveTrainYPID = new PIDFController(0.00115,0.1,0.0005,0.00011);

        driveTrainTurnPID = new PIDFController(0.03515,0,0.00001,0.0001);
        driveState = DriveState.ZERO;
    }

    public void drive(double strafe, double forward, double turn)
    {
        mecanumDrive.driveRobotCentric(strafe, forward, turn);
    }

    public void driveTrainController()
    {
        Robot robot = Robot.getInstance();
        switch(driveState)
        {
            case Y:
                xCalculation = 0;
                yCalculation = driveTrainYPID.calculate(robot.leftFront.getEncoderCount());
                turnCalculation = driveTrainTurnPID.calculate(currentHeading(), turnAngle);
                break;
            case TURN:
                xCalculation = 0;
                yCalculation = 0;
                turnCalculation = driveTrainTurnPID.calculate(currentHeading(), turnAngle);
                break;
            case X:
                xCalculation = driveTrainXPID.calculate(robot.rightFront.getEncoderCount());
                yCalculation = 0;
                turnCalculation = driveTrainTurnPID.calculate(currentHeading(), turnAngle);
                break;
            case DIAG:
                xCalculation = driveTrainXPID.calculate(robot.rightFront.getEncoderCount());
                yCalculation = driveTrainYPID.calculate(robot.leftFront.getEncoderCount());
                turnCalculation = driveTrainTurnPID.calculate(currentHeading(), turnAngle);
                break;
            case ZERO:
                xCalculation = 0;
                yCalculation = 0;
                turnCalculation = 0;
                break;
            default:
                break;

        }
        if(yCalculation > speed){yCalculation = speed;}
        if(yCalculation < -speed){yCalculation = -speed;}
        if(xCalculation > speed){xCalculation = speed;}
        if(xCalculation < -speed){xCalculation = -speed;}
        if(turnCalculation > speed){turnCalculation = speed;}
        if(turnCalculation < -speed){turnCalculation = -speed;}


        mecanumDrive.driveRobotCentric(xCalculation, yCalculation, turnCalculation);
    }

    public void autoDrive(double driveDist)
    {
        driveTrainYPID.setSetPoint(driveDist);
        driveState = DriveState.Y;
    }

    public void autoTurn(double turn)
    {
        turnAngle = turn;
        driveTrainTurnPID.setSetPoint(turnAngle);
        driveState = DriveState.TURN;
    }

    public void autoStrafe(double strafeDist)
    {
        driveState = DriveState.X;
        driveTrainXPID.setSetPoint(strafeDist);
    }

    public void autoDiag(double distance, double strafeDistance)
    {
        driveTrainYPID.setSetPoint(distance);
        driveTrainXPID.setSetPoint(strafeDistance);
        driveState = DriveState.DIAG;
    }

    public void resetEncoders()
    {
        Robot robot = Robot.getInstance();

        robot.leftBack.resetEncoder();
        robot.rightBack.resetEncoder();
        robot.leftFront.resetEncoder();
        robot.rightFront.resetEncoder();
    }
    public void resetPID()
    {
        driveTrainXPID.reset();
        driveTrainYPID.reset();
       // driveTrainTurnPID.reset();
    }

    public boolean XatSetPoint()
    {
        if(driveTrainXPID.atSetPoint()) return true;
        else return false;
    }

    public boolean YatSetPoint()
    {
        if(driveTrainYPID.atSetPoint()) return true;
        else return false;
    }
    public boolean TurnatSetPoint()
    {
        if(driveTrainTurnPID.atSetPoint() && !isBusy()) return true;
        else return false;
    }
    public boolean atSetPoint()
    {
        if(driveTrainXPID.atSetPoint() && driveTrainYPID.atSetPoint() && driveTrainTurnPID.atSetPoint()) return true;
        else return false;
    }

    public boolean isBusy() {
        Robot robot = Robot.getInstance();
        boolean isBusy;
        if (robot.rightFront.busy() && robot.leftBack.busy() && robot.rightBack.busy() && robot.leftFront.busy())
            isBusy = true;
        else isBusy = false;
        return isBusy;
    }

    public void stop()
    {
        Robot robot = Robot.getInstance();

        robot.rightBack.set(0);
        robot.leftBack.set(0);
        robot.rightFront.set(0);
        robot.leftFront.set(0);

    }

    public boolean isFinished(double time)
    {
        if(start == true)
        {
            start = false;
        timer.reset();
        }
        if(timer.seconds() > time)return true;
        else return false;
    }

    public void setTolerance(double tolerance)
    {
        driveTrainXPID.setTolerance(tolerance);
        driveTrainYPID.setTolerance(tolerance);
    }



    public void turnTolerance(double turnTolerance)
    {
        turnTol = turnTolerance;
        driveTrainTurnPID.setTolerance(turnTolerance);
    }

    public void dashBoard()
    {
        Robot robot = Robot.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("TurnSetpoint", turnAngle);
        packet.put("heading", currentHeading());
        packet.put("XSetpoint",robot.mecanumDriveTrain.driveTrainXPID.getSetPoint());
        packet.put("Encoder",robot.rightFront.getEncoderCount());
        packet.put("YSetpoint",robot.mecanumDriveTrain.driveTrainYPID.getSetPoint());
        packet.put("YEncoder",robot.leftFront.getEncoderCount());
        packet.put("timer",timer.seconds());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public void resetTimer()
    {
        start = true;
    }

    public void disablePID()
    {
        driveState = DriveState.ZERO;
    }
}