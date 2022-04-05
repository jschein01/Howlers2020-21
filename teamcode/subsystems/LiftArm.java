package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
import org.firstinspires.ftc.teamcode.hardwaremaps.motors.HerbergerMotor;

public class LiftArm {

    public PIDController liftPID;
    private ElapsedTime timer = new ElapsedTime();
    private boolean zeroBool;

    public enum LiftHeight
    {
        TOP,
        MIDDLE,
        BOTTOM,
        ZERO,
        CAP,
    }

    public LiftHeight liftHeight = LiftHeight.ZERO;

    public LiftArm(final HardwareMap hwMap) {
        Robot robot = Robot.getInstance();
        robot.lift = new HerbergerMotor(hwMap, "lift", 134.4);
        robot.lift.runUsingEncoder();
        robot.lift.setInverted(false);
        robot.lift.resetEncoder();
        robot.box = new SimpleServo(hwMap,"box",0,90);
        robot.box.setInverted(true);

        liftPID = new PIDController(10, 0 ,0.01);
    }

    private double speed = 0.6;

    public void setSpeed(double speed) {
        this.speed = speed;
    }


    public double getSpeed() {
        return speed;
    }


    public void setHeight(double setHeight) {

        liftPID.setSetPoint(setHeight);

    }

    public void liftController() {
       Robot robot = Robot.getInstance();

        switch(liftHeight)
        {
            case ZERO:
                liftPID.setSetPoint(100);
                if(zeroBool){robot.box.setPosition(0.32); zeroBool = false;}
                stop();//10
                break;
            case BOTTOM:
                liftPID.setSetPoint(600);
                break;
            case MIDDLE:
                liftPID.setSetPoint(1075);
                break;
            case TOP:
                liftPID.setSetPoint(1775);
                break;
            case CAP:
                liftPID.setSetPoint(1850);
                break;
        }

       if(liftHeight != LiftHeight.ZERO){robot.lift.setVelocity(liftPID.calculate(robot.lift.getEncoderCount()));zeroBool = true;}
    }

    public boolean isBusy() {
        Robot robot = Robot.getInstance();
        boolean isBusy;
        if (robot.lift.busy())
            isBusy = true;
        else isBusy = false;
        return isBusy;
    }

    public void stop()
    {
        Robot robot = Robot.getInstance();
        robot.lift.set(0);
        robot.lift.resetEncoder();
    }

    public boolean isFinished()
    {
        timer.reset();
        if(timer.time() >= 0.4) return true;
        else return false;
    }
}