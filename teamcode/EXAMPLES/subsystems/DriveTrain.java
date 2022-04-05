package org.firstinspires.ftc.teamcode.EXAMPLES.subsystems;


import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.EXAMPLES.hardwaremaps.HowlersHardware;
import org.firstinspires.ftc.teamcode.EXAMPLES.hardwaremaps.motors.HowlersMotor;

public class DriveTrain {

    private double speed = 0.6;
    public double setSpeed(double setter) {
        speed = setter;
        return speed;
    }
    public double getSpeed() {
        return speed;
    }

    private MotorGroup leftMotors;
    private MotorGroup rightMotors;
    private DifferentialDrive driveTrain;

    public DriveTrain(final HardwareMap hwMap) {
        HowlersHardware robot = HowlersHardware.getInstance();
        robot.rightFront = new HowlersMotor(hwMap, "rightFront", 134.4);
        robot.rightBack = new HowlersMotor(hwMap, "rightBack", 134.4);
        robot.leftBack = new HowlersMotor(hwMap, "leftBack", 134.4);
        robot.leftFront = new HowlersMotor(hwMap, "leftFront", 134.4);

        leftMotors = new MotorGroup(robot.rightFront, robot.rightBack);
        rightMotors = new MotorGroup(robot.leftBack, robot.leftFront);

        driveTrain = new DifferentialDrive(leftMotors, rightMotors);
    }

    public void drive(double forward, double turn) {
        driveTrain.arcadeDrive(forward, turn);
    }

    public boolean isBusy() {
        HowlersHardware robot = HowlersHardware.getInstance();
        boolean isBusy;
        if(robot.rightFront.busy() && robot.leftBack.busy() && robot.rightBack.busy() && robot.leftFront.busy()) isBusy = true;
        else isBusy = false;
        return isBusy;
    }

}