package org.firstinspires.ftc.teamcode.EXAMPLES.hardwaremaps;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EXAMPLES.hardwaremaps.motors.HowlersMotor;
import org.firstinspires.ftc.teamcode.EXAMPLES.subsystems.DriveTrain;

public class HowlersHardware {
    // static variable single_instance of type Singleton
    private static HowlersHardware instance = null;

    HardwareMap hwMap =  null;
    private ElapsedTime period  = new ElapsedTime();

    public HowlersMotor rightFront = null;
    public HowlersMotor leftFront = null;
    public HowlersMotor rightBack = null;
    public HowlersMotor leftBack = null;

    public DriveTrain driveTrain = null;

    // private constructor restricted to this class itself
    private HowlersHardware() {

    }

    // static method to create instance of Singleton class
    public static HowlersHardware getInstance() {
        if (instance == null)
            instance = new HowlersHardware();

        return instance;
    }

    public static void destroyInstance() {
        instance = null;
    }

    public static HowlersHardware resetInstance() {
        instance = new HowlersHardware();
        return instance;
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        driveTrain = new DriveTrain(hwMap);
    }
}