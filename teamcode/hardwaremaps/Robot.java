package org.firstinspires.ftc.teamcode.hardwaremaps;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Camera.Camera;
import org.firstinspires.ftc.teamcode.Camera.CameraRed;
import org.firstinspires.ftc.teamcode.hardwaremaps.motors.HerbergerMotor;
import org.firstinspires.ftc.teamcode.subsystems.DuckWheel;
import org.firstinspires.ftc.teamcode.subsystems.LiftArm;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveTrain;

public class Robot {

        // Static variable reference of single_instance
        // of type Singleton
        private static Robot single_instance = null;

        HardwareMap hwMap;
        private ElapsedTime period = new ElapsedTime();
        //drivetrain
        public HerbergerMotor rightFront;
        public HerbergerMotor leftFront;
        public HerbergerMotor rightBack;
        public HerbergerMotor leftBack;

        public BNO055IMU imu;

        public MecanumDriveTrain mecanumDriveTrain = null;

        public Camera camera = null;
        public CameraRed cameraRed = null;

        public HerbergerMotor intake = null;



        //duckwheel
        public HerbergerMotor duckWheelMotor = null;

        public DuckWheel duckWheel = null;

        //lift
        public HerbergerMotor lift = null;
        public ServoEx box;

        public LiftArm liftArm = null;




        // Constructor
        // Here we will be creating private constructor
        // restricted to this class itself
        private Robot()
        {

        }

        // Static method
        // Static method to create instance of Singleton class
        public static Robot getInstance()
        {
            if (single_instance == null)
                single_instance = new Robot();

            return single_instance;
        }
        public static Robot resetInstance()
        {
                single_instance = new Robot();
                return single_instance;
        }

        public void init(HardwareMap ahwMap, MecanumDriveTrain.DriveMode driveMode)
        {
            hwMap = ahwMap;
            mecanumDriveTrain = new MecanumDriveTrain(hwMap, driveMode);

            liftArm = new LiftArm(hwMap);
            duckWheel = new DuckWheel(hwMap);
            intake = new HerbergerMotor(hwMap,"intake",134.4);
            intake.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if(driveMode == MecanumDriveTrain.DriveMode.AUTONOMOUS)
            {

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hwMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
            }


        }

}
