package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
import org.firstinspires.ftc.teamcode.hardwaremaps.motors.HerbergerMotor;

public class DuckWheel {

    private ElapsedTime timer = new ElapsedTime();
    private boolean start = true;
    double speed;
    double duckSpeed;

    public DuckWheel(final HardwareMap hwMap)
    {
        Robot robot = Robot.getInstance();
        robot.duckWheelMotor = new HerbergerMotor(hwMap, "duckWheelMotor",134.4);
        robot.duckWheelMotor.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        speed = 0.5;

    }
    public void runDuckWheel()
    {
        Robot robot = Robot.getInstance();
        robot.duckWheelMotor.set(speed);
        speedControl(1.2);

    }
    public void runDuckOpposite()
    {
        Robot robot = Robot.getInstance();
        robot.duckWheelMotor.set(-speed);
        speedControl(1.2);

    }

    public void speedControl(double time) {

        double seconds = timer.seconds();
        speed = duckSpeed;
        if(duckSpeed > 0.85){speed = 0.85;}
        if(seconds <= time ){duckSpeed = 0.5 + (0.85 * (seconds / time));}
    }


    public void stopDuck()
    {
        Robot robot = Robot.getInstance();
        robot.duckWheelMotor.set(0);
        speed = 0.5;
        resetTimer();
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

    public void resetTimer()
    {
        timer.reset();
    }



}

/*switch(duckSpeed)
        {
            case START:
                speed = 0.5;
                duckSpeed = DuckSpeed.SLOW;
                break;
            case SLOW:
                if(isFinished(time))
                {
                    speed = 0.6;
                    resetTimer();
                    duckSpeed = DuckSpeed.MEDIUM;
                }
                break;
            case MEDIUM:
                if(isFinished(time))
                {
                    speed = 0.7;
                    resetTimer();
                    duckSpeed = DuckSpeed.FAST;
                }

                break;
            case FAST:
                if(isFinished(time))
                {
                    speed = 0.8;
                }
                break;
            default:
                speed = 0.5;
                break;

        }*/