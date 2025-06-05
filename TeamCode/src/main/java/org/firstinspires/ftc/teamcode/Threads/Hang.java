package org.firstinspires.ftc.teamcode.Threads;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.PID.ExtendoPID;

public class Hang {

    //Declarations
    DcMotorEx Extendo;
    //Gamepad gm;

    Telemetry telemetry = null;

    public static double kp = 0.01, ki = 0, kd = 0.00001;
    public static double target = 100;
    ExtendoPID generalPID = null;
    HardwareClass hardwareClass = null;

    //Singleton
    private static Hang single_instance = null;
    Thread thread = null;
    private boolean running = false;

    public Hang(HardwareClass hardwareClass, Telemetry telemetry){
        this.Extendo = hardwareClass.Push;
        this.telemetry = telemetry;
        this.hardwareClass = hardwareClass;
    }

    public void setup(){
        running = true;
        Extendo.setDirection(DcMotor.Direction.REVERSE);

        Extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        generalPID = ExtendoPID.getInstance(Extendo, "","",telemetry);
        generalPID.setCoefficients(kp,ki,kd);
        generalPID.start();
    }

    public void wait(int sec) throws InterruptedException{
        generalPID.delay(sec);
    }


    public void resetMotor(){
        generalPID.setReference(-100);
        Extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        generalPID.setReference(0);
        Extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void activate(){
        generalPID.activetePower();
    }

    public void still(){

    }

    public void goToPosition(double target){
        generalPID.setReference(target);
    }

    public void stop(){
        running = false;
        generalPID.stop();
    }

    public boolean getStatus(){
        return running;
    }

    public static synchronized Hang getInstance(HardwareMap hardwareMap, Telemetry telemetry){
        if(single_instance == null){
            single_instance = new Hang(HardwareClass.getInstance(hardwareMap), telemetry);
        }
        return single_instance;
    }
}
