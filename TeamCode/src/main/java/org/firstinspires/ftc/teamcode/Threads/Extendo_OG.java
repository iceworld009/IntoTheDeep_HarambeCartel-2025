package org.firstinspires.ftc.teamcode.Threads;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClass;

public class Extendo_OG {

    //Declarations
    DcMotorEx Extendo;
    //Gamepad gm;

    Telemetry telemetry = null;

    public static double target = 0;
    public static double Power = 1;
    double Multiplier = 1;
    HardwareClass hardwareClass = null;

    //Singleton
    private static Extendo_OG single_instance = null;
    Thread thread = null;
    private boolean running = false;

    public Extendo_OG(HardwareClass hardwareClass, Telemetry telemetry){
        this.Extendo = hardwareClass.Extendo;
        this.telemetry = telemetry;
        this.hardwareClass = hardwareClass;
    }

    public void setup(){
        running = true;
        Extendo.setDirection(DcMotor.Direction.REVERSE);

        Extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void start(){
        running = true;
        if(thread == null || !thread.isAlive()){
            thread = new Thread(() ->{
                while(running){
                    goToPosition((int)target);
                }
            });
        }
        thread.start();
    }

    public void wait(int sec){
        try {
            Thread.sleep(sec);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


    public void resetMotor(){
        Extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void goToPosition(int target){
        if(target != Extendo.getCurrentPosition()){
            Extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Extendo.setTargetPosition(target);
            Extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(Extendo.getCurrentPosition() - target < 0){
                Multiplier = -1;
            }else{
                Multiplier = 1;
            }
            Extendo.setPower(Power * Multiplier);

            while(Extendo.isBusy()){
                //Working on it :)
            }

            Extendo.setPower(0);
            Extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void stop(){
        running = false;
    }

    public void setTarget(int pos){
        target = pos;
    }

    public boolean getStatus(){
        return running;
    }

    public static synchronized Extendo_OG getInstance(HardwareMap hardwareMap, Telemetry telemetry){
        if(single_instance == null){
            single_instance = new Extendo_OG(HardwareClass.getInstance(hardwareMap), telemetry);
        }
        return single_instance;
    }
}
