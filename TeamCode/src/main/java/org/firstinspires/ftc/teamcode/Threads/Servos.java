package org.firstinspires.ftc.teamcode.Threads;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClass;

public class Servos {

    //Declarations
    public Servo ClawOut , OuttakeRotate, MobiDick , Cam, SClaw, SExtendo, IntakeRotate;

    public CRServo Brush;
    public  Servo SpecArm;
    private HardwareClass hardwareClass;

    int a  = 1, y = 1 , x = 1 , b = 1;

    //Singleton
    private static Servos single_instance = null;
    private boolean running = false;
    Thread thread = null;

    double SpecP , RIP , ROP , CIP, COP, PVP ;

    int trans = 0;

    public Servos(HardwareClass hardwareClass, Telemetry telemetry , HardwareMap hardwareMap){
        this.ClawOut = hardwareClass.ClawOut;
        //this.Extendo = hardwareClass.Extendo;
        this.IntakeRotate = hardwareClass.IntakeRotate;
        this.OuttakeRotate = hardwareClass.OuttakeRotate;
        this.hardwareClass = hardwareClass;
        this.SExtendo = hardwareClass.SExtendo;
        this.SClaw = hardwareClass.SClaw;
        this.Brush = hardwareClass.Brush;
    }

    public void start(){
        running = true;
        if(thread == null || !thread.isAlive()){
            thread = new Thread(() ->{
                while(running){
                    OuttakeRotate.setPosition(ROP);
                    IntakeRotate.setPosition(RIP);
                    ClawOut.setPosition(COP);
                }
            });
        }
        thread.start();
    }

    /** TeleOP */


    //Extendo
    public void ExtendoMax()
    {
        SExtendo.setPosition(HardwareClass.ExtendoMaxPoz);
    }
    public void ExtendoMin()
    {
        SExtendo.setPosition(HardwareClass.ExtendoMinPoz);
    }
    public void Extendorest()
    {
        SExtendo.setPosition(HardwareClass.ExtendoRestPoz);
    }
    public void ExtendoShort()
    {
        SExtendo.setPosition(HardwareClass.ExtendoShortPoz);
    }


    //Brush
    public void BrushOFF()
    {
        Brush.setPower(0);
    }
    public void BrushOn()
    {
        Brush.setPower(-1);
    }

    // Intake rotate

    public void IntakePreTake()
    {
        IntakeRotate.setPosition(HardwareClass.IntakeRest);
    }

    public void IntakeOut()
    {
        IntakeRotate.setPosition(HardwareClass.IntakeDown);
    }

    public void IntakeIn()
    {
        IntakeRotate.setPosition(HardwareClass.IntakeUp);

    }


    //Claw intake

    public void SClawIn()
    {
        SClaw.setPosition(HardwareClass.SClawMin);
    }

    public void SClawOut()
    {
        SClaw.setPosition(HardwareClass.SClawMax);
    }

    //Outtake Claw

    public void OutClawTake()
    {
        ClawOut.setPosition(HardwareClass.ClawOutClosedpoz);
    }

    public void OutClawOpen()
    {
        ClawOut.setPosition(HardwareClass.ClawOutOpenedPoz);
    }

    //Outtake rotate

    public void OutTakeRest()
    {
        OuttakeRotate.setPosition(HardwareClass.OuttakeRestPoz);
    }

    public void OutTakeBasket()
    {
        OuttakeRotate.setPosition(HardwareClass.Outtakebasketpoz);
    }

    public void OutTakeTrans()
    {
        OuttakeRotate.setPosition(HardwareClass.Outtaketakepoz);
    }






    public void wait(int sec){
        try {
            Thread.sleep(sec);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void waitThr(int sec){
        try {
            thread.sleep(sec);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void setup(){

    }

    public void resume(){
        running = true;
    }

    public void stop(){
        running = false;
    }

    public boolean getStatus(){
        return running;
    }

    public static synchronized Servos getInstance(HardwareMap hardwareMap , Telemetry telemetry ){
        if(single_instance == null){
            single_instance = new Servos(HardwareClass.getInstance(hardwareMap), telemetry , hardwareMap);
        }
        return single_instance;
    }
}
