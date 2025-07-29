package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@Config
public class HardwareClass {


    //DcMotor

    public int CAMERA_MAX_RIGHT = 0;
    public int CAMERA_MAX_LEFT = 0;
    public int ROBOT_MAX_LEFT = 0;
    public int ROBOT_MAX_RIGHT = 0;

    public DcMotorEx Push;

    public DcMotor FR, FL , BR , BL;
    public DcMotorEx LS , RS;
    public double SLIDES_MAX_POWER = 1;

    public static double ExtendoMaxPoz = 0.5, ExtendoMinPoz = 0, ExtendoRestPoz = 0.02, ExtendoShortPoz = 0.5;    // Pozitii extendo
    /** SLIDES PID VALUES */
    public static double IntakeDown = 0.83, IntakeUp = 0.28, IntakeRest = 0.60;     ;

    public static int JumpHeight = 400;

    public static double Outtaketakepoz = 0  , OuttakeRestPoz = 0.3, Outtakebasketpoz = 1;

    public static double ClawOutClosedpoz = 0.6, ClawOutOpenedPoz = 0.3;

    public static double IN_ROBOT = 0, HIGH_BASKET = 890 ,LOW_BASKET = 350, HIGH_S_BASKET = 800;
    public double PREP_SPECIMEN = 330;
    public double PLACE_SPECIMEN = 330;

    public double HATZ_SPECIMEN = 90;

    /** AUTO COORDS*/

    /**  EXTENDO PID VALUES  */

    //Servo
    public static double SClawMax = 0.73, SClawMin = 0.49   ;//gheara

    public Servo ClawOut , OuttakeRotate, MobiDick , Cam, SClaw, SExtendo, IntakeRotate;

    public CRServo Brush;

    //Color senzor
    public ColorRangeSensor Yoda = null;

    private static HardwareClass hardwareClass = null;

    public HardwareClass(HardwareMap hardwareMap){
        //Chassy Motors
        this.FR = hardwareMap.get(DcMotor.class , "FR");
        this.FL = hardwareMap.get(DcMotor.class , "FL");
        this.BR = hardwareMap.get(DcMotor.class , "BR");
        this.BL = hardwareMap.get(DcMotor.class , "BL");
        //Slides
        this.RS = hardwareMap.get(DcMotorEx.class,"RS");
        this.LS = hardwareMap.get(DcMotorEx.class,"LS");
        //Servos
        this.Brush = hardwareMap.get(CRServo.class, "BRUSH");
        this.ClawOut = hardwareMap.get(Servo.class, "CO");
        this.SClaw = hardwareMap.get(Servo.class, "SC");
        this.OuttakeRotate = hardwareMap.get(Servo.class, "RO");
        //this.Cam = hardwareMap.get(Servo.class, "CAM");
        this.MobiDick = hardwareMap.get(Servo.class, "MD");
        this.SExtendo = hardwareMap.get(Servo.class, "SEX");
        this.IntakeRotate = hardwareMap.get(Servo.class, "IR");

        this.Yoda = hardwareMap.get(ColorRangeSensor.class,"YD");
        // this.IntakeSensor = hardwareMap.get(ColorRangeSensor.class, "IS");
    }

    public static synchronized HardwareClass getInstance(HardwareMap hardwareMap){
        if(hardwareClass == null)
            hardwareClass = new HardwareClass(hardwareMap);
        return hardwareClass;
    }
}
