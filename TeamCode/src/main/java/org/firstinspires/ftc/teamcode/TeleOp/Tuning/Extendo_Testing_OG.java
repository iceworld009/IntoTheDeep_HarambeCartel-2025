package org.firstinspires.ftc.teamcode.TeleOp.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Threads.Extendo_OG;

//@TeleOp(name="Extendo Testing", group = "")
@Config
public class Extendo_Testing_OG extends LinearOpMode {
    
    DcMotorEx Extendo = null;
    public static double target = 0;

    Extendo_OG extendoOg = null;
    
    @Override
    public void runOpMode(){
        
        // after INIT
        Extendo = hardwareMap.get(DcMotorEx.class , "EX");

        //LeftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        Extendo.setDirection(DcMotor.Direction.REVERSE);
        Extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extendoOg = Extendo_OG.getInstance(hardwareMap , telemetry);
        extendoOg.start();

        waitForStart();
        
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive()){
            //What happens after START
            extendoOg.setTarget((int)target);
        }

        // kp = 0.01 , kd = 0, ki = 0 || max = 900 , transfer = 50
        
    }
}