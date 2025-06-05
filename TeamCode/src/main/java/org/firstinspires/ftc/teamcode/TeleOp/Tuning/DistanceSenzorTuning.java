package org.firstinspires.ftc.teamcode.TeleOp.Tuning;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@TeleOp
//@Config
public class DistanceSenzorTuning extends OpMode {
    
    public Rev2mDistanceSensor servo = null;
    public static String senzorName = "";
    public static double Position= 0;
    
    @Override
    public void init(){
        this.servo = hardwareMap.get(Rev2mDistanceSensor.class , senzorName);
    }
    
    @Override
    public void loop() {
        telemetry.addLine(servo.getDistance(DistanceUnit.MM) + " ");
    }
}