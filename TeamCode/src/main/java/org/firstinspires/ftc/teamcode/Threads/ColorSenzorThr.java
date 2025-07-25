package org.firstinspires.ftc.teamcode.Threads;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareClass;

public class ColorSenzorThr {

    //Declarations
    public ColorRangeSensor Yoda;
    private HardwareClass hardwareClass;

    int a  = 1, y = 1 , x = 1 , b = 1;

    //Singleton
    private static ColorSenzorThr single_instance = null;
    private boolean running = false;
    Thread thread = null;

    double SpecP , RIP , ROP , CIP, COP, PVP ;

    int trans = 0;

    public ColorSenzorThr(HardwareClass hardwareClass, Telemetry telemetry , HardwareMap hardwareMap){
        this.hardwareClass = hardwareClass;
        this.Yoda = hardwareClass.Yoda;
    }

    public int getColor(){
        if(!checkSample_DIST()){
            return -1;
        }

        if(!checkSample_INTENS()){
            return -1;
        }

        if(Yoda.red() > Yoda.green() && Yoda.green() - 100 > Yoda.blue() && Yoda.alpha() > 3300){
            return 3;//YELLOW
        }

        if(Yoda.blue() > Yoda.green() && Yoda.blue() > Yoda.red()){
            return 2;//BLUE
        }

        /*
        if(Yoda.red() > Yoda.green() && Yoda.red() > Yoda.blue()){
            return 1;//RED
        }
        */
        return 1;
    }

    boolean checkSample_DIST(){
        if(Yoda.getDistance(DistanceUnit.MM) < 60){
            return true;
        }
        return false;
    }

    public boolean checkSample_INTENS(){
        if(Yoda.alpha() > 500){
            return true;
        }
        return false;
    }

    public void wait(int sec){
        try {
            Thread.sleep(sec);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public static synchronized ColorSenzorThr getInstance(HardwareMap hardwareMap , Telemetry telemetry ){
        if(single_instance == null){
            single_instance = new ColorSenzorThr(HardwareClass.getInstance(hardwareMap), telemetry , hardwareMap);
        }
        return single_instance;
    }
}
