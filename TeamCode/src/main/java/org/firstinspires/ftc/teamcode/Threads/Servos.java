package org.firstinspires.ftc.teamcode.Threads;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClass;

public class Servos {

    //Declarations
    public Servo ClawIn , ClawOut , IntakeRotate, OuttakeRotate, Pivot, Specimen, SpecClaw;
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
        this.ClawIn = hardwareClass.ClawIn;
        this.ClawOut = hardwareClass.ClawOut;
        //this.Extendo = hardwareClass.Extendo;
        this.IntakeRotate = hardwareClass.IntakeRotate;
        this.OuttakeRotate = hardwareClass.OuttakeRotate;
        this.Pivot = hardwareClass.Pivot;
        this.hardwareClass = hardwareClass;
    }

    public void start(){
        running = true;
        if(thread == null || !thread.isAlive()){
            thread = new Thread(() ->{
                while(running){
                    Specimen.setPosition(SpecP);
                    OuttakeRotate.setPosition(ROP);
                    IntakeRotate.setPosition(RIP);
                    ClawIn.setPosition(CIP);
                    ClawOut.setPosition(COP);
                    Pivot.setPosition(PVP);
                }
            });
        }
        thread.start();
    }

    /** TeleOP */

    public void intake(){
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_PREP);
        ClawIn.setPosition(hardwareClass.CLAW_IN_OPEN);
        Pivot.setPosition(hardwareClass.PIVOT_MAX_RIGHT);
    }

    public void transfer_P(){
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_DOWN);
        wait(20);
        ClawIn.setPosition(hardwareClass.CLAW_IN_ADJUST);
        ClawOut.setPosition(hardwareClass.CLAW_OUT_OPEN);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_TAKE);
        wait(100);
        Pivot.setPosition(hardwareClass.PIVOT_MAX_RIGHT);
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_UP);
        wait(200);
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_PREP);
        wait(200);
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_UP);
    }

    public void camIn(){
        hardwareClass.Cam.setPosition(hardwareClass.CAM_IN);
    }

    public void camOut(){
        hardwareClass.Cam.setPosition(hardwareClass.CAM_OUT);
    }

    public void transfer(){
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_DOWN);
        wait(80);
        ClawIn.setPosition(hardwareClass.CLAW_IN_CLOSED);
        ClawOut.setPosition(hardwareClass.CLAW_OUT_OPEN);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_TAKE);
        wait(100);
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_UP);
        //adjust();
    }

    public void transferShort(){
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_DOWN);
        wait(80);
        ClawIn.setPosition(hardwareClass.CLAW_IN_CLOSED);
        ClawOut.setPosition(hardwareClass.CLAW_OUT_OPEN);
        wait(100);
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_UP);
        wait(100);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_TAKE);
        wait(100);
        //adjust();
    }

    public void transferTO(){
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_DOWN - 0.02);
        wait(20);
        ClawIn.setPosition(hardwareClass.CLAW_IN_CLOSED);
        ClawOut.setPosition(hardwareClass.CLAW_OUT_OPEN);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_TAKE);
        wait(100);
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_UP);
        //adjust();
    }

    public void outtakeDown(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_OPEN);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_TAKE);
    }

    public void outtakeVERYDown(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_OPEN);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_TAKE_LOW);
    }

    public void transferSpec(){
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_DOWN);
        wait(100);
        ClawIn.setPosition(hardwareClass.CLAW_IN_ADJUST);
        wait(100);
        ClawOut.setPosition(hardwareClass.CLAW_OUT_OPEN);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_TAKE);
        wait(100);
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_UP);
        wait(100);
        //adjust();
    }

    public void align(){
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_DOWN);
        wait(40);
        double pos = Pivot.getPosition();
        Pivot.setPosition(pos + 0.2);
        wait(50);
        Pivot.setPosition(pos);
        wait(70);
    }

    public void alignHigh(){
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_DOWN - 0.06);
        wait(200);
        double pos = Pivot.getPosition();
        Pivot.setPosition(pos + 0.2);
        wait(70);
        Pivot.setPosition(pos);
        wait(100);
    }

    public void outtake(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_SPEC);
        wait(100);
        ClawIn.setPosition(hardwareClass.CLAW_IN_OPEN);
        wait(100);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_PLACE - 0.05);
    }

    public void outtakeTransf(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_CLOSED);
        wait(100);
        ClawIn.setPosition(hardwareClass.CLAW_IN_OPEN);
        wait(100);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_PLACE);
    }

    public void outtakeS(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_CLOSED);
        wait(100);
        ClawIn.setPosition(hardwareClass.CLAW_IN_OPEN);
        wait(100);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_HIGH_VERT);
    }

    public void outtakeSpec(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_SPEC);
        wait(170);
        ClawIn.setPosition(hardwareClass.CLAW_IN_OPEN);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_SPEC);
    }

    public void see(){
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_SEE);
    }

    public void placeInBasket(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_OPEN);
        wait(100);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_VERTICAL);
    }

    public void placeInBasket_S(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_OPEN - 0.09);
        wait(100);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_VERTICAL);
    }

    public void intakeSpec(){
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_PREP);
        ClawIn.setPosition(hardwareClass.CLAW_IN_OPEN);
        Pivot.setPosition(hardwareClass.PIVOT_MAX_RIGHT);
    }

    public void prepOuttake(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_CLOSED);
        wait(200);
        ClawIn.setPosition(hardwareClass.CLAW_IN_OPEN);
        wait(200);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_VERTICAL);
    }

    public void releaseSpecimen(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_OPEN);
    }

    public void rotatePivot(double target){
        Pivot.setPosition(target);
    }

    public void closeSpecimen(){
        Specimen.setPosition(hardwareClass.SPECIMEN_CLOSED);
    }

    public void openSpecimen(){
        Specimen.setPosition(hardwareClass.SPECIMEN_OPEN);
    }


    public void pickUpSample(){
        ClawIn.setPosition(hardwareClass.CLAW_IN_CLOSED);
        wait(200);
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_PREP);
        Pivot.setPosition(hardwareClass.PIVOT_MAX_LEFT);
    }

    public void specArmOpen(){
        SpecClaw.setPosition(hardwareClass.SPEC_CLAW_OPEN);
    }
    public void specArmClose(){
        SpecClaw.setPosition(hardwareClass.SPEC_CLAW_CLOSE);
    }


    public void specArmUpFar(){
        SpecClaw.setPosition(hardwareClass.SPEC_CLAW_CLOSE);
    }

    public void specClawOpen(){
        SpecClaw.setPosition(hardwareClass.SPEC_CLAW_OPEN);
    }

    public void specClawClose(){

    }


    public void adjust_P(){
        Pivot.setPosition(hardwareClass.PIVOT_MAX_LEFT);
        wait(500);
        ClawIn.setPosition(hardwareClass.CLAW_IN_CLOSED);
        Pivot.setPosition(hardwareClass.PIVOT_MAX_RIGHT);
    }

    public void adjust(){
        ClawIn.setPosition(hardwareClass.CLAW_IN_ADJUST);
        Pivot.setPosition(hardwareClass.PIVOT_MAX_LEFT);
        wait(500);
        ClawIn.setPosition(hardwareClass.CLAW_IN_CLOSED);
        Pivot.setPosition(hardwareClass.PIVOT_MAX_RIGHT);
    }

    /** Fine state */

    public void intakePrep(double rot){
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_PREP);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_TAKE);
        ClawIn.setPosition(hardwareClass.CLAW_IN_OPEN);
        Pivot.setPosition(rot);
    }

    public void intakeDown(double rot){
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_DOWN);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_TAKE);
        ClawIn.setPosition(hardwareClass.CLAW_IN_OPEN);
        Pivot.setPosition(rot);
    }

    public void intakeDown_noPivot(){
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_DOWN);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_TAKE);
        ClawIn.setPosition(hardwareClass.CLAW_IN_OPEN);
    }

    public void intakeTake(){
        ClawIn.setPosition(hardwareClass.CLAW_IN_ADJUST);
    }

    public void intakeUp(){
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_UP);
    }

    public void intakeAdjust(){
        ClawIn.setPosition(hardwareClass.CLAW_IN_CLOSED);
        Pivot.setPosition(hardwareClass.PIVOT_MAX_RIGHT);
    }

    public void transferClose(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_CLOSED);
    }

    public void transferGive(){
        ClawIn.setPosition(hardwareClass.CLAW_IN_OPEN);
    }

    public void outtakePut(){
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_PLACE - 0.05);
    }

    public void outtakePlace(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_OPEN);
    }

    public void outtakeReturn(){
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_VERTICAL);
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
