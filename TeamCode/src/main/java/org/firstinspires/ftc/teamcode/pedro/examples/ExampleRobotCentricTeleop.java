package org.firstinspires.ftc.teamcode.pedro.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.Threads.Extendo;
import org.firstinspires.ftc.teamcode.Threads.Holonomic;
import org.firstinspires.ftc.teamcode.Threads.Servos;
import org.firstinspires.ftc.teamcode.Threads.Slides;
import org.firstinspires.ftc.teamcode.pedro.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedro.constants.LConstants;

/**
 * This is an example teleop that showcases movement and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

//@TeleOp(name = "HAN SOLO", group = "Examples")
public class ExampleRobotCentricTeleop extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    Slides slides = null;
    Servos servos = null;
    Extendo extendo = null;
    HardwareClass hardwareClass = null;
    Holonomic holonomic = null;

    int ExtendoPosition = 0;
    int pivotType = 1;


    // TeleOp variables

    String Style = "Drive";

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        Constants.setConstants(FConstants.class,LConstants.class);
        slides = Slides.getInstance(hardwareMap, telemetry);
        servos = Servos.getInstance(hardwareMap , telemetry);
        extendo = Extendo.getInstance(hardwareMap,telemetry);
        hardwareClass = HardwareClass.getInstance(hardwareMap);
        holonomic = Holonomic.getInstance(hardwareMap , gamepad1);

        //Setup Individual Motors
        hardwareClass.Extendo.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.LS.setDirection(DcMotorSimple.Direction.REVERSE);

        hardwareClass.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.BL.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        if(!slides.getStatus()){
            slides.setup();
        }

        if(!extendo.getStatus()){
            extendo.setup();
        }

        if(!holonomic.getStatus()){
            holonomic.start();
        }

        follower.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        switch (Style){
            case "Drive" : {
                //Intake mode
                if(gamepad1.right_bumper){
                    servos.intake();
                    ExtendoPosition = (int) hardwareClass.CLOSE;
                    holonomic.decel();
                    servos.placeInBasket();
                    delay(100);
                    slides.setCoefs(0.003 ,0 ,0);
                    slides.goToPosition(hardwareClass.IN_ROBOT);
                    pivotType = 1;
                    Style = "Intake";
                }

                //Place in basket and return
                if(gamepad1.left_bumper){
                    servos.outtake();
                    delay(50);
                    slides.setCoefs(0.01,0 ,0);
                    slides.goToPosition(hardwareClass.HIGH_BASKET);
                }

                //Prep for high basket
                if(gamepad1.left_trigger > 0.5){
                    servos.placeInBasket();
                    delay(100);
                    slides.setCoefs(0.003 ,0 ,0);
                    slides.goToPosition(hardwareClass.IN_ROBOT);
                }

                if(gamepad1.right_trigger > 0.5){
                    servos.outtake();
                    delay(50);
                    slides.setCoefs(0.01,0 ,0);
                    slides.goToPosition(hardwareClass.LOW_BASKET);
                }

                if(gamepad1.x){
                    hardwareClass.Push.setPower(-1);
                }else if(gamepad1.a){
                    hardwareClass.Push.setPower(1);
                }else {
                    hardwareClass.Push.setPower(0);
                }

                if(gamepad1.dpad_down){
                    slides.setCoefs(0.03 ,0 ,0);
                    slides.goToPosition(-100);
                }

                if(gamepad1.dpad_up){
                    slides.setCoefs(0.02 ,0 ,0);
                    slides.goToPosition(hardwareClass.HIGH_BASKET);
                    servos.placeInBasket();
                }

                //Give colored sample to human player
                if(gamepad1.y){
                    extendo.goToPosition(hardwareClass.MAX);
                    servos.pickUpSample();
                }

                if(gamepad1.b){
                    slides.setCoefs(0.01 , 0 , 0);
                    slides.goToPosition(hardwareClass.IN_ROBOT);
                    extendo.goToPosition(-50);
                    hardwareClass.OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_TAKE);
                    Style = "Spec";
                }


                break;
            }
            case "Intake" : {
                if(gamepad1.right_trigger > 0.5 && ExtendoPosition < hardwareClass.MAX){
                    ExtendoPosition += 30;
                }else if(gamepad1.right_bumper && ExtendoPosition > hardwareClass.CLOSE){
                    ExtendoPosition -= 10;
                }
                extendo.goToPosition(ExtendoPosition);

                if(pivotType == 1){
                    servos.rotatePivot(hardwareClass.PIVOT_MAX_LEFT * gamepad1.left_trigger + hardwareClass.PIVOT_MAX_RIGHT);
                }

                if(gamepad1.a){
                    Style = "Drive";
                    holonomic.accel();
                    hardwareClass.IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_DOWN);
                    servos.wait(100);
                    servos.transfer();
                    delay(100);
                    extendo.goToPosition(hardwareClass.IN);
                    servos.adjust();
                }

                if(gamepad1.dpad_down){
                    extendo.resetMotor();
                }

                break;
            }
            case "Spec" : {

                //Intake mode
                if(gamepad1.right_bumper){
                    servos.intake();
                    ExtendoPosition = (int) hardwareClass.CLOSE;
                    holonomic.decel();
                    servos.placeInBasket();
                    delay(100);
                    slides.setCoefs(0.003 ,0 ,0);
                    slides.goToPosition(hardwareClass.IN_ROBOT);
                    pivotType = 1;
                    Style = "Intake";
                }

                //Take Specimen
                if(gamepad1.right_trigger == 1){
                    servos.placeInBasket();
                    slides.setCoefs(0.01 , 0 , 0);
                    slides.goToPosition(hardwareClass.IN_ROBOT);
                    hardwareClass.OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_TAKE);
                }

                //Place specimen
                if(gamepad1.left_bumper){
                    slides.setCoefs(0.01 , 0 , 0);
                    servos.outtakeSpec();
                    delay(100);
                    slides.goToPosition(hardwareClass.PREP_SPECIMEN);

                }

                if(gamepad1.x){
                    hardwareClass.Push.setPower(-1);
                }else if(gamepad1.a){
                    hardwareClass.Push.setPower(1);
                }else {
                    hardwareClass.Push.setPower(0);
                }

                if(gamepad1.dpad_down){
                    slides.setCoefs(0.03 ,0 ,0);
                    slides.goToPosition(-100);
                }

                if(gamepad1.dpad_up){
                    slides.setCoefs(0.02 ,0 ,0);
                    slides.goToPosition(hardwareClass.HIGH_BASKET);
                    servos.placeInBasket();
                }

                break;
            }
            default: break;
        }

        follower.update();

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }

    private void delay(int delay){
        try {
            Thread.sleep(delay);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}