package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.Threads.ColorSenzorThr;
import org.firstinspires.ftc.teamcode.Threads.Holonomic;
import org.firstinspires.ftc.teamcode.Threads.Servos;
import org.firstinspires.ftc.teamcode.Threads.Slides;
import org.opencv.core.Mat;

/**
 * This opmode demonstrates how one can augment driver control by following Road Runner arbitrary
 * Road Runner trajectories at any time during teleop. This really isn't recommended at all. This is
 * not what Trajectories are meant for. A path follower is more suited for this scenario. This
 * sample primarily serves as a demo showcasing Road Runner's capabilities.
 * <p>
 * This bot starts in driver controlled mode by default. The player is able to drive the bot around
 * like any teleop opmode. However, if one of the select buttons are pressed, the bot will switch
 * to automatic control and run to specified location on its own.
 * <p>
 * If A is pressed, the bot will generate a splineTo() trajectory on the fly and follow it to
 * targetA (x: 45, y: 45, heading: 90deg).
 * <p>
 * If B is pressed, the bot will generate a lineTo() trajectory on the fly and follow it to
 * targetB (x: -15, y: 25, heading: whatever the heading is when you press B).
 * <p>
 * If Y is pressed, the bot will turn to face 45 degrees, no matter its position on the field.
 * <p>
 * Pressing X will cancel trajectory following and switch control to the driver. The bot will also
 * cede control to the driver once trajectory following is done.
 * <p>
 * The following may be a little off with this method as the trajectory follower and turn
 * function assume the bot starts at rest.
 * <p>
 * This sample utilizes the SampleMecanumDriveCancelable.java and TrajectorySequenceRunnerCancelable.java
 * classes. Please ensure that these files are copied into your own project.
 */
@TeleOp(name="TeleOp Solo", group = "Solo")
public class TeleOpAugmentedDriving extends LinearOpMode {
    // Define 2 states, drive control or automatic control

    //Holonomic chassy = null;
    Slides slides = null;
    Servos servos = null;
    ColorSenzorThr colorSensor = null;
     int drop = 0;
    HardwareClass hardwareClass = null;
    Holonomic holonomic = null;

    // TeleOp variables

    String Style = "Drive";

    int Slides_Jumped = 0, Drop = 0;
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector = new Vector2d(-60, -56);
    // The heading we want the bot to end on for targetA
    double targetAHeading = Math.toRadians(45);

    // The location we want the bot to automatically go to when we press the B button
    Vector2d targetBVector = new Vector2d(-15, 25);

    // The angle we want to align to when we press Y
    double targetAngle = Math.toRadians(45);

    Pose2d pose = null;

    int automatic = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        slides = Slides.getInstance(hardwareMap, telemetry);
        servos = Servos.getInstance(hardwareMap, telemetry);
        hardwareClass = HardwareClass.getInstance(hardwareMap);
        holonomic = Holonomic.getInstance(hardwareMap, gamepad1);
        colorSensor = ColorSenzorThr.getInstance(hardwareMap,telemetry);

        //Setup Individual Motors
        hardwareClass.LS.setDirection(DcMotorSimple.Direction.REVERSE);

        hardwareClass.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.BL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();


        if (!slides.getStatus()) {
            slides.setup();
        }

        if (!holonomic.getStatus()) {
            holonomic.start();
        }

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {


            switch (Style) {
                case "Drive": {
                    if (this.gamepad1.right_bumper) {
                            servos.ExtendoMax();
                            servos.SClawOut();
                            servos.IntakePreTake();
                            Style = "Intake";
                    }
                    break;
                }
                case "Intake": {
                    if (this.gamepad1.right_bumper) servos.ExtendoShort();

                    if (this.gamepad1.right_trigger > 0.15) {
                        servos.IntakeOut();
                        servos.BrushOn();
                    }
                    if (this.gamepad1.left_trigger > 0.15) {
                        servos.IntakePreTake();
                        servos.BrushOFF();
                    }
                    if (this.gamepad1.a) {
                        servos.SClawIn();
                        sleep(200);
                        servos.BrushOFF();
                        servos.IntakeIn();
                        servos.ExtendoMin();
                        Style = "Outtake";
                        servos.OutTakeTrans();
                        slides.nulpoz();
                    }

                    /*
                    if (colorSensor.getColor() == 1){
                        servos.SClawIn();
                        sleep(200);
                        servos.BrushOFF();
                        servos.IntakeIn();
                        servos.ExtendoMin();
                        Style = "Outtake";
                        servos.OutTakeTrans();
                        slides.nulpoz();
                    }
                     */

                    if (this.gamepad1.x) {
                        servos.BrushOFF();
                        servos.IntakeIn();
                        sleep(30);
                        servos.ExtendoMin();
                        Style = "Drive";
                    }

                    break;
                }


                case "Outtake": {
                    if (this.gamepad1.y) {
                        servos.ExtendoShort();
                        //sleep(150);
                        servos.SClawOut();
                        servos.ExtendoMin();
                        Style = "Drive";
                    }

                    if (this.gamepad1.left_bumper) {
                        servos.OutClawTake();
                        servos.SClawOut();
                        sleep(400);
                        slides.goToPosition(HardwareClass.HIGH_BASKET);
                        servos.OutTakeBasket();
                        Drop = 1;
                    }

                    if (this.gamepad1.left_trigger > 0.15 && Drop == 1) {
                        Drop = 0;
                        servos.OutClawOpen();
                        sleep(300);
                        servos.OutTakeRest();
                        sleep(500);
                        slides.nulpoz();
                        Style = "Drive";
                    }

                }

            }
        }
    }



    private void delay(int delay){
        try {
            Thread.sleep(delay);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}