package org.firstinspires.ftc.teamcode.TeleOp.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Config
public class ColorSenzorTuning extends OpMode {

    public ColorRangeSensor sensor = null;
    public static String senzorName = "";

    @Override
    public void init(){
        this.sensor = hardwareMap.get(ColorRangeSensor.class , senzorName);
    }

    @Override
    public void loop() {
        telemetry.addLine(sensor.alpha() + " - alpha");
        telemetry.addLine(sensor.red() + " - red");
        telemetry.addLine(sensor.blue() + " - blue");
        telemetry.addLine(sensor.green() + " - green");
        telemetry.addLine(sensor.getDistance(DistanceUnit.MM) + " - distance(mm)");
        telemetry.update();
    }
}