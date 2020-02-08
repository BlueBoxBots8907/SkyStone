package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.hardware.Servos;

@Autonomous(name="do nothing", group = "autos")
public class DoNothing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
    telemetry.addData("doing nothing", true);
    telemetry.update();
    sleep(30000);
    }
}