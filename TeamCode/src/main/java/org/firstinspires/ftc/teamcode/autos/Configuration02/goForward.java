package org.firstinspires.ftc.teamcode.autos.Configuration02;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.hardware.Servos;
import org.firstinspires.ftc.teamcode.hardware.Switches;
import org.openftc.revextensions2.ExpansionHubServo;


@Autonomous(name= "go forward ", group="Sky autonomous")
public class goForward extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        Servos hooks = new Servos(hardwareMap);
        Switches switches = new Switches(hardwareMap);
        boolean[] switchStates = switches.readSwitches();
        ExpansionHubServo leftServo, rightServo;

        leftServo = hardwareMap.get(ExpansionHubServo.class, "FoundationServoLeft");
        rightServo = hardwareMap.get(ExpansionHubServo.class, "FoundationServoRight");
        rightServo.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(14)
                        .build()
                );
    }
}
