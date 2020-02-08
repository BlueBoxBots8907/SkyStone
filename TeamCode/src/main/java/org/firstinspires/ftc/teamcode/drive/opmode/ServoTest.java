package org.firstinspires.ftc.teamcode.drive.opmode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.openftc.revextensions2.ExpansionHubServo;

import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (group = "test")
public class ServoTest extends LinearOpMode {
    private ExpansionHubServo leftServo, rightServo;

    @Override
    public void runOpMode() {
        leftServo = hardwareMap.get(ExpansionHubServo.class, "FoundationServoLeft");
        rightServo = hardwareMap.get(ExpansionHubServo.class, "FoundationServoRight");
        leftServo.setDirection(Servo.Direction.REVERSE);
    while (!isStopRequested()){
    leftServo.setPosition(0);
    rightServo.setPosition(0);
    sleep(1000);
        leftServo.setPosition(0.8);
        rightServo.setPosition(0.8);
        sleep(1000);
    }
    }
}
