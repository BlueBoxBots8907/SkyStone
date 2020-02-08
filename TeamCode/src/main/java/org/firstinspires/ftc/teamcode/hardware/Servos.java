package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.revextensions2.ExpansionHubServo;

public class Servos {
    private ExpansionHubServo leftFoundationServo, rightFoundationServo, armServo, power, clawServo, capstoneServo;
    // private CRServo clawServo;

    private final double LEFT_DOWN_POSITION = 0.2,
            LEFT_UP_POSITION = 0.75,
            RIGHT_DOWN_POSITION = 0.2,
            RIGHT_UP_POSITION = 0.75;

    public Servos(HardwareMap hardwareMap) {
        leftFoundationServo = hardwareMap.get(ExpansionHubServo.class, "FoundationServoLeft");
        rightFoundationServo = hardwareMap.get(ExpansionHubServo.class, "FoundationServoRight");
        rightFoundationServo.setDirection(Servo.Direction.REVERSE);
        armServo = hardwareMap.get(ExpansionHubServo.class, "armServo");
        clawServo = hardwareMap.get(ExpansionHubServo.class, "clawServo");
        power = hardwareMap.get(ExpansionHubServo.class, "power");
        capstoneServo = hardwareMap.get(ExpansionHubServo.class, "capstoneServo");
    }

    public void foundationUp() {
        leftFoundationServo.setPosition(LEFT_UP_POSITION);
        rightFoundationServo.setPosition(RIGHT_UP_POSITION);
    }

    public void foundationDown() {
        leftFoundationServo.setPosition(LEFT_DOWN_POSITION);
        rightFoundationServo.setPosition(RIGHT_DOWN_POSITION);
    }

    public void setArmPosition(double pos) {
        armServo.setPosition(pos);
    }
    public void initPower() {
        power.setPosition(1);
    }
    public void clawOpen() {
        clawServo.setPosition(0.35);
    }

    public void clawClose() {
        clawServo.setPosition(1);
    }
    public void clawHalfOpen() { clawServo.setPosition(0.6); }

    public void dropCapstone() {capstoneServo.setPosition(0.3);}
    public void liftCapstone() {capstoneServo.setPosition(9);}

}
