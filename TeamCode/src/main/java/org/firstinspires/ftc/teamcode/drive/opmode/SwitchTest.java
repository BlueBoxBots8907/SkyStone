package org.firstinspires.ftc.teamcode.drive.opmode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

import org.firstinspires.ftc.teamcode.hardware.Switches;

@TeleOp (group = "test")
public class SwitchTest extends LinearOpMode {
    private boolean[] switchStates;
    @Override
    public void runOpMode() {
        Switches switches = new Switches(hardwareMap);
    while (!isStopRequested()){
        switchStates = switches.readSwitches();
        telemetry.addData("SwitchAllianceBlue State", switchStates[0]);
        telemetry.addData("SwitchStartLoading State", switchStates[1]);
        telemetry.addData("SwitchFoundationUs State", switchStates[2]);
        telemetry.addData("SwitchParkingBridge State", switchStates[3]);
        telemetry.addData("SwitchAllianceRed State", switchStates[4]);
        telemetry.addData("SwitchStartBuiding State", switchStates[5]);
        telemetry.addData("SwitchFoundationPartner State", switchStates[6]);
        telemetry.addData("SwitchParkingWall State", switchStates[7]);
        telemetry.update();
    }
    }
}
