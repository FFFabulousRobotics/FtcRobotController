package org.firstinspires.ftc.teamcode.previousCode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotAuto;

@Disabled
@Autonomous(group = "Test")
public class AutoTest2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotAuto robotAuto = new RobotAuto(this);
        waitForStart();
        while (opModeIsActive()){
            if(gamepad2.a){
                robotAuto.forward(24);
            }
            if(gamepad2.y){
                robotAuto.forward(-24);
            }
            if(gamepad2.x){
                robotAuto.leftShift(24);
            }
            if(gamepad2.b){
                robotAuto.rightShift(24);
            }
            if(gamepad2.dpad_left){
                robotAuto.spin(90);
            }
            if(gamepad2.dpad_right){
                robotAuto.spin(-90);
            }
            if(gamepad2.dpad_up){
                robotAuto.spin(0);
            }
            if(gamepad2.dpad_down){
                robotAuto.spin(180);
            }
        }

    }
}