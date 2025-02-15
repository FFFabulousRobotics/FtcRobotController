package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.PIDController;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;

@TeleOp(group = "Test")
public class LiftPIDTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        PIDController liftPID = new PIDController();
        RobotTop robotTop = new RobotTop(this);
        waitForStart();
        liftPID.reset();
        liftPID.setPIDArguments(0.007,0.005,0.0005);
        liftPID.setIntegralSumLimit(100);
        int targetPos = 500;
        int currentPos;
        int error;
        while(opModeIsActive()){
            currentPos = robotTop.getLiftPosition();
            error = targetPos - currentPos;
            double power = liftPID.updatePID(error);
            robotTop.setLiftPower(power);
            telemetry.addData("target",targetPos);
            telemetry.addData("current",currentPos);
            telemetry.addData("error",error);
            telemetry.addData("power",power);
            telemetry.addData("p",liftPID.pOut);
            telemetry.addData("i",liftPID.iOut);
            telemetry.addData("d",liftPID.dOut);
            telemetry.update();

            if(gamepad1.y){
                targetPos = 1000;
                liftPID.reset();
            }
            if(gamepad1.a){
                targetPos = 300;
                liftPID.reset();
            }
            sleep(50);
        }
    }
}
