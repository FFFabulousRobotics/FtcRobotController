package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class WheelOffset extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BR");
        waitForStart();

        double k1=1,k2=0.8535,k3=0.9985,k4 = 0.849;

        frontLeftMotor.setPower(1*k1);
        backLeftMotor.setPower(1*k2);
        frontRightMotor.setPower(1*k3);
        backRightMotor.setPower(1*k4);
        boolean flag = true;
        while (opModeIsActive()){
            if(getRuntime() >= 3 && flag){
                int FLPos = frontLeftMotor.getCurrentPosition();
                int BLPos = backLeftMotor.getCurrentPosition();
                int FRPos = frontLeftMotor.getCurrentPosition();
                int BRPos = backRightMotor.getCurrentPosition();
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                telemetry.addData("FL", FLPos);
                telemetry.addData("BL", BLPos);
                telemetry.addData("FR", FRPos);
                telemetry.addData("BR", BRPos);
                double[] pos = {FLPos,BLPos,FRPos,BRPos};
                double[] rate = {1, pos[0]/pos[1], pos[0]/pos[2], pos[0]/pos[3]};
                telemetry.addData("rate origin", rate[0]);
                telemetry.addData("rate origin", rate[1]);
                telemetry.addData("rate origin", rate[2]);
                telemetry.addData("rate origin", rate[3]);
                double maxinum = Math.max(Math.max(rate[0],rate[1]),Math.max(rate[2],rate[3]));
                double coefficient = 1/maxinum;
                int t = 0;
                for (double each:rate
                     ) {
                    rate[t] = each * coefficient;
                    t += 1;
                }
                telemetry.addData("k1", rate[0]);
                telemetry.addData("k2", rate[1]);
                telemetry.addData("k3", rate[2]);
                telemetry.addData("k4", rate[3]);
                telemetry.update();
                flag = false;
            }
            sleep(50);
        }
    }
}
