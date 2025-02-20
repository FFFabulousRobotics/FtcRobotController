package org.firstinspires.ftc.teamcode.roadrunner.tuning.otos;
import  org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive;
@TeleOp
public class OTOSPositionOffsetTuner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
       RobotAuto robotAuto = new RobotAuto(this);
        telemetry.addLine("OTOS Position Offset Tuner");
        telemetry.addLine("Line the robot against the corner of two walls facing forward and Press START.");
        telemetry.addLine("Then rotate the robot exactly 180 degrees and press it back into the corner.");
        telemetry.addLine("Finally, copy the pose offset into line 42 of SparkFunOTOSDrive.");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            robotAuto.update();
            telemetry.addData("Heading (deg)",robotAuto.getPosition().h);
            if (Math.abs(robotAuto.getPosition().h) > 175) {
                telemetry.addData("X Offset", robotAuto.getPosition().x / 2);
                telemetry.addData("Y Offset", robotAuto.getPosition().y / 2);
            } else {
                telemetry.addLine("Rotate the robot 180 degrees and align it to the corner again.");
            }
            telemetry.update();
        }


    }
}
