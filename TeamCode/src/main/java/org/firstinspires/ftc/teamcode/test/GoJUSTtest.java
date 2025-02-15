package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SampleAuto;
import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;
@Autonomous
public class GoJUSTtest extends LinearOpMode {
    public enum method  {BACK,OUT};
    public RobotAuto robotAuto = new RobotAuto(this);
    public RobotTop robotTop = new RobotTop(this);
    final double TURN_LOCK_POSITION = 0.62;


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        robotTop.setTurnPosition(TURN_LOCK_POSITION);
        GoAndUp(20,20,20);



    }




    public void GoAndUp(double x,double y,double h)
    {
        double currentX,currentY,dx,dy;
        SparkFunOTOS.Pose2D pose = robotAuto.getPosition();
        currentX = pose.x; currentY = pose.y;
        dx = x-currentX;
        dy = y-currentY;


        while(calcDistance(dx,dy) > 1)
        {
            FakePidUp(method.OUT);
            robotAuto.JustGo(x,y,h);
        }

    }

    private double calcDistance(double x,double y){
        return Math.sqrt(x * x + y * y);
    }


    public void FakePidUp(method state) {
        double a = robotTop.getLiftPosition();
        if(state == method.OUT){
            if (a < 1260) {
                a = robotTop.getLiftPosition();
                robotTop.setLiftPower(1);
            } else if (a >= 1260)
            {
                robotTop.setLiftPower(0.05);
                sleep(10);
                robotTop.setLiftPower(0);
                sleep(40);
            }
        } else if (state == method.BACK) {
            if (a >= 40) {
                a = robotTop.getLiftPosition();
                robotTop.setLiftPower(-1);
            }
            else if (a <= 40)
            {
                robotTop.setLiftPower(0);
            }

        }
    }
}
