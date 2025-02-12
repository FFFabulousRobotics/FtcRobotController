package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sun.source.tree.WhileLoopTree;

import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;
import org.firstinspires.ftc.teamcode.hardware.RobotChassis;

@Autonomous
public class SampleAuto extends LinearOpMode {

    final int STRETCH_BACK_POSITION = 70;
    final int STRETCH_OUT_POSITION = 1500;
    final double SPIN_DEFAULT_POSITION_L = 1;
    final double SPIN_DEFAULT_POSITION_R = 0.464;
    final double SPIN_HOVERING_POSITION_L = 0.4;
    final double SPIN_HOVERING_POSITION_R = 1;
    final double SPIN_DOWN_POSITION = 0;
    final double TURN_BACK_POSITION = 0.5;
    final double TURN_LOCK_POSITION = 0.62;
    final double TURN_HOVERING_POSITION = 0.75;
    final double TURN_DOWN_POSITION = 0.85;
    final double GRAB_OPEN_POSITION = 0.4;
    final double GRAB_CLOSE_POSITION = 0.92;
    final double TOP_BACK = 0.03;
    final double TOP_OUT = 0.66;
    final private  double posD[] = {0,0,0};//投篮位置
    final private double posT[][] = {{0,0,0},//第一个sample夹取位置
                                    {0,0,0},//第二个夹取位置
                                    {0,0,0},//第三个夹取位置
                                    {0,0,0}};//停靠位置
    private RobotAuto robotAuto;
    private RobotTop robotTop;
    private RobotChassis robotChassis;
    private Pose2D target;

    @Override
    public void runOpMode() {

        robotAuto = new RobotAuto(this);
        robotTop = new RobotTop(this);
        robotChassis = new RobotChassis(this);

        target = robotAuto.getPosition();
        waitForStart();

        while (opModeIsActive()) {

            ThrowsSample();
            pickUpAndThrow(1);
            pickUpAndThrow(2);
            pickUpAndThrow(3);
            goCorrect(posT[3][0], posT[3][1], posT[3][2]);

            Pose2D a = robotAuto.getPosition();

            telemetry.addData("君所言甚是，然吾等FTC比赛，非惟机械较量，亦集智巧、程式与协作于一堂。\n" ,
                    "各队匠心独运，巧构器械，编制妙算，遇险不惧，技压群雄。\n" ,
                    "若君欲共襄盛举，速至官方报名之处，攀登科技之巅。\n");
            telemetry.addData("x", a.x);
            telemetry.addData("y", a.y);
            telemetry.addData("h", a.h);
            telemetry.update();
        }
    }

    public void goCorrect(double x,double y,double z) {
            robotAuto.gotoPos(x,y);
            robotAuto.turnToHeading(1,z);
    }
    public void pickUpAndThrow(int a)
    {
        goCorrect(posT[a-1][0],posT[a-1][1],posT[a-1][2]);
        SPINcontrol();
        ARMcontrol();
        robotAuto.grab();
        SPINcontrol();
        ARMcontrol();
        robotAuto.release();
        ThrowsSample();
    }

    public void ARMcontrol()
    {
        if (robotTop.getArmStretchPosition()==STRETCH_BACK_POSITION)
        {
            while(robotTop.getArmStretchPosition()<= STRETCH_OUT_POSITION)
            {robotTop.setStretchPower(0.9);}
        robotTop.setStretchPower(0);
        }
        if (robotTop.getArmStretchPosition()==STRETCH_OUT_POSITION)
        {
            while(robotTop.getArmStretchPosition() >= STRETCH_BACK_POSITION)
            {robotTop.setStretchPower(-0.9);}
            robotTop.setStretchPower(0);
        }
    }

    public void FakePidUp() {
        double a = robotTop.getLiftPosition();
        if (a < 1260) {
            a = robotTop.getLiftPosition();
            robotTop.setLiftPower(1);
        } else if (a >= 1260)
        {
            robotTop.setLiftPower(1);
            sleep(50);
        }
    }

    public void SPINcontrol()
    {
        if(robotTop.getArmLeftSpinPosition()==SPIN_HOVERING_POSITION_L)
        {
            robotTop.setArmLeftSpinPosition(SPIN_DEFAULT_POSITION_L);
            robotTop.setArmRightSpinPosition(SPIN_DEFAULT_POSITION_R);
        }
        else if(robotTop.getArmLeftSpinPosition()==SPIN_DEFAULT_POSITION_L)
        {
            robotTop.setArmLeftSpinPosition(SPIN_HOVERING_POSITION_L);
            robotTop.setArmRightSpinPosition(SPIN_HOVERING_POSITION_R);
        }
    }

    public void ThrowsSample()
    {
        goCorrect(posD[0],posD[1],posD[2]);
        SPINcontrol();
        while(robotTop.getLiftPosition() <= 1260)FakePidUp();
        robotAuto.topOut();
        while(robotTop.getTopServoPosition() <= TOP_OUT)FakePidUp();
        robotAuto.topBack();
        while(robotTop.getTopServoPosition() >= TOP_BACK)FakePidUp();
        robotTop.setLiftTargetPos(40);
        if(robotTop.getLiftPosition() == 40)
            SPINcontrol();
    }
}


