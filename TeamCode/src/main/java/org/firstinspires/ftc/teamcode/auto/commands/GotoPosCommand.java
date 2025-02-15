package org.firstinspires.ftc.teamcode.auto.commands;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.auto.Command;
import org.firstinspires.ftc.teamcode.hardware.RobotAuto;

public class GotoPosCommand implements Command {

    double desiredX,desiredY;
    RobotAuto robotAuto;
    double currentX,currentY,dx,dy,angle,unitX,unitY,deltaDistance;
    double kp;
    SparkFunOTOS.Pose2D pose;

    public GotoPosCommand(RobotAuto robotAuto, double desiredX, double desiredY){
        this.robotAuto = robotAuto;
        this.desiredX = desiredX;
        this.desiredY = desiredY;
    }
    @Override
    public void iterate() {
        // get the position error
        pose = robotAuto.getPosition();
        currentX = pose.x; currentY = pose.y;
        dx = desiredX-currentX;
        dy = desiredY-currentY;

        // change it into a unit length
        angle = Math.atan2(dy,dx);
        unitY = Math.sin(angle);
        unitX = Math.cos(angle);

        // P control
        deltaDistance = robotAuto.calcDistance(dx,dy);
        kp = deltaDistance * 0.06;
        if(kp > 1) kp = 1;
        robotAuto.absoluteDriveRobot(-unitY * kp,unitX * kp,0);
    }

    @Override
    public boolean hasNext() {
        return robotAuto.calcDistance(dx,dy) > 0.5;
    }

    @Override
    public void init() {
        SparkFunOTOS.Pose2D pose = robotAuto.getPosition();
        currentX = pose.x; currentY = pose.y;
        dx = desiredX-currentX;
        dy = desiredY-currentY;
    }

    @Override
    public void finish() {
        robotAuto.stopMotor();
    }
}
