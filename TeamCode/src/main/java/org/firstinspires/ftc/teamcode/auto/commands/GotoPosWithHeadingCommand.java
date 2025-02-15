package org.firstinspires.ftc.teamcode.auto.commands;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.Command;
import org.firstinspires.ftc.teamcode.hardware.PIDController;
import org.firstinspires.ftc.teamcode.hardware.RobotAuto;

public class GotoPosWithHeadingCommand implements Command {
    RobotAuto robotAuto;
    double desiredX,desiredY,heading;
    double currentX,currentY,dx,dy,angle,unitX,unitY,deltaDistance;
    double headingError;
    double kp;
    double proportionalGain = 0.06;
    double P_TURN_GAIN = 0.05;
    SparkFunOTOS.Pose2D pose;
    PIDController pidControllerForDistance = new PIDController();
    PIDController pidControllerForHeading = new PIDController();

    public GotoPosWithHeadingCommand(RobotAuto robotAuto, double desiredX, double desiredY, double heading){
        this.robotAuto = robotAuto;
        this.desiredX = desiredX;
        this.desiredY = desiredY;
        this.heading = heading;
    }

    @Override
    public void iterate() {
        pose = robotAuto.getPosition();
        currentX = pose.x; currentY = pose.y;
        dx = desiredX-currentX;
        dy = desiredY-currentY;
        angle = Math.atan2(dy,dx);
        unitY = Math.sin(angle);
        unitX = Math.cos(angle);
        deltaDistance = robotAuto.calcDistance(dx,dy);

        double rate = pidControllerForDistance.updatePID(deltaDistance);
        if(rate >= 1) rate = 1;

        double turnSpeed = pidControllerForHeading.updatePID(robotAuto.getHeadingError(heading));

        // Clip the speed to the maximum permitted value.
        turnSpeed = Range.clip(turnSpeed, -0.6, 0.6);

        robotAuto.absoluteDriveRobot(-unitY * rate,unitX * rate, -turnSpeed);

        robotAuto.absoluteDriveRobot(-unitY * rate,unitX * rate, -turnSpeed);
    }

    @Override
    public boolean hasNext() {
        return robotAuto.calcDistance(dx,dy) > 0.5 || Math.abs(headingError) > 0.5;
    }

    @Override
    public void init() {
        pose = robotAuto.getPosition();
        currentX = pose.x; currentY = pose.y;
        dx = desiredX-currentX;
        dy = desiredY-currentY;

        pidControllerForDistance.setPIDArguments(proportionalGain,0,0);
        pidControllerForDistance.reset();

        pidControllerForHeading.setPIDArguments(P_TURN_GAIN,0,0);
        pidControllerForHeading.reset();
    }

    @Override
    public void finish() {
        robotAuto.stopMotor();
    }
}
