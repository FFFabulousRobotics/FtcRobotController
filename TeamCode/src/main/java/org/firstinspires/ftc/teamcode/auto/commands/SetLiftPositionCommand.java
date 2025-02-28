package org.firstinspires.ftc.teamcode.auto.commands;

import org.firstinspires.ftc.teamcode.auto.ContinuousCommand;
import org.firstinspires.ftc.teamcode.hardware.PIDController;
import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;

public class SetLiftPositionCommand extends ContinuousCommand {
    RobotAuto robotAuto;
    RobotTop robotTop;
    int targetPos;
    int currentPos;
    int error;
    PIDController liftPID = new PIDController();

    public SetLiftPositionCommand(RobotAuto robotAuto, int position) {
        this.robotAuto = robotAuto;
        this.targetPos = position;
        this.robotTop = robotAuto.robotTop;
    }

    @Override
    public void iterate() {
        double power;
//        if (targetPos >= 50 || currentPos >= 100) {
//            currentPos = robotTop.getLiftPosition();
//            error = targetPos - currentPos;
//            power = liftPID.updatePID(error);
//        } else {
//            power = 0;
//        }
        currentPos = robotTop.getLiftPosition();
        error = targetPos - currentPos;
        power = liftPID.updatePID(error);
        robotTop.setLiftPower(power);
    }

    @Override
    public boolean hasNext() {
        return Math.abs(error) >= 100;
    }

    @Override
    public void init() {
        liftPID.reset();
        liftPID.setPIDArguments(0.003, 0.004, 0.0005);
        liftPID.setIntegralSumLimit(100);
    }
}
