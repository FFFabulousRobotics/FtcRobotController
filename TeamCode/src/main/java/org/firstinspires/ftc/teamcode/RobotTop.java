package org.firstinspires.ftc.teamcode;

import android.content.res.Resources;

import com.qualcomm.robotcore.hardware.Servo;

public class RobotTop {
    TimedServo armServo;
    TaskOpMode opMode;

    public RobotTop(TaskOpMode opMode) {
        this.opMode = opMode;
        this.armServo = new TimedServo(opMode.hardwareMap.get(Servo.class, Resources.getSystem().getString(R.string.armServo)));
    }

    public void stretchArm() {
        opMode.scheduleTask(armServo.setPosition(1, 500));
    }

    public void retractArm() {
        opMode.scheduleTask(armServo.setPosition(0, 500));
    }
}
