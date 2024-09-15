package org.firstinspires.ftc.teamcode;

import android.content.res.Resources;

import com.qualcomm.robotcore.hardware.Servo;

public class RobotTop {
    TimedServo armServo;

    public RobotTop(TaskOpMode opMode) {
        this.armServo = new TimedServo(opMode.hardwareMap.get(Servo.class, Resources.getSystem().getString(R.string.armServo)));
    }

    public Task stretchArm() {
        return armServo.setPosition(1, 500);
    }

    public Task retractArm() {
        return armServo.setPosition(0, 500);
    }
}
