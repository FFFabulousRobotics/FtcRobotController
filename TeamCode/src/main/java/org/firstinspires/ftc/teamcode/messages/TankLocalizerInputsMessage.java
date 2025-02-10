package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

import java.util.ArrayList;

public final class TankLocalizerInputsMessage {
    public long timestamp;
    public PositionVelocityPair[] left;
    public PositionVelocityPair[] right;

    public TankLocalizerInputsMessage(ArrayList<Object> left, ArrayList<Object> right) {
        this.timestamp = System.nanoTime();
        this.left = left.toArray(new PositionVelocityPair[0]);
        this.right = right.toArray(new PositionVelocityPair[0]);
    }
}
