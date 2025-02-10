package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Interface for localization methods.
 */
public interface Localizer {

    void setPose(Pose2d pose);

    /**
     * Returns the current pose estimate.
     * NOTE: Does not update the pose estimate;
     * you must call update() to update the pose estimate.
     * @return the Localizer's current pose
     */
    Pose2d getPose();

    /**
     * Updates the Localizer's pose estimate.
     * @return the Localizer's current velocity estimate
     */
    PoseVelocity2d update();

    /**
     * A default implementation of the Localizer interface using the SparkFun OTOS sensor.
     * <p>
     * This implementation assumes that the SparkFunOTOS sensor is configured to use
     * linear units of inches and angular units of degrees. When converting to Road Runner's
     * Pose2d, the heading is converted from degrees to radians.
     */
    class SparkFunOtosLocalizer implements Localizer {
        private SparkFunOTOS otos;
        private Pose2d currentPose;

        /**
         * Constructs a SparkFunOtosLocalizer with the given sensor.
         * You may add additional sensor configuration here if needed.
         *
         * @param sensor a pre-initialized SparkFunOTOS instance
         */
        public SparkFunOtosLocalizer(SparkFunOTOS sensor) {
            this.otos = sensor;
            // Initialize current pose to origin
            this.currentPose = new Pose2d(0, 0, 0);
        }

        @Override
        public void setPose(Pose2d pose) {
            this.currentPose = pose;
            // Convert Road Runner pose to sensor's Pose2D (assuming sensor uses inches and degrees)
            SparkFunOTOS.Pose2D sensorPose = new SparkFunOTOS.Pose2D(
                    pose.position.x,
                    pose.position.y,
                    Math.toDegrees(pose.heading.real)
            );
            otos.setPosition(sensorPose);
        }

        @Override
        public Pose2d getPose() {
            // Retrieve sensor pose; sensor returns values in inches and degrees
            SparkFunOTOS.Pose2D sensorPose = otos.getPosition();
            // Convert heading from degrees to radians for Road Runner
            return new Pose2d(
                    sensorPose.x,
                    sensorPose.y,
                    Math.toRadians(sensorPose.h)
            );
        }

        @Override
        public PoseVelocity2d update() {
            // In this simple implementation, we update the current pose by reading from the sensor
            // and return a zero velocity estimate.
            Pose2d newPose = getPose();
            this.currentPose = newPose;
            return new PoseVelocity2d(new Vector2d(this.currentPose.position.x,this.currentPose.position.y)
                    ,this.currentPose.heading.real);
        }
    }
}

