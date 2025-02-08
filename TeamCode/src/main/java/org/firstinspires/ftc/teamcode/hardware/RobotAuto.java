package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;   


import com.qualcomm.robotcore.util.Range;

@SuppressWarnings(value = "unused")
public class RobotAuto {
    LinearOpMode opMode;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    RobotChassis robotChassis;
    RobotTop robotTop;
    private double headingError = 0;
    @SuppressWarnings("FieldCanBeLocal")
    private double targetHeading = 0;
    private double previousHeading = 0;
    private double deltaHeading = 0;
    @SuppressWarnings("FieldMayBeFinal")
    private double driveSpeed = 0;
    private double turnSpeed = 0;
     private DistanceSensor sensorDistance;    

    IMU imu;
    SparkFunOTOS otos;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    public RobotAuto(LinearOpMode opMode) {
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        this.robotChassis = new RobotChassis(opMode);
        this.robotTop = new RobotTop(opMode);
        otos = opMode.hardwareMap.get(SparkFunOTOS.class, "otos");
        configureOtos();
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        robotChassis.setTargetPosition(new int[]{0, 0, 0, 0});
    }

    static final double COUNTS_PER_MOTOR_REV = 560;

    // gearing UP (will affect the direction of wheel rotation) < 1 < gearing DOWN
    // eg. for a 12-tooth driving a 24-tooth, the value is 24/12=2.0
    static final double DRIVE_GEAR_REDUCTION = (double) 3 / 5;

    // wheel diameter in inches
    static final double WHEEL_DIAMETER_INCHES = 2.55;

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double STRIFE_REDUCTION = 1.25;

    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable
    static final double P_STRAFE_GAIN = 0.03;   // Strafe Speed Control "Gain".
    static final double P_TURN_GAIN = 0.1;     // Larger is more responsive, but also less stable
    static final double D_TURN_GAIN = -0.1;
    static final double HEADING_THRESHOLD = 0.5;

    public double getSteeringCorrection(double desiredHeading, double proportionalGain, double dGain) {

        // Determine the heading current error
        headingError = desiredHeading - getHeading();
        deltaHeading = getHeading() - previousHeading;
        if(Math.abs(deltaHeading) > 45) deltaHeading = 0;


        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        previousHeading = getHeading();
        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain + deltaHeading * dGain, -1, 1);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {

        // Determine the heading current error
        headingError = desiredHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Read the robot heading directly from the IMU.
     *
     * @return The heading of the robot in degrees.
     */
    public double getHeading() {
        return getHeading(AngleUnit.DEGREES);
    }

    /**
     * read the Robot heading directly from the IMU
     *
     * @param unit The desired angle unit (degrees or radians)
     * @return The heading of the robot in desired units.
     */
    public double getHeading(AngleUnit unit) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Yaw/Pitch/Roll", orientation.toString());
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
        return orientation.getYaw(unit);
    }

    public double getREVdistance() {
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
        return sensorDistance.getDistance(DistanceUnit.INCH);
    }


    
    public RobotAuto driveStraight(double maxDriveSpeed,
                                   double distance,
                                   double heading) {
        robotChassis.resetEncoder();

        // Ensure that the OpMode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            setStraightTargetPosition(moveCounts);

            robotChassis.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            robotChassis.driveRobot(maxDriveSpeed, 0, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() && robotChassis.isAllBusy()) {

                // Determine required steering to keep on heading
                double turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                robotChassis.driveRobot(maxDriveSpeed, 0, -turnSpeed);
//                telemetry.addData("x", "%4.2f, %4.2f, %4.2f, %4.2f, %4d", maxDriveSpeed, distance, heading, turnSpeed, moveCounts);
//                telemetry.addData("target", robotChassis.getTargetPosition()[0]);
//                telemetry.addData("target", robotChassis.getTargetPosition()[1]);
//                telemetry.addData("target", robotChassis.getTargetPosition()[2]);
//                telemetry.addData("target", robotChassis.getTargetPosition()[3]);
//                telemetry.addData("encoder", robotChassis.getCurrentPosition()[0]);
//                telemetry.addData("encoder", robotChassis.getCurrentPosition()[1]);
//                telemetry.addData("encoder", robotChassis.getCurrentPosition()[2]);
//                telemetry.addData("encoder", robotChassis.getCurrentPosition()[3]);
//                telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            robotChassis.stopMotor();
            robotChassis.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }

    public RobotAuto driveStrafe(double maxDriveSpeed,
                                 double distance,
                                 double heading
    ) {
        robotChassis.resetEncoder();

        // Ensure that the OpMode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH * STRIFE_REDUCTION);
            setStrafeTargetPosition(moveCounts);

            robotChassis.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            robotChassis.driveRobot(0, maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() && robotChassis.isAllBusy()) {

                // Determine required steering to keep on heading
                double turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                robotChassis.driveRobot(0, maxDriveSpeed, -turnSpeed);
                //                telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f, %4d",maxDriveSpeed,distance,heading,turnSpeed,moveCounts);
                telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            robotChassis.stopMotor();
            robotChassis.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }

    public void setStraightTargetPosition(int moveCounts) {
        int[] motorPos = robotChassis.getCurrentPosition();
        motorPos[0] = motorPos[0] + moveCounts;
        motorPos[1] = motorPos[1] + moveCounts;
        motorPos[2] = motorPos[2] + moveCounts;
        motorPos[3] = motorPos[3] + moveCounts;
        robotChassis.setTargetPosition(motorPos);
    }

    public void setStrafeTargetPosition(int moveCounts) {
        int[] motorPos = robotChassis.getCurrentPosition();
        motorPos[0] = motorPos[0] + moveCounts;
        motorPos[1] = motorPos[1] - moveCounts;
        motorPos[2] = motorPos[2] - moveCounts;
        motorPos[3] = motorPos[3] + moveCounts;
        robotChassis.setTargetPosition(motorPos);
    }

    public RobotAuto turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        //绝对值的问题？
        while (opMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {


            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            robotChassis.driveRobot(0, 0, -turnSpeed);
            //            telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f",maxTurnSpeed,turnSpeed,heading,getHeading());
            telemetry.update();
        }

        // Stop all motion;
        robotChassis.stopMotor();
        return this;
    }

    private void configureOtos() {
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);

        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        otos.calibrateImu();

        otos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

    public SparkFunOTOS.Pose2D getPosition() {
        return otos.getPosition();
    }

    private double calcDistance(double x,double y){
        return Math.sqrt(x * x + y * y);
    }

    public RobotAuto gotoPos(double desiredX, double desiredY){
        double currentX,currentY,dx,dy,angle,unitX,unitY,deltaDistance;
        double kp;
        SparkFunOTOS.Pose2D pose = getPosition();
        currentX = pose.x; currentY = pose.y;
        dx = desiredX-currentX;
        dy = desiredY-currentY;

        while (calcDistance(dx,dy) > 1){
            pose = getPosition();
            currentX = pose.x; currentY = pose.y;
            dx = desiredX-currentX;
            dy = desiredY-currentY;
            telemetry.addData("dx",dx);
            telemetry.addData("dy",dy);
            angle = Math.atan2(dy,dx);
            unitY = Math.sin(angle);
            unitX = Math.cos(angle);
            deltaDistance = calcDistance(dx,dy);
            kp = deltaDistance * 0.06;
            telemetry.addData("ux",unitX);
            telemetry.addData("uy",unitY);
            if(kp > 1) kp = 1;
            robotChassis.absoluteDriveRobot(-unitY * kp,unitX * kp,0);
        }

        return this;
    }

    public RobotAuto forward(double d) {
        return driveStraight(0.6, -d, getHeading());
    }

    public RobotAuto fastForward(double d) {
        return driveStraight(0.9, -d, getHeading());
    }


    public RobotAuto backward(double d) {
        return driveStraight(0.6, d, getHeading());
    }

    public RobotAuto fastBackward(double d) {
        return driveStraight(0.9, d, getHeading());
    }

    public RobotAuto rightShift(double d) {
        return driveStrafe(0.9, -d, getHeading());
    }

    public RobotAuto leftShift(double d) {
        return driveStrafe(0.9, d, getHeading());
    }

    public RobotAuto spin(double h) {
        return turnToHeading(0.6, h);
    }

    public RobotAuto fastSpin(double h) {
        return turnToHeading(0.9, h);
    }

    public RobotAuto sleep(long milliseconds) {
        opMode.sleep(milliseconds);
        return this;
    }

    protected double[] SpinVector(double[] vector, double angle) {
        double x = vector[0] * Math.cos(Math.toRadians(angle)) - vector[1] * Math.sin(Math.toRadians(angle));
        double y = vector[0] * Math.sin(Math.toRadians(angle)) + vector[1] * Math.cos(Math.toRadians(angle));
        return new double[]{x, y};
    }

    protected double[] getDisplacement(double[] CurrentPos, double[] DesiredPos) {
        double CurrentX = CurrentPos[0];
        double CurrentY = CurrentPos[1];
        double CurrentHeading = CurrentPos[2];
        double DesiredX = DesiredPos[0];
        double DesiredY = DesiredPos[1];
        double DesiredHeading = DesiredPos[2];
        double[] Displacement = {DesiredX - CurrentX, DesiredY - CurrentY, CurrentHeading};
        Displacement = SpinVector(Displacement, -CurrentHeading);
        return Displacement;
    }

    /**
     * Go to the position given (track only include left/right and forward/backward)
     * Go forward/backward first,then move left/right.
     *
     * @param CurrentPos The current position.(position{axial,lateral,heading})
     * @param DesiredPos The desired position.(position{axial,lateral,heading})
     * @return RobotHardware class.
     */
    public RobotAuto gotoPosition(double[] CurrentPos, double[] DesiredPos) {
        double[] Displacement = getDisplacement(CurrentPos, DesiredPos);
        double DesiredHeading = DesiredPos[2];
        return fastForward(-Displacement[0])
                .leftShift(-Displacement[1])
                .fastSpin(DesiredHeading);
    }

    public RobotAuto gotoPosition(double x, double y, double h) {
        SparkFunOTOS.Pose2D CurrentPos = getPosition();
        double[] DesiredPos = {x, y, h};
        return gotoPosition(new double[]{CurrentPos.x, CurrentPos.y, CurrentPos.h}, DesiredPos);
    }

    /**
     * Go to the position given (track only include left/right and forward/backward)
     * Move left/right first,then go forward/backward.
     *
     * @param CurrentPos The current position.(position{axial,lateral,heading})
     * @param DesiredPos The desired position.(position{axial,lateral,heading})
     * @return RobotHardware class.
     */
    public RobotAuto gotoPosition2(double[] CurrentPos, double[] DesiredPos) {
        double[] Displacement = getDisplacement(CurrentPos, DesiredPos);
        double DesiredHeading = DesiredPos[2];
        return leftShift(Displacement[1])
                .fastForward(Displacement[0])
                .fastSpin(DesiredHeading);
    }

    public RobotAuto gotoPosition2(double x, double y, double h) {
        SparkFunOTOS.Pose2D CurrentPos = getPosition();
        double[] DesiredPos = {x, y, h};
        return gotoPosition2(new double[]{CurrentPos.x, CurrentPos.y, CurrentPos.h}, DesiredPos);
    }

    public RobotAuto setLiftPower(double power) {
        robotTop.setLiftPower(power);
        return this;
    }

    public RobotAuto setTurnPosition(double p){
        robotTop.setTurnPosition(p);
        return this;
    }

    public RobotAuto grab() {
        robotTop.setLiftServoPosition(0.6);
        return this;
    }

    public RobotAuto release() {
        robotTop.setLiftServoPosition(0.2);
        return this;
    }

    public RobotAuto topOut() {
        robotTop.setTopServoPosition(0.66);
        return this;
    }
    public RobotAuto topBack(){
        robotTop.setTopServoPosition(0.03);
        return this;
    }
}
