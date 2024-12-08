package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(group = "Test")
public class ServoTest2 extends LinearOpMode {
    @Override
    public void runOpMode(){
        Servo armStretchServo = hardwareMap.get(Servo.class, "armStretch");
        Servo armTurnServo = hardwareMap.get(Servo.class, "armTurn");
        Servo armSpinXServo = hardwareMap.get(Servo.class, "armSpinX");
        Servo armSpinYServo = hardwareMap.get(Servo.class, "armSpinY");
        Servo armGrabServo = hardwareMap.get(Servo.class, "armGrab");
        Servo liftServo = hardwareMap.get(Servo.class, "liftServo");
        Servo containerServo = hardwareMap.get(Servo.class, "containerServo");
        Servo liftTopServo = hardwareMap.get(Servo.class, "liftTop");
        Servo[] servos = {armStretchServo, armTurnServo, armSpinXServo, armSpinYServo, armGrabServo, liftServo, containerServo, liftTopServo};
        String[] names = {"stretch", "turn", "spinX", "spinY", "grab","lift","container","top"};
        //stretch(1): [0, 0.3]
        //turn(3): [0.38-back, 0.71-out]
        //grab(5): [0-open, 0.53-close]
        //spinY(4): default: 0.07
        //spinX(2): default: 0.5

        waitForStart();
        int index = 0;
        boolean activate = false;
        Gamepad previousGamepad1 = new Gamepad();
        previousGamepad1.copy(gamepad1);
        double servoPos = 0;
        Servo servo = servos[index];

        while (opModeIsActive()){
            if(gamepad1.y){
                servoPos = Math.min(1, servoPos + 0.01);
            }if(gamepad1.a){
                servoPos = Math.max(0, servoPos - 0.01);
            }
            if(gamepad1.b && !previousGamepad1.b){
                index += 1;
                index = index % servos.length;
                servo = servos[index];
            }
            if(gamepad1.x && !previousGamepad1.x){
                activate = !activate;
            }
            if (activate) {
                servo.setPosition(servoPos);
            }

            telemetry.addData("Y","add position");
            telemetry.addData("A","minus position");
            telemetry.addData("X","activate");
            telemetry.addData("B","change servo");
            telemetry.addData("--------","-------");
            telemetry.addData("pos", servoPos);
            telemetry.addData("servo", names[index]);
            telemetry.addData("activated",activate);
            telemetry.update();

            previousGamepad1.copy(gamepad1);
            sleep(100);
        }
    }
}
