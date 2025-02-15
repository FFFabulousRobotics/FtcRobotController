package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController{
    protected ElapsedTime timer = new ElapsedTime();
    protected double previousError = 0;
    protected double porportion = 0, integral = 0, derivative = 0;
    protected double previousFilterEstimate = 0;
    protected double currentFilterEstimate = 0;

    public double Kp, Ki, Kd;
    public double integralSumLimit = 0.25;
    public double filterGain = 0.8;

    //for telemetry
    public double pOut,iOut,dOut;

    public void reset(){
        timer.reset();
        integral = 0;
    }

    public double updatePID(double error){
        // 滤除噪声，减少对D控制干扰
        currentFilterEstimate = (filterGain * previousFilterEstimate) + (1- filterGain) * (error - previousError);
        derivative = currentFilterEstimate / timer.seconds();

        // 设置积分上限防止溢出
        integral = Math.min(integral + (error * timer.seconds()), integralSumLimit);

        timer.reset();
        previousFilterEstimate = currentFilterEstimate;
        previousError = error;

        pOut = Kp * error;
        iOut = Ki * integral;
        dOut = Kd * derivative;

        double out = (Kp * error) + (Ki * integral) + (Kd * derivative);
        return out;
    }

    public void setPIDArguments(double kp, double ki, double kd){
        this.Kp = kp;
        this.Ki = ki;
        this.Kd = kd;
    }

    public double[] getPIDArguments(){
        return new double[]{Kp, Ki, Kd};
    }

    public double getFilterGain() {
        return filterGain;
    }

    public void setFilterGain(double filterGain) {
        this.filterGain = filterGain;
    }

    public double getIntegralSumLimit() {
        return integralSumLimit;
    }

    public void setIntegralSumLimit(double integralSumLimit) {
        this.integralSumLimit = integralSumLimit;
    }
}
