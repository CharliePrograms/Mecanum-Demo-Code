package org.firstinspires.ftc.teamcode.utils;

public class Pose2d {
    public double x;
    public double y;
    public double heading; // in radians

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    @Override
    public String toString() {
        return String.format("(x=%.2f, y=%.2f, θ=%.2f rad)", x, y, heading);
    }
}