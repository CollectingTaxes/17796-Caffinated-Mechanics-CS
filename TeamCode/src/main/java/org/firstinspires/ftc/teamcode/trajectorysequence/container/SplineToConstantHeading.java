package org.firstinspires.ftc.teamcode.trajectorysequence.container;

public class SplineToConstantHeading extends PathSegment {
    public volatile double x, y, endHeading;
    public SplineToConstantHeading(double x, double y, double endHeading) {
        this.x = x;
        this.y = y;
        this.endHeading = endHeading;
    }
}
