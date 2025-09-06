package org.firstinspires.ftc.teamcode.TestAndNotes;

import androidx.annotation.NonNull;

public class RobotLocationRadians {
    // Store the robot's heading in radians (internal representation).
    // Why radians? Because Java math functions (sin, cos, tan, etc.) all use radians.
    double angleRadians;
    double x = 0;
    double y = 0;

    // Constructor: user provides an angle in DEGREES,
    // we immediately convert it into RADIANS for storage.
    // Conversion formula: radians = degrees × (π / 180)
    public RobotLocationRadians(double angleDegrees){
        this.angleRadians = Math.toRadians(angleDegrees);
    }

    // Get the robot's heading, normalized and converted back to DEGREES
    public double getHeading(){
        double angle = this.angleRadians;

        // --- Normalization Step ---
        // A circle in radians = 2π (~6.283), half circle = π (~3.141)
        // We want to wrap the angle so it always stays in [-π, π]
        // That corresponds to [-180°, 180°].

        // If the angle is bigger than π (180°),
        // subtract 2π (360°) until it’s back in range.
        while(angle > Math.PI){
            angle -= 2 * Math.PI;
        }

        // If the angle is smaller than -π (-180°),
        // add 2π (360°) until it’s back in range.
        while(angle < -Math.PI){
            angle += 2 * Math.PI;
        }

        // Finally, convert the normalized radians back into DEGREES for readability
        // Conversion formula: degrees = radians × (180 / π)
        return Math.toDegrees(angle);
    }

    @NonNull
    @Override
    public String toString(){
        // Print the raw (not normalized) angle in radians
        // Example: after several turns, could be much larger than 2π
        return "RobotLocationRadians: angle (" + angleRadians + ")";
    }

    // Turn the robot by a given amount of DEGREES
    // Convert the change to radians, then add it to the stored heading
    // Conversion formula: radians = degrees × (π / 180)
    public void turn(double angleChangeDegrees){
        angleRadians += Math.toRadians(angleChangeDegrees);
    }

    // Set the robot's heading directly, given in DEGREES
    // Again, convert to radians for storage
    public void setAngle(double angleDegrees){
        this.angleRadians = Math.toRadians(angleDegrees);
    }
    public double getAngle(){
        return Math.toDegrees(this.angleRadians);
    }

    public double getX(){
        return this.x;
    }
    public void setX(double set){
        this.x = set;
    }
    public void changeX(double change){
        this.x += change;
    }

    public double getY(){
        return this.y;
    }
    public void setY(double set){
        this.y = set;
    }
    public void changeY(double change){
        this.y += change;
    }
}
