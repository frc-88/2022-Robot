// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.coprocessor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class GameObject {
    public String name = "";
    public int count = 0;
    private double x = 0.0;
    private double y = 0.0;
    private double z = 0.0;
    public MessageTimer timer = new MessageTimer(500_000);

    public GameObject(String name)
    {
        this.name = name;
    }

    public void set(double x, double y, double z) {
        timer.reset();
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Pose2d get() {
        return new Pose2d(x, y, new Rotation2d());
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public boolean isValid() {
        return name.length() > 0 && timer.isActive();
    }
}
