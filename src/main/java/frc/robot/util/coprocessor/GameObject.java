// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.coprocessor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class GameObject {
    private String name = "";
    private int index = 0;
    private double x = 0.0;
    private double y = 0.0;
    private double z = 0.0;
    private double yaw = 0.0;
    public MessageTimer timer = new MessageTimer(150_000);

    public GameObject(String name, int index)
    {
        this.name = name;
        this.index = index;
    }

    public void set(double x, double y, double z, double yaw) {
        timer.reset();
        this.x = x;
        this.y = y;
        this.z = z;
        this.yaw = yaw;
    }

    public Pose2d getPose() {
        return new Pose2d(x, y, new Rotation2d(yaw));
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

    public double getDistance() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    public double getYaw() {
        return yaw;
    }

    public String getName() {
        return name;
    }

    public int getIndex() {
        return index;
    }

    public boolean isValid() {
        return name.length() > 0 && timer.isActive();
    }
}
