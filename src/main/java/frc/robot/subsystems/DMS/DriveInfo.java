// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DMS;

/** Add your docs here. */
public class DriveInfo <T> {
public T FL;
public T FR;
public T RL;
public T RR;

public DriveInfo(T value){
    FL = value;
    FR = value;
    RL = value;
    RR = value;
    }

    public DriveInfo(T fl, T fr, T rl, T rr){
        FL = fl;
        FR = fr;
        RL = rl;
        RR = rr;
        }

}


