/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.chargerrobotics.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.chargerrobotics.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
	/**
	 * Creates a new LimelightSubsystem.
	 */
	private NetworkTable table;
	private NetworkTableEntry tx, ty, tv, ts, leds, camMode, pip;
	private boolean isRunning;
	private static LimelightSubsystem instance;

	private double x, y, v, s;

	public LimelightSubsystem() {
		table = NetworkTableInstance.getDefault().getTable("limelight");
		pip = table.getEntry("getpipe");
		tx = table.getEntry("tx");
		ty = table.getEntry("ty");
		tv = table.getEntry("tv");
		ts = table.getEntry("ts");
		leds = table.getEntry("ledMode");
		camMode = table.getEntry("camMode");
	}

	public static LimelightSubsystem getInstance() {
		if (instance == null) {
			instance = new LimelightSubsystem();
			CommandScheduler.getInstance().registerSubsystem(instance);
		}
		return instance;
	}

	public void setLEDStatus(boolean enabled) {
		leds.setDouble(enabled ? 0.0 : 1.0);
		camMode.setNumber(enabled ? 0 : 1);
	}

	// Placeholder values for coordinates of markers, in inches (should probably put
	// these in constants but don't want to screw up anything)
	// Looking down on the field with the initiation line to the south, the markers  
	// would be placed as:
	// 2|                 N                 |3
	//
	// W            \(r😏bot)/              E
	//
	//
	// 1| __________________________________|4
	//                   S
	//The lines next to each number represent the way a board covered in retroreflective tape of a specfic color or shape are placed in respect to eachother.

	public final double onex = 0;
	public final double oney = 0;
	public final double twox = 0;
	public final double twoy = 20;
	public final double threex = 60;
	public final double threey = 20;
	public final double fourx = 60;
	public final double foury = 0;

	public double getX() {
		/**
		 * If there is no target (v == 0) then return 0.0 for angle... don't want robot
		 * to turn to a target that doesn't exist
		 */
		if (v == 0) {
			return 0.0;
		} else {
			return x;
		}
	}

	public double getY() {
		return y;
	}

	public double getV() {
		return v;
	}

	public double getS() {
		return s;
	}

	public int pl;
	public double distM;
	public double skewangle;
	public double xcoord;
	public double ycoord;
	public double realangle;
	public double compassdeg;

	
	public int findPL() {
		for (int b = 0; b <= 10; b++) {
			pip.setNumber(b);
			pl = b;
			if (v == 1)
				break;
			// cycles through pipelines looking for a target
		}

		return pl;

	}
	// finds distance to markers
	public Double distanceM() {
		findPL();
		if (pl == 0 || pl == 1)
			return null;
		distM = Constants.cameraHeight * (Math.tan(Math.toRadians(Math.abs(y - Constants.cameraAngle))));
		return distM;
	}

	// returns x coordinates in inches
	public Double locX() {
		double skew;

		findPL();
		if (pl == 2 || pl == 4) {
			skew = s * -1.0;
			realangle = skew + x;
		} else if (pl == 3 || pl == 5) {
			skew = s + 90.0;
			realangle = skew - x;
		} else {
			return null;
		}
		switch (pl) {
			case 0:
				return null;
			case 1:
				return null;
			case 2:
				xcoord = onex;
			case 3:
				xcoord = twox;
			case 4:
				xcoord = threex;
			case 5:
				xcoord = fourx;
		}
		Double X = distanceM() * Math.cos(Math.toRadians(realangle)) + xcoord;
		return X;
	}

	// returns y coordinantes in inches
	public Double locY() {
		double skew;

		findPL();
		if (pl == 2 || pl == 4) {
			skew = s * -1.0;
			realangle = skew + x;
		} else if (pl == 3 || pl == 5) {
			skew = s + 90.0;
			realangle = skew - x;
		} else {
			return null;
		}
		switch (pl) {
			case 0:
				return null;
			case 1:
				return null;
			case 2:
				ycoord = oney;
			case 3:
				ycoord = twoy;
			case 4:
				ycoord = threey;
			case 5:
				ycoord = foury;
		}
		Double Y = distanceM() * Math.sin(Math.toRadians(realangle)) + ycoord;
		return Y;
	}

	// finds absolute rotation of robot on field,
	public Double degrees() {

		findPL();
		switch (pl) {
			case 0:
				return null;
			case 1:
				return null;
			case 2:
				compassdeg = 270.00 - Math.abs(s); // marker 1
			case 3:
				compassdeg = Math.abs(s) + 270.00; // marker 2
			case 4:
				compassdeg = 90.00 - Math.abs(s); // marker 3
			case 5:
				compassdeg = Math.abs(s) + 90.00; // marker 4
		}
		return compassdeg;

	}

	// distance in inches to shooter target
	public Double distance() {

		findPL();
		if (pl != 0) {
			return null;
		} else {
			/**
			 * Note: Math.tan takes radians...thus the conversion. Finds distance to
			 * shooting target
			 * 
			 * Note: Constants must be set precisely to the robots configuration or the
			 * distance calculations will be wrong. If all of a sudden the distance is off
			 * from one match to another, check the angle of the LimeLight camera. If it
			 * gets bumped and the angle changes then everything will be off. That is really
			 * the main variable that can get bumped.
			 */
			return ((Constants.targetHeight - Constants.cameraHeight)
					/ Math.tan(Math.toRadians(Constants.cameraAngle + y)));
		}
	}

	@Override
	public void periodic() {
		super.periodic();
		x = tx.getDouble(0.0);
		y = ty.getDouble(0.0);
		v = tv.getDouble(0.0);
		s = ts.getDouble(0.0);
		SmartDashboard.putNumber("LimelightX", x);
		SmartDashboard.putNumber("LimelightY", y);
		SmartDashboard.putNumber("LimelightS", s);
		SmartDashboard.putNumber("Limelightdistance", distance());
		SmartDashboard.putNumber("LimelightdistanceM", distanceM());
		SmartDashboard.putNumber("Limelightcardinal", degrees());
		SmartDashboard.putNumber("LimelightxCoord", locX());
		SmartDashboard.putNumber("LimelightyCoord", locY());
	}
}
