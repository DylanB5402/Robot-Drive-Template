package org.usfirst.frc.team687.robot.utilities;

import java.util.ArrayList;

public class BezierCurve {
	
	private double m_x0;
	private double m_y0;
	private double m_x1;
	private double m_y1;
	private double m_x2;
	private double m_y2;
	private double m_x3;
	private double m_y3;
	private double m_step;
	private double m_x;
	private double m_y;
	private double m_t;
	private double m_angle;
	private double m_a;
	private ArrayList<Double> m_tList = new ArrayList<Double>();
	private	ArrayList<Double> m_xList = new ArrayList<Double>();
	private ArrayList<Double> m_yList = new ArrayList<Double>();
	private ArrayList<Double> m_angleList = new ArrayList<Double>();
	private ArrayList<Double> m_slopeList = new ArrayList<Double>();
	private ArrayList<Double> m_yInterceptList = new ArrayList<Double>();
	private ArrayList<Double> m_perpendicularSlopeList = new ArrayList<Double>();
	private ArrayList<Double> m_hypotenuseList = new ArrayList<Double>();
	private ArrayList<Double> m_deltaXList = new ArrayList<Double>();
	private double m_deltaX;
	private double m_deltaY;
	private double m_prevX;
	private double m_prevY;
	private double m_distance;
	private double m_hypotenuse;
	private double m_slope;
	private double m_yIntercept;
	private double m_perpendicularSlope;
	
	/*
	 * 
	 */
	public BezierCurve(double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, double step ) {
		m_x0 = x0;
		m_y0 = y0;
		m_x1 = x1;
		m_y1 = y1;
		m_x2 = x2;
		m_y2 = y2;
		m_x3 = x3;
		m_y3 = y3;
		m_step = step;
	}
	
	public void generateCurve() {
		m_distance = 0;
		m_hypotenuse = 0;
		m_prevX = 0;
		m_prevY = 0;
		m_t = 0;
		while (m_t != 1) {
			m_t = m_a/m_step;
			m_x = (m_x0 * Math.pow((1-m_t), 3)) + (3 * m_x1 * m_t * Math.pow((1-m_t), 2)) + (3 * m_x2 * (1-m_t) * Math.pow(m_t, 2)) + (m_x3 * Math.pow(m_t, 3));               
			m_y = (m_y0 * Math.pow((1-m_t), 3)) + (3 * m_y1 * m_t * Math.pow((1-m_t), 2)) + (3 * m_y2 * (1-m_t) * Math.pow(m_t, 2)) + (m_y3 * Math.pow(m_t, 3));   
			m_deltaX = m_x - m_prevX;
			m_deltaY = m_y - m_prevY;			 
            m_angle = Math.atan2(m_deltaX, m_deltaY);
            m_angle = Math.toDegrees(m_angle);
            m_hypotenuse = NerdyMath.distanceFormula(m_x, m_y, m_prevX, m_prevY);
            if (m_deltaX != 0) {
            	m_slope = m_deltaY/m_deltaX;
            }
            else {
            	m_slope = 0;
            }
            m_yIntercept = m_y - m_slope * m_x;
            if (m_slope != 0) {
            	m_perpendicularSlope = -Math.pow(m_slope, -1);
            	
            }
            else {
            	m_perpendicularSlope = 0;
            }
            m_distance += m_hypotenuse;
            m_tList.add(m_t);
            m_xList.add(m_x);
            m_yList.add(m_y);
            m_angleList.add(m_angle);
            m_slopeList.add(m_slope);
            m_yInterceptList.add(m_yIntercept);
            m_perpendicularSlopeList.add(m_perpendicularSlope);
            m_hypotenuseList.add(m_hypotenuse);
            m_deltaXList.add(m_deltaX);
            m_prevX = m_x;
            m_prevY = m_y;
            m_a += 1;
         
		}
	}
	
	public ArrayList<Double> getXList() {
		return m_xList;
	}
	
	public ArrayList<Double> getYList() {
		return m_yList;
	}
	
	public ArrayList<Double> getAngleList() {
		return m_angleList;
	}
	
	public double getX(int t) {
		return m_xList.get(t);
	}
	
	public double getY(int t) {
		return m_yList.get(t);
	}
	
	public double getAngle(int t) {
		return m_angleList.get(t);
	}
	
	public double getSlope(int t) {
		return m_slopeList.get(t);
	}
	
	public double getYIntercept(int t) {
		return m_yInterceptList.get(t);
	}
	
	public double getPerpendicularSlope(int t) {
		return m_perpendicularSlopeList.get(t);
	}
	
	public double getHypotenuse(int t) {
		return m_hypotenuseList.get(t);
	}
	
	public double getDeltaX(int t) {
		return m_deltaXList.get(t);
	}
	
	public double getCurveLength() {
		return m_distance;
	}
	
	public double getLastX() {
		if (m_xList.size() > 0) {
			return m_xList.get((int) Math.round(m_step));
		}
		else {
			return 0;
		}
	}
	
	public double getLastY() {
		if (m_yList.size() > 0) {
			return m_yList.get((int) Math.round(m_step));
		}
		else {
			return 0;
		}
	}
}

