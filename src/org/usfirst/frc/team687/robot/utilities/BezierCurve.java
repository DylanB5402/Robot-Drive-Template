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
	private double[] t_list;
	private	double[] m_xList;
	private double[] m_yList;
	private double[] m_angleList;
	private double m_deltaX;
	private double m_deltaY;
	private double m_prevX;
	private double m_prevY;
	private double m_distance = 0;
	private double m_hypotenuse;
	
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
		m_hypotenuse = 0;
		m_prevX = 0;
		m_prevY = 0;
		while (m_t != 1) {
			m_t = m_a/m_step;
			m_x = (m_x0 * Math.pow((1-m_t), 3)) + (3 * m_x1 * m_t * Math.pow((1-m_t), 2)) + (3 * m_x2 * (1-m_t) * Math.pow(m_t, 2)) + (m_x3 * Math.pow(m_t, 3));               
			m_y = (m_y0 * Math.pow((1-m_t), 3)) + (3 * m_y1 * m_t * Math.pow((1-m_t), 2)) + (3 * m_y2 * (1-m_t) * Math.pow(m_t, 2)) + (m_y3 * Math.pow(m_t, 3));   
			m_deltaX = m_x - m_prevX;
			m_deltaY = m_y - m_prevY;			 
            m_angle = Math.atan2(m_deltaX, m_deltaY);
            m_angle = Math.toDegrees(m_angle);
            m_hypotenuse = NerdyMath.distanceFormula(m_x, m_y, m_prevX, m_prevY);
            m_distance += m_hypotenuse;
            
		}
	}
	
	public double[] getXList() {
		return m_xList;
	}
	
	public double[] getYList() {
		return m_yList;
	}
	
	public double[] getAngleList() {
		return m_angleList;
	}
	
	public double getX(int x) {
		return m_xList[x];
	}
	
	public double getY(int y) {
		return m_yList[y];
	}
	
	public double getCurveLength() {
		return m_distance;
	}
	
	public double getLastX() {
		return m_xList[(int) Math.round(m_step) - 1];
	}
	
	public double getLastY() {
		return m_yList[(int) Math.round(m_step) - 1];
	}
}
