package org.rivierarobotics.mathutil;

import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.Line2D;

import javax.swing.JFrame;
import javax.swing.JPanel;

public class SwerveTester extends JFrame {

    private static final long serialVersionUID = 1L;
    public Vector2d[] pos = { new Vector2d(-1, 1), new Vector2d(1, 1), new Vector2d(-1, -1), new Vector2d(1, -1) };
    public double ang = -Math.PI / 4;
    public double omega = .2;
    public Vector2d trans = new Vector2d(0, .5);
    public Vector2d[] drive = SwerveCalculator.calculateAllModules(ang, omega, trans, pos);

    public SwerveTester() {
        JPanel panel = new JPanel();
        getContentPane().add(panel);
        setSize(450, 450);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    }

    public void paint(Graphics g) {
        super.paint(g);
        Graphics2D g2 = (Graphics2D) g;
        for (int i = 0; i < 4; i++) {
            Vector2d drawPos = pos[i].rotate(ang).scale(50).add(new Vector2d(225, 225));
            Vector2d drawVel = drive[i].scale(100).rotate(ang);
            Vector2d endPos = drawPos.add(drawVel);
            Line2D lin = new Line2D.Double(drawPos.getX(), drawPos.getY(), endPos.getX(), endPos.getY());
            g2.draw(lin);
        }
    }

    public static void main(String[] args) {
        SwerveTester st = new SwerveTester();
        st.setVisible(true);
    }

}
