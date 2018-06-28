package javacontrols;
import java.awt.Container;
import java.io.IOException;
import java.util.Iterator;
import java.util.Random;
import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.WindowConstants;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.ejml.ops.MatrixIO;
import org.ejml.simple.SimpleMatrix;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

public class SimulateSwerve {
    
    public static SimpleMatrix simulate() {
        String parentDir = "/home/petey/Documents/RoboticsCode/DiffSwerve/simulation/";
        try {
            //Simulation time params
            double dt = .005;
            double totalTime = 1;
            int numTimeSteps = (int)(totalTime/dt) + 1;
            
            //State feedback matrices from files
            SimpleMatrix A_d = SimpleMatrix.wrap(MatrixIO.loadCSV(parentDir + "Ad_mat.txt", true));
            SimpleMatrix B_d = SimpleMatrix.wrap(MatrixIO.loadCSV(parentDir + "Bd_mat.txt", true));
            SimpleMatrix C = new SimpleMatrix(2,4);
            C.set(0, 0, 1);
            C.set(0, 2, 1);
            C.set(1, 0, 1);
            C.set(1, 2, -1);    
            SimpleMatrix K_d = SimpleMatrix.wrap(MatrixIO.loadCSV(parentDir + "K_mat.txt", true));
            double kvff = 0.10455;//TODO: make this not a magic number 

            //Kalman matrices
            double pNoise = Math.PI/360;
            double oNoise = Math.PI/360;
            SimpleMatrix Q = SimpleMatrix.identity(4).scale(pNoise);
            SimpleMatrix R = SimpleMatrix.identity(2).scale(oNoise);
            
            //Initialize dynamic matrices
            SimpleMatrix x = new SimpleMatrix(4,1);
            SimpleMatrix x_hat = new SimpleMatrix(4,1);
            SimpleMatrix x_hat_ = new SimpleMatrix(4,1);
            SimpleMatrix y = C.mult(x);
            SimpleMatrix P = SimpleMatrix.identity(4);
            SimpleMatrix P_ = SimpleMatrix.identity(4);
            SimpleMatrix Kk = new SimpleMatrix(4,2);
            SimpleMatrix u = new SimpleMatrix(2,1);
            SimpleMatrix Rs = new SimpleMatrix(4,1);
            SimpleMatrix vff = new SimpleMatrix(2,1);
            SimpleMatrix oNoiseVec = new SimpleMatrix(4,1);
            SimpleMatrix pNoiseVec = new SimpleMatrix(4,1);

            //Allocate logging buffers
            SimpleMatrix x_history = new SimpleMatrix(4,numTimeSteps);
            SimpleMatrix x_hat_history = new SimpleMatrix(4,numTimeSteps);
            SimpleMatrix u_history = new SimpleMatrix(2,numTimeSteps);
            SimpleMatrix y_history = new SimpleMatrix(2,numTimeSteps);
            
            for(int t = 0; t < numTimeSteps; t++) {
                //closed loop setpoints
                Rs.set(0, Math.PI/2);
                Rs.set(1, x_hat.get(1));
                Rs.set(2, x_hat.get(2));
                Rs.set(3, 28*Math.PI);
                
                //calculating voltages
                vff.set(0, Rs.get(3)*kvff);
                vff.set(1, -Rs.get(3)*kvff);
                u = K_d.mult(Rs.minus(x)).plus(vff);
                
                //cap input
                u.set(0, Math.min(12, Math.abs(u.get(0)))*Math.signum(u.get(0)));
                u.set(1, Math.min(12, Math.abs(u.get(1)))*Math.signum(u.get(1)));
                
                //record everything
                x_history.insertIntoThis(0, t, x);
                x_hat_history.insertIntoThis(0, t, x_hat);
                u_history.insertIntoThis(0, t, u);
                y_history.insertIntoThis(0, t, y);


                //update system
                Random rand = new Random();
                RandomMatrices_DDRM.fillGaussian(pNoiseVec.getDDRM(), 0.0, pNoise, rand);
                RandomMatrices_DDRM.fillGaussian(oNoiseVec.getDDRM(), 0.0, oNoise, rand);
                x = A_d.mult(x).plus(B_d.mult(u)).plus(pNoiseVec);
                y = C.mult(x.plus(oNoiseVec));
                
                //kalman filter prediction
                P_ = A_d.mult(P).mult(A_d.transpose()).plus(Q);
                x_hat_ = A_d.mult(x_hat).plus(B_d.mult(u));
                
                //kalman update
                Kk = P_.mult(C.transpose()).mult((C.mult(P_).mult(C.transpose()).plus(R)).invert());
                x_hat = x_hat_.plus(Kk.mult(y.minus(C.mult(x_hat_))));
                P = P_.minus(Kk.mult(C).mult(P_));                
            }
            return x_history;
            
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }
    
    //Plotting in Java is hell
    public static XYDataset createPhiDataset(SimpleMatrix x_history, double dt, double totalTime) {
        XYSeries phi = new XYSeries("phi");
        int numTimeSteps = (int)(totalTime/dt) + 1;

        double time = 0;
        for(Iterator<Double> iter = x_history.iterator(true, 0, 0, 0, numTimeSteps-1); iter.hasNext();) {
            phi.add(time, iter.next());
            time += dt;
        }
    
        XYSeriesCollection dataset = new XYSeriesCollection();
        dataset.addSeries(phi);
        return dataset;
    }
    
    public static XYDataset createAlphaPrimeDataset(SimpleMatrix x_history, double dt, double totalTime) {
        XYSeries alphaPrime = new XYSeries("alphaprime");
        int numTimeSteps = (int)(totalTime/dt) + 1;

        double time = 0;
        for(Iterator<Double> iter = x_history.iterator(true, 3, 0, 3, numTimeSteps-1); iter.hasNext();) {
            alphaPrime.add(time, iter.next());
            time += dt;
        }
        XYSeriesCollection dataset = new XYSeriesCollection();
        dataset.addSeries(alphaPrime);
        return dataset;
    }
    
    //Swing is an abomination
    public static void main(String[] args) {
        JFrame frame = new JFrame();
        Container pane = frame.getContentPane();
        pane.setLayout(new BoxLayout(pane, BoxLayout.PAGE_AXIS));
        SimpleMatrix x_history = simulate();
        JFreeChart phiChart = ChartFactory.createXYLineChart("Module Position", "Time (sec)", "Position (Rad)", createPhiDataset(x_history, .005,.5));
        JFreeChart alphaChart = ChartFactory.createXYLineChart("Wheel Speed", "Time (sec)", "Vel (Rad/s)", createAlphaPrimeDataset(x_history, .005,.5));
        pane.add(new ChartPanel(phiChart));
        pane.add(new ChartPanel(alphaChart));
        frame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
        frame.setExtendedState(JFrame.MAXIMIZED_BOTH);
        frame.setVisible(true);
    }

}
