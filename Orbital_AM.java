import java.awt.BasicStroke;
import java.awt.Color;
import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Random; 

import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.Clip;

import org.opensourcephysics.controls.AbstractSimulation;
import org.opensourcephysics.controls.SimulationControl;
import org.opensourcephysics.display.DrawingPanel;
import org.opensourcephysics.display.Trail;
import org.opensourcephysics.frames.DisplayFrame;
import org.opensourcephysics.frames.PlotFrame;

public class Orbital_AM extends AbstractSimulation {
	Random random = new Random(); 
	protected static DisplayFrame frame = new DisplayFrame("x", "y", "Frame"); 
	protected static PlotFrame lawOne = new PlotFrame("x", "y", "Law One");
	protected static PlotFrame lawTwo = new PlotFrame("x", "y", "Law Two");
	protected static PlotFrame lawThree = new PlotFrame("x", "y", "Law Three");
	protected List<Trail> trails; 
	protected static ArrayList<Particle> bodies = new ArrayList<Particle>(); 
	protected DrawingPanel drawingPanel; 

	protected final double g = 9.803; //gravity constant
	protected final double G = 6.67384*Math.pow(10, -11);
	final private static boolean DEBUG_MODE = false;

	//get = false
	protected boolean set_get = false; 

	final private static double SUN_MASS = 1.98892E30; //kg 
	final private static double EARTH_MASS = 5.9742E24; //kg
	final private static double EARTH_ORBIT = 1.5E11; //meters - earth radius 
	final private static double EARTH_TANGENTIAL_VELOCITY = 30000; 
	final private static double COLLISION_RAD = 3; 

	final private static int[] FRAME_LOC = {0, 0};
	final private static int[] FRAME_DIMENSIONS = {600, 700};
	final private static int[] CONTROL_LOC = {850, 0}; 
	final private static int[] CONTROL_DIMENSIONS = {200, 700}; 
	final private static double[] FRAME_MINMAX = {-1.2*EARTH_ORBIT, 1.2*EARTH_ORBIT, -1.2*EARTH_ORBIT, 1.2*EARTH_ORBIT}; //scale at which the frame starts

	private static double m1, m2; 
	private static double stepsPerDisplay; 
	private static double TIME_STEP = 86000; 
	private static double BODY_NUM; 
	private static double EFFICIENCY; 
	private static boolean COLLISIONS; 

	private static boolean LAW_ONE; 
	private static boolean LAW_TWO; 
	private static boolean LAW_THREE;
	private static boolean RESET = true; 

	protected double total_time = 0; 

	protected int mass_info; 
	private HashMap<Integer, Color> map = new HashMap<Integer, Color>(); 
	protected List<Particle> temp = new ArrayList<Particle>(); 

	Test tester = new Test();

	int count = 0; 

	protected void doStep(){
		count++; 
		//tester.revalidate(bodies);
		if(count % 100 == 0){
			tester.refresh(bodies);
			count = 0; 
		}

		total_time += TIME_STEP; 
		calculate_accelerations(); 
		move_bodies(); 
		if(LAW_ONE)
			law_one(); 
		if(LAW_TWO)
			law_two(); 
		if(LAW_THREE)
			law_three(); 

		if(DEBUG_MODE){
			System.out.println();
			System.out.println("System is moving " + bodies.size() + " bodies.");
			for (int ii = 0; ii < bodies.size(); ii++) {
				trails.get(ii).addPoint(bodies.get(ii).getX_pos(), bodies.get(ii).getY_pos()); //connect the masses
				trails.get(ii).setStroke(new BasicStroke(2));
				trails.get(ii).color = bodies.get(ii).color; 
				frame.addDrawable(trails.get(ii)); 
			}
		}
	}

	private void calculate_accelerations(){
		for (int ii = 0; ii < bodies.size(); ii++) { //step through arraylist of masses
			bodies.get(ii).forces.clear(); //clear old forces
			bodies.get(ii).forces.removeAll(bodies.get(ii).forces); 

			for (int jj = 0; jj < bodies.size(); jj++) {
				if(ii == jj)
					continue; 
				bodies.get(ii).forces.add(grav_force(bodies.get(ii), bodies.get(jj)));
			}
			double xSum = sumForces(bodies.get(ii).forces, 0); 
			double ySum = sumForces(bodies.get(ii).forces, 1);
			bodies.get(ii).acc_x = xSum/bodies.get(ii).mass; //a = F/m
			bodies.get(ii).acc_y = ySum/bodies.get(ii).mass;

			if(DEBUG_MODE){
				System.out.println("X Acceleration of " + ii + " is " + bodies.get(ii).acc_x);
				System.out.println("Y Acceleration of " + ii + " is " + bodies.get(ii).acc_y);
			}
		}
	}

	public void law_one(){

	}

	public void law_two(){
		for (int ii = 0; ii < bodies.size(); ii ++) {
			lawTwo.append(ii, total_time, bodies.get(ii).v_x);
		}
	}

	public void law_three(){

	}

	private Force grav_force(Particle m1, Particle m2){
		double dist = dist(m1, m2);
		double mag = G*m1.mass*m2.mass/Math.pow(dist, 2);

		if(DEBUG_MODE)
			System.out.println("Force added to mass " + m1 + " in magnitude " + mag + " dir: " + calcAng(m1, m2));
		return new Force(mag, calcAng(m1, m2), "Gravitational Force"); 
	}

	private void move_bodies(){
		for (int ii = 0; ii < bodies.size(); ii++) {
			for (int jj = 0; jj < bodies.size(); jj++) {
				if(ii == jj) continue; 
				else if (checkCollision(bodies.get(ii), bodies.get(jj))) {
					setCollision(bodies.get(ii), bodies.get(jj));
					//System.out.println("NEW V: " + bodies.get(ii).v_x);
				}
				else {
					bodies.get(ii).bump = null; 
				}
			}
		}
		for (int ii = 0; ii < bodies.size(); ii++) {
			bodies.get(ii).Step(frame, true); //not 2d simulated 
		}
	}

	/**
	 * CREDIT
	 * 
	 * @param p1
	 * @param p2
	 * @return
	 */
	private boolean checkCollision(Particle p1, Particle p2) {
		DrawingPanel drawingPanel = frame.getDrawingPanel();
		int p1CenterPixelX = drawingPanel.xToPix(p1.getX());
		int p1CenterPixelY = drawingPanel.yToPix(p1.getY());
		int p2CenterPixelX = drawingPanel.xToPix(p2.getX());
		int p2CenterPixelY = drawingPanel.yToPix(p2.getY());
		int dist = (int)Math.round(dist(p1CenterPixelX, p1CenterPixelY, p2CenterPixelX, p2CenterPixelY));

		if(dist + COLLISION_RAD > p1.radius + p2.radius)
			return false;
		else if(p1.bump == null || p2.bump == null) //their collision hasn't already been computed
			return true;

		else if(p1.bump.equals(p2) || p2.bump.equals(p1))
			return false; //don't count this time as a collision because it's already been computed

		return true;
	}

	private void setCollision(Particle m1, Particle m2){
		m1.bump = m2; 
		m2.bump = m1; 

		double v1ix = m1.v_x; 
		double v2ix = m2.v_x; 
		double v1iy = m1.v_y; 
		double v2iy = m2.v_y; 

		double v1fx;  
		double v2fx; 
		double v1fy; 
		double v2fy;  

		if(EFFICIENCY == 100){ //efficiency == 100
			//elastic collision 
			//v1i + v2i = v1f + v2f 
			//mv1 + mv2 = mv1 + mv2
			v1fx = 2*(m1.mass*v1ix + m2.mass*v2ix)/(m1.mass + m2.mass); 
			v1fx -= v1ix; 
			v2fx = 2*(m1.mass*v1ix + m2.mass*v2ix)/(m1.mass + m2.mass); 
			v2fx -= v2ix; 
			v1fy = 2*(m1.mass*v1iy + m2.mass*v2iy)/(m1.mass + m2.mass); 
			v1fy -= v1iy; 
			v2fy = 2*(m1.mass*v1iy + m2.mass*v2iy)/(m1.mass + m2.mass); 
			v2fy -= v2iy; 

			m1.v_x = v1fx;
			m2.v_x = v2fx;
			m1.v_y = v1fy;
			m2.v_y = v2fy;

			System.out.println("M1's vx changed from " + v1ix + " to " + m1.v_x);
			System.out.println("M2's vx changed from " + v2ix + " to " + m2.v_x);
		}
		else {
			//inelastic collision
			Particle monster = new Particle(); 
			double v_final = (m1.mass*m1.vector(m1.v_x, m1.v_y) + m2.mass*m2.vector(m1.v_x, m1.v_y))/(m1.mass + m2.mass); 
			monster.init(m1.x_pos, m1.y_pos, v_final, calcAng(m1, m2), TIME_STEP, 0);
			System.out.println("MONSTER: " + v_final + " ANG: " + calcAng(m1, m2));
			Color newColor = new Color((m1.color.getRed() + m2.color.getRed())/2, 
					(m1.color.getGreen() + m2.color.getGreen())/2, 
					(m1.color.getBlue() + m2.color.getBlue())/2); 
			monster.color = newColor;
			monster.useRiemann = false; 
			monster.pixRadius = 10; 
			monster.mass = m1.mass + m2.mass; 
			monster.color = map.get(random.nextInt(map.size()-1)+1); 
			frame.removeDrawable(m1);
			frame.removeDrawable(m2); 
			bodies.remove(m1); 
			bodies.remove(m2); 
			bodies.add(monster); 
			frame.addDrawable(monster); 
			monster.Step(frame, true);
		}
	}

	public void set_info(){
		mass_info = (int) control.getDouble("Mass Info"); 

		if(!set_get){
			if(DEBUG_MODE)
				System.out.println("GETTING");
			control.setValue("Mass", bodies.get(mass_info).mass);
			control.setValue("X", bodies.get(mass_info).x_pos); 
			control.setValue("Y", bodies.get(mass_info).y_pos);
			control.setValue("Vtan", bodies.get(mass_info).v_tangent);
			control.println("Mass " + mass_info + " properties shown."); 
			set_get = !set_get; 
		}
		else {
			if(DEBUG_MODE)
				System.out.println("SETTING");
			double angle = calcAng(bodies.get(mass_info), bodies.get(0))-90;
			double newTan = control.getDouble("Vtan");

			bodies.get(mass_info).x_pos = control.getDouble("X");
			bodies.get(mass_info).y_pos = control.getDouble("Y");
			bodies.get(mass_info).mass = control.getDouble("Mass");

			bodies.get(mass_info).v_tangent = newTan; 
			bodies.get(mass_info).v_x = newTan*Math.cos(angle); 
			bodies.get(mass_info).v_y = newTan*Math.sin(angle);
			control.println("Mass " + mass_info + " properties set.");
			set_get = !set_get; 
		}
		frame.addDrawable(bodies.get(mass_info));
		bodies.get(mass_info).Step(frame, true);
		frame.render(); 

		if(DEBUG_MODE){
			System.out.println("Set vtan of " + mass_info + " to " + bodies.get(mass_info).v_tangent);
			System.out.println("Set x pos to " + bodies.get(mass_info).x_pos + " should be " + control.getDouble("X"));
			System.out.println("Set y pos to " + bodies.get(mass_info).y_pos + " should be " + control.getDouble("Y"));
			System.out.println("Mass v_x " + bodies.get(mass_info).v_x);
			System.out.println("Mass v_y " + bodies.get(mass_info).v_y);
		}
	}

	int reset_counter = 0; 

	public void initialize() {
		temp.addAll(bodies); 

		if(reset_counter != 0){
			reset_counter = 0; 
			RESET = control.getBoolean("Reset");
		}

		super.setStepsPerDisplay(3);  
		super.delayTime = 0;

		if(RESET){
			system_reset();  
		}
	}

	public void add_mass(){
		bodies.add(new Particle()); 
		control.println("Mass # " + (bodies.size()-1) + " added; access it using control panel.");
		bodies.get(bodies.size()-1).init(0, 0, 0, 0, TIME_STEP, 0);
	}

	public void system_reset(){
		reset_counter = 1; 
		System.out.println("RESET");
		frame.dispose(); 
		frame = new DisplayFrame("x", "y", "Frame"); 
		frame.addButton("print_data", "Print Accelerations and Force", "Print Data", this);
		frame.addButton("clear_trails", "Clear Trails", "Clear Trail Points", this);
		frame.addButton("add_mass", "Add Mass", "Add New Mass", this);
		//frame.add("TEXT", tf1);

		frame.setSize(FRAME_DIMENSIONS[0], FRAME_DIMENSIONS[1]); 
		frame.setLocation(FRAME_LOC[0], FRAME_LOC[1]);
		frame.setPreferredMinMax(FRAME_MINMAX[0], FRAME_MINMAX[1], FRAME_MINMAX[2], FRAME_MINMAX[3]);

		lawOne.setSize(300, 300); 
		lawTwo.setSize(300, 300); 
		lawThree.setSize(300, 300); 
		lawOne.setLocation(600, 0); 
		lawTwo.setLocation(900, 0); 
		lawThree.setLocation(1200, 0); 
		lawOne.setVisible(LAW_ONE);
		lawTwo.setVisible(LAW_THREE);
		lawThree.setVisible(LAW_THREE);

		map.put(0, Color.red); 
		map.put(1, Color.green); 
		map.put(2, Color.blue); 
		map.put(3, Color.yellow.darker()); 
		map.put(4, Color.cyan.darker());
		map.put(5, Color.pink.brighter());
		map.put(6, Color.blue);
		map.put(7, Color.orange);
		map.put(8, Color.green.brighter());
		map.put(9, Color.blue.brighter());
		map.put(10, Color.pink.darker());

		frame.clearData();
		frame.clearDrawables();
		bodies.removeAll(bodies); 
		bodies.clear();

		TIME_STEP = control.getDouble("Time Step");
		BODY_NUM = control.getDouble("Number of Bodies");
		COLLISIONS = control.getBoolean("Collisions"); 

		if(control.getBoolean("Elastic Collisions"))
			EFFICIENCY = 100; 
		else 
			EFFICIENCY = 0; 

		m1 = control.getDouble("M1");
		m2 = control.getDouble("M2");

		trails = new ArrayList<Trail>(); 

		for (int ii = 0; ii < BODY_NUM; ii++) {
			trails.add(new Trail()); 
			bodies.add(new Particle()); 
			bodies.get(ii).useRiemann = false; 
			bodies.get(ii).init(ii*EARTH_ORBIT, 0, 0, 0, 0, 0, 0, TIME_STEP, 0);
			bodies.get(ii).pixRadius = 10; 
			bodies.get(ii).color = map.get(random.nextInt(map.size()-1)+1); 
			frame.addDrawable(bodies.get(ii)); 
		}

		bodies.get(0).mass = m1; 
		bodies.get(1).mass = m2;
		//bodies.get(1).v_y = EARTH_TANGENTIAL_VELOCITY; 

		//set_info(); 
	}

	public void reset(){		
		control.setValue("Number of Bodies", 2);
		control.setValue("Time Step", 100000);
		control.setValue("M1", SUN_MASS);
		control.setValue("M2", EARTH_MASS);

		control.setValue("Collisions", true); 
		control.setValue("Elastic Collisions", true); 
		control.setValue("Mass Info", 1); 

		control.setValue("Mass", EARTH_MASS);
		control.setValue("X", EARTH_ORBIT); 
		control.setValue("Y", 0);
		control.setValue("Vtan", EARTH_TANGENTIAL_VELOCITY);

		frame.clearData();
		frame.clearDrawables();
		bodies.removeAll(bodies); 
		bodies.clear();
		system_reset(); 
	}

	public void start(){
		tester.init(bodies); 

		bodies.get(1).v_y = control.getDouble("Vtan"); 
		System.out.println("V_Y: " + bodies.get(1).v_y);

		frame.setSize(FRAME_DIMENSIONS[0], FRAME_DIMENSIONS[1]); 
		frame.setLocation(FRAME_LOC[0], FRAME_LOC[1]);
		//frame.setPreferredMinMax(FRAME_MINMAX[0], FRAME_MINMAX[1], FRAME_MINMAX[2], FRAME_MINMAX[3]);
	}

	public void stop(){
		set_get = false; 
		//set_info(); 
	}

	List<Thread> appThreads = new ArrayList<Thread>();//keeps track of running lists


	public static void main(String[] args) {
		SimulationControl control = SimulationControl.createApp(new Orbital_AM()); //creates simulation implementing this class
		control.addButton("one_toggle", "Law 1"); 
		control.addButton("two_toggle", "Law 2"); 
		control.addButton("three_toggle", "Law 3"); 

		control.addButton("set_info", "Set Mass");

		control.setValue("Number of Bodies", 2);
		control.setValue("Time Step", 0.04);
		control.setValue("M1", SUN_MASS);
		control.setValue("M2", EARTH_MASS);

		control.setValue("Collisions", true); 
		control.setValue("Elastic Collisions", true); 
		control.setValue("Mass Info", 1); 

		control.setValue("Mass", EARTH_MASS);
		control.setValue("X", EARTH_ORBIT); 
		control.setValue("Y", 0);
		control.setValue("Vtan", EARTH_TANGENTIAL_VELOCITY);
		control.setValue("Reset", false); 
		frame.setVisible(false);
		Test test = new Test(); 
	}

	public static void one_toggle() {
		LAW_ONE = !LAW_ONE; 
		lawOne.setVisible(LAW_ONE);
	}
	public static void two_toggle() {
		LAW_TWO = !LAW_TWO; 
		lawTwo.setVisible(LAW_TWO);
	}
	public static void three_toggle() {
		LAW_THREE = !LAW_THREE; 
		lawThree.setVisible(LAW_THREE);
	}

	public void print_data(){
		control.println("SIMULATION DATA: "); 
		for (int ii = 0; ii < bodies.size(); ii++) {
			control.println("Velocity of Particle #" + (ii+1) + ": " + bodies.get(ii).vector(bodies.get(ii).v_x, bodies.get(ii).v_y));
			control.println("Accelerations of Particle #" + (ii+1) + ": " + bodies.get(ii).vector(bodies.get(ii).acc_x, bodies.get(ii).acc_y));
			control.println("Force on Particle #" + (ii+1) + ": " + bodies.get(ii).vector(bodies.get(ii).acc_x, bodies.get(ii).acc_y));
		}
	}

	public void clear_trails(){
		for (int ii = 0; ii < trails.size(); ii++) 
			trails.get(ii).clear();
	}

	/**
	 * Sums an ArrayList of forces. This method is used to calculate 
	 * acceleration on a particular mass: each mass has an ArrayList of 
	 * forces such as its weight and spring forces that must be summed 
	 * to determine acceleration (Newton's 2nd Law). 
	 * 
	 * @param f
	 * 	Arraylist of forces. 
	 * @param dir
	 * 	X or Y direction (changes whether to use x or y components of forces). 
	 * @return
	 * 	The net force in a direction (can be +/-). 
	 */
	static double sumForces(ArrayList<Force> f, double dir){
		double sum = 0; 
		if(dir == 0){
			for (int ii = 0; ii < f.size(); ii++) {
				sum += f.get(ii).forceX(); //get x components
			}
		}
		else {
			for (int ii = 0; ii < f.size(); ii++) {
				sum += f.get(ii).forceY(); //get y components
			}
		}
		return sum; 
	}

	/**
	 * Calculate the distance between two masses using distance formula. 
	 * 
	 * @param a
	 * 	Center mass. 
	 * @param b
	 * 	Target mass. 
	 * @return
	 * 	Distance between the two. 
	 */
	double dist(Particle a, Particle b){
		double dist = Math.sqrt(Math.pow(a.getX_pos() - b.getX_pos(), 2) + Math.pow(a.getY_pos() - b.getY_pos(), 2)); 
		return dist; //distance formula
	}

	double dist(double x1, double y1, double x2, double y2){
		return Math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
	}

	/**
	 * Calculate the angle between two masses in degrees. 
	 * 
	 * @param a
	 * 	Center mass. 
	 * @param b
	 * 	Target mass. 
	 * @return
	 * 	Angle (in degrees) between the two masses. 
	 */
	public static double calcAng(Particle a, Particle b) {
		double xDiff = b.x_pos - a.x_pos; //find delta x
		double yDiff = b.y_pos - a.y_pos; //find delta y
		double angle = Math.toDegrees(Math.atan2(yDiff, xDiff));
		//angle = Math.round(angle*100)/100; 
		return angle; //inverse tangent of y/x
	}

	/**
	 * Play a sound! 
	 * 
	 * @param filename
	 * 	The name of the sound file. 
	 * 
	 * Credit: http://stackoverflow.com/questions/2416935/how-to-play-wav-files-with-java 
	 */
	public static void play(String filename){
		try {
			Clip clip = AudioSystem.getClip(); //make a new clip
			clip.open(AudioSystem.getAudioInputStream(new File(filename))); 
			clip.start(); //play clip 
		}
		catch (Exception exc){
			exc.printStackTrace(System.out);
		}
	}
}
