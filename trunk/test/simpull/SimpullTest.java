package simpull;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import javax.swing.JFrame;

import simpull.Circle;
import simpull.Composite;
import simpull.CustomBodyFinisher;
import simpull.Particle;
import simpull.Rectangle;
import simpull.SimpleSpring;
import simpull.VectorForce;

public class SimpullTest extends JFrame implements KeyListener {
	
	private static final long serialVersionUID = -6434349154299741766L;
	private Composite star, tumbler;
	private Particle pendulum;
	private Group defaultPhysicsGroup = new Group(true);
	private float vel = 0.5f;
	private float lessVel = vel / 5f;

	public SimpullTest() {
		super("simpull - stress test");
		Simpull.init(1/4f);
		Simpull.add(new VectorForce(false, 0f, 3f));
		System.out.println("loading...");
		load();
		// After all physics objects added, the group can be added to the engine
		Simpull.add(defaultPhysicsGroup);
		
		addKeyListener(this);
	    addWindowListener(new WindowAdapter(){
	        public void windowClosing(WindowEvent we){
	          System.exit(0);
	        }
	      });
	    setSize(160, 90);
	    setVisible(true);
		
		
		System.out.println("running...");
		final Vector2f isRunning = new Vector2f();
		new Thread() {
			long startTime = System.currentTimeMillis();
			public void run() {
				// run for 30 seconds
				while ((System.currentTimeMillis() - startTime) < (30 * 1000)) {
					Simpull.step(); // do all physics calcs
					Thread.yield();
				}
				isRunning.x = 111;
			};
		}.start();
		while (isRunning.x == 0) {
			Thread.yield();
		}
		System.out.println("...complete!");
        System.exit(0);
	}
	
	public static void main(String[] args) {
		new SimpullTest();
	}
	
	protected void load() {
		{ // set up the branch(es) in the upper right corner
			defaultPhysicsGroup.add(new Rectangle(785, 90, 40, 40, 0.4f, true, 1, 0, 1));

			defaultPhysicsGroup.add(new Rectangle(746, 79, 40, 40, 0.2f, true, 1, 0, 1));
	
			defaultPhysicsGroup.add(new Rectangle(710, 72, 40, 40, 0, true, 1, 0, 1));
		}
		
		{ // star 
			star = new Composite(false, 8, true);
			// The top-left point of the star is (710, 10), so I got the points
			// on the start from the image creation software and added the reference 
			// location to get a good physical representation...cool trick!
			star.add(new Circle(14 + 710, 7 + 10, 1f, false, 1f, 0.3f, 1f));
			star.add(new Circle(29 + 710, 0 + 10, 1f, false, 1f, 0.3f, 1f));
			star.add(new Circle(20 + 710, 14 + 10, 1f, false, 1f, 0.3f, 1f));
			star.add(new Circle(29 + 710, 29 + 10, 1f, false, 1f, 0.3f, 1f));
			star.add(new Circle(14 + 710, 22 + 10, 1f, false, 1f, 0.3f, 1f));
			star.add(new Circle(0 + 710, 29 + 10, 1f, false, 1f, 0.3f, 1f));
			star.add(new Circle(7 + 710, 14 + 10, 1f, false, 1f, 0.3f, 1f));
			star.add(new Circle(0 + 710, 0 + 10, 1f, false, 1f, 0.3f, 1f));
			star.setCompositeCompleted(new CustomBodyFinisher(3, 1));
			defaultPhysicsGroup.add(star);
		}
		
		{ // The tumbler
			tumbler = new Composite(true, 20, true);
			tumbler.add(new Circle(50, 10, 4, false, 1, 0.3f, 0));
			tumbler.add(new Circle(150, 10, 4, false, 1, 0.3f, 0));
			tumbler.add(new Circle(100, 60, 4, false, 1, 0.3f, 0));
			
			// Finish the body by connecting the particles we just added with some constraints that have grateImg
			// as a view (scaled to the right size to fit between each particle).
			tumbler.setCompositeCompleted(new CustomBodyFinisher(8, 0.9f));
			defaultPhysicsGroup.add(tumbler);
		}
		
		// Until the water is ready, use a wheel inside to tumble
		defaultPhysicsGroup.add(new Wheel(90, 35, 10, false, 0.5f, 0.3f, 0, 1));
		
		// add a mountain (cosine wave), with walls at the edge of the screen up to the height of the wave
		final int iterations = 50;
		final float xMax = 800f / ((float)Math.PI * 2f);
		float lastDangleX = 0f;
		for (int i = 0; i < iterations; ++i) {
			float x = (float)Math.PI * 2f * ((float)i / (float)iterations);
			float worldX = x * xMax;
			float y = (float)Math.cos(x) * 100 + 330;
			Rectangle rockRect = new Rectangle(worldX, y, 20, 20, (float)(Math.random() * Math.PI * 2), true, 1, 0.3f, 0);
			defaultPhysicsGroup.add(rockRect);
			// Every so often, add a side wall rock piece on either side
			if (i % 4 == 0) {
				defaultPhysicsGroup.add(new Rectangle(5, y, 20, 20, (float)(Math.random() * Math.PI * 2), true, 1, 0.3f, 0));
				defaultPhysicsGroup.add(new Rectangle(795, y, 20, 20, (float)(Math.random() * Math.PI * 2), true, 1, 0.3f, 0));
			}
			// If the spacing is right, add a dangling participle!!!
			if (worldX > 130 && worldX < 680) {
				if ((worldX - lastDangleX) > 31) {
					lastDangleX = worldX;
					float magic = worldX > 400 ? 800 - worldX : worldX;
					Rectangle danglerRect = new Rectangle(worldX, y + 25 + (magic / 4), 30, 30, 0, false, 2, 0.3f, 0);
					if (pendulum == null) {
						pendulum = danglerRect;
					}
					defaultPhysicsGroup.add(danglerRect);
					SimpleSpring spring = new SimpleSpring(rockRect, danglerRect, 0.5f, false, 2, 1, false);
					defaultPhysicsGroup.add(spring);
				}
			}
		}
		
		{ // left side of tires
			float x = 340;
			defaultPhysicsGroup.add(new Wheel(x, 200, 20, false, 2, 0.3f, 0, 1));
			defaultPhysicsGroup.add(new Wheel(x, 160, 20, false, 2, 0.3f, 0, 1));
			defaultPhysicsGroup.add(new Wheel(x, 120, 20, false, 2, 0.3f, 0, 1));
			defaultPhysicsGroup.add(new Wheel(x, 80, 20, false, 2, 0.3f, 0, 1));
			defaultPhysicsGroup.add(new Wheel(x, 40, 20, false, 2, 0.3f, 0, 1));
			defaultPhysicsGroup.add(new Wheel(x, 0, 20, false, 2, 0.3f, 0, 1));
		}
		{ // right side of tires
			float x = 460;
			defaultPhysicsGroup.add(new Wheel(x, 200, 20, false, 2, 0.3f, 0, 1));
			defaultPhysicsGroup.add(new Wheel(x, 160, 20, false, 2, 0.3f, 0, 1));
			defaultPhysicsGroup.add(new Wheel(x, 120, 20, false, 2, 0.3f, 0, 1));
			defaultPhysicsGroup.add(new Wheel(x, 80, 20, false, 2, 0.3f, 0, 1));
			defaultPhysicsGroup.add(new Wheel(x, 40, 20, false, 2, 0.3f, 0, 1));
			defaultPhysicsGroup.add(new Wheel(x, 0, 20, false, 2, 0.3f, 0, 1));
		}
	}
	
	@Override
	public void keyPressed(KeyEvent e) {
		if (e.getKeyCode() == KeyEvent.VK_LEFT) {
			star.setAngularVelocity(-vel);
			pendulum.addForce(new VectorForce(true, -40, 0));
			tumbler.setAngularVelocity(-lessVel);
		} else if (e.getKeyCode() == KeyEvent.VK_RIGHT) {
			star.setAngularVelocity(vel);
			pendulum.addForce(new VectorForce(true, 40, 0));
			tumbler.setAngularVelocity(lessVel);
		}
	}

	@Override
	public void keyReleased(KeyEvent e) {
		star.setAngularVelocity(0);
		tumbler.setAngularVelocity(0);
	}

	@Override
	public void keyTyped(KeyEvent e) {}

}
