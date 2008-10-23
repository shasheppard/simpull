/*
    Copyright (c) 2008, Shaun Curtis Sheppard
    All rights reserved.
    
    Redistribution and use in source and binary forms, with or without 
    modification, are permitted provided that the following conditions are met:

        * Redistributions of source code must retain the above copyright 
          notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright 
          notice, this list of conditions and the following disclaimer in the 
          documentation and/or other materials provided with the distribution.
        * Neither the name of Shaun Curtis Sheppard nor the names of its 
          contributors may be used to endorse or promote products derived from 
          this software without specific prior written permission.
    
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
    POSSIBILITY OF SUCH DAMAGE.
*/

package simpull;
	
/** A Spring-like constraint that connects two particles */
public class SimpleSpring implements IConstraint {
	
	public Particle particle1;
	public Particle particle2;	

	int stiffness;
	int restLength;

	private boolean isCollidable;
	private SpringParticle collisionParticle;
	
	/**
	 * @param particle1 The first particle this constraint is connected to.
	 * @param particle2 The second particle this constraint is connected to.
	 * @param stiffness The strength of the spring. Valid values are between 0 and 1. Lower values
	 * result in softer springs. Higher values result in stiffer, stronger springs.
	 * @param isCollidable Determines if the constraint will be checked for collision
	 * @param rectHeight If the constraint is collidable, the height of the collidable area
	 * can be set in pixels. The height is perpendicular to the two attached particles.
	 * @param rectScale If the constraint is collidable, the scale of the collidable area
	 * can be set in value from 0 to 1. The scale is percentage of the distance between 
	 * the the two attached particles.
	 * @param scaleToLength If the constraint is collidable and this value is true, the 
	 * collidable area will scale based on changes in the distance of the two particles. 
	 */
	public SimpleSpring(
			Particle particle1, 
			Particle particle2, 
			int stiffness/*= 0.5*/,
			boolean isCollidable/*= false*/,
			int rectHeight/*= 1*/,
			int rectScale/*= 1*/,
			boolean scaleToLength/*= false*/) {

		this.stiffness = stiffness;
		this.particle1 = particle1;
		this.particle2 = particle2;
		// if the two particles are at the same location, offset slightly
		if (particle1.position.x == particle2.position.x && particle1.position.y == particle2.position.y) {
			particle2.position.x += FP.POINT_0001;
		}
		restLength = getCurrLength();
		setCollidable(isCollidable, rectHeight, rectScale, scaleToLength);
	}
	
	public SimpleSpring(
			Particle particle1, 
			Particle particle2, 
			boolean isCollidable/*= false*/,
			int rectHeight/*= 1*/,
			int rectScale/*= 1*/,
			boolean scaleToLength/*= false*/) {
		this(particle1, particle2, 0, isCollidable, rectHeight, rectScale, scaleToLength);
	}
	
	/**
	 * The rotational value created by the positions of the two particles attached to this
	 * SpringConstraint. You can use this property to in your own painting methods, along with the 
	 * center property. 
	 * 
	 * @returns the current rotation in radians
	 */			
	public int getRotation() {
		int diffX = particle1.position.x - particle2.position.x;
		int diffY = particle1.position.y - particle2.position.y;
		return FP.atan2(diffY, diffX);
	}
	
	/**
	 * The center position created by the relative positions of the two particles attached to this.
	 * 
	 * @returns A Vector2f representing the center of this
	 */			
	public Vector2f getCenter() {
		return new Vector2f((particle1.position.x + particle2.position.x) >> 1,
				(particle1.position.y + particle2.position.y) >> 1);
	}
	
	/**
	 * If the isCollidable property is true, you can set the scale of the collidible area
	 * between the two attached particles. Valid values are from 0 to 1. If you set the value to 1, then
	 * the collision area will extend all the way to the two attached particles. Setting the value lower
	 * will result in an collision area that spans a percentage of that distance. Setting the value
	 * higher will cause the collision rectangle to extend past the two end particles.
	 */		 	
	public void setRectScale(int scale) {
		if (getCollisionParticle() == null) {
			return;
		}
		getCollisionParticle().setRectScale(scale);
	}
	
	public int getRectScale() {
		return getCollisionParticle().getRectScale();
	}
	
	/**
	 * Returns the length of the SpringConstraint, the distance between its two 
	 * attached particles.
	 */ 
	public int getCurrLength() {
		int diffX = particle1.position.x - particle2.position.x;
		int diffY = particle1.position.y - particle2.position.y;
		
		return FP.sqrt((int) (((long) diffX * diffX) >> FP.FRACTION_BITS) + (int) (((long) diffY * diffY) >> FP.FRACTION_BITS));
		// above replaces return (float)Math.sqrt(diff.x * diff.x + diff.y * diff.y);
	}
	
	/**
	 * If the isCollidable property is true, you can set the height of the 
	 * collidible rectangle between the two attached particles. Valid values are greater 
	 * than 0. If you set the value to 10, then the collision rect will have a height of 10.
	 * The height is perpendicular to the line connecting the two particles
	 */	 
	public int getRectHeight() {
		return getCollisionParticle().getRectHeight();
	}
	
	public void setRectHeight(int height) {
		if (getCollisionParticle() == null) {
			return;
		}
		getCollisionParticle().setRectHeight(height);
	}			
	
	public int getRestLength() {
		return restLength;
	}
	
	/**
	 * The restLength property sets the length. This value will be
	 * the distance between the two particles unless their position is altered by external forces. 
	 * The SpringConstraint will always try to keep the particles this distance apart. Values must 
	 * be > 0.
	 */			
	public void setRestLength(int restLength) {
		if (restLength <= 0) {
			throw new IllegalArgumentException("restLength must be greater than 0");
		}
		this.restLength = restLength;
	}
	
	/**
	 * Determines if the area between the two particles is tested for collision. If this value is on
	 * you can set the rectHeight and rectScale properties 
	 * to alter the dimensions of the collidable area.
	 */			
	@Override
	public boolean isCollidable() {
		return isCollidable;
	}
	
	/**
	 * For cases when isCollidable is true and only one of the
	 * two end particles are fixed. This value will dispose of collisions near the
	 * fixed particle, to correct for situations where the collision could never be
	 * resolved. Values must be between 0.0 and 1.0.
	 */	
	public int getFixedEndLimit() {
		if (collisionParticle == null) {
			return 0;
		}
		return collisionParticle.getFixedEndLimit();
	}	
			
	public void setFixedEndLimit(int fixedEndLimit) {
		if (collisionParticle == null) {
			return;
		}
		collisionParticle.setFixedEndLimit(fixedEndLimit);
	}
	
	public void setCollidable(boolean isCollidable, int rectHeight, 
			int rectScale, boolean scaleToLength/*=false*/) {
		this.isCollidable = isCollidable;
		collisionParticle = null;
		if (isCollidable) {
			collisionParticle = new SpringParticle(particle1, particle2, this, rectHeight, rectScale, scaleToLength);			
		}
		
		// FIXME: There is no call to collisionParticle.init() like there is when a constraint is added to a group and therefore some things will not work 
	}
	
	/** @return true if the passed particle is one of the two particles attached */
	@Override
	public boolean isConnectedTo(IPhysicsObject other) {
		return (other == particle1 || other == particle2);
	}
	
	/** @return true if both connected particle's isFixed property is true. */
	public boolean getFixed() {
		return particle1.isFixed && particle2.isFixed;
	}
	
	/**
	 * Sets up the visual representation of this SpringContraint. This method is called 
	 * automatically when an instance of this SpringContraint's parent Group is added to 
	 * {@link Simpull}, when  this SpringContraint's Composite is added to a Group, or this 
	 * SpringContraint is added to a Composite or Group.
	 */		
	@Override
	public void init() {	
		cleanup();
		if (isCollidable()) {
			collisionParticle.init();
		}
	}
	
	@Override
	public void cleanup() {}
	
	/**
	 * Corrects the position of the attached particles based on their position and
	 *  mass. This method is called automatically during the Simpull.step() cycle.
	 */		
	@Override
	public void resolve() {
		if (particle1.isFixed && particle2.isFixed) {
			return;
		}
		int deltaLength = getCurrLength();
		int invMass1 = particle1.getInvMass();
		int invMass2 = particle2.getInvMass();
		int invMassSum = invMass1 + invMass2;
		
		int diff = (int) (((long) (deltaLength - getRestLength()) << FP.FRACTION_BITS) / ((int) (((long) deltaLength * invMassSum) >> FP.FRACTION_BITS)));
		// above replaces float diff = (deltaLength - getRestLength()) / (deltaLength * invMassSum);
		
		int diffStiff = (int) (((long) diff * stiffness) >> FP.FRACTION_BITS);
		// above replaces float diffStiff = diff * stiffness;
		
		int diffX = particle1.position.x - particle2.position.x;
		int diffY = particle1.position.y - particle2.position.y;
		
		int dmdsX = (int) (((long) diffX * diffStiff) >> FP.FRACTION_BITS);
		// above replaces int dmdsX = diffX * diffStiff;
		
		int dmdsY = (int) (((long) diffY * diffStiff) >> FP.FRACTION_BITS);
		// above replaces int dmdsY = diffY * diffStiff;
		
		particle1.position.x -= (int) (((long) dmdsX * invMass1) >> FP.FRACTION_BITS);
		// above replaces particle1.position.x -= dmdsX * particle1.getInvMass();
		
		particle1.position.y -= (int) (((long) dmdsY * invMass1) >> FP.FRACTION_BITS);
		// above replaces particle1.position.y -= dmdsY * particle1.getInvMass();
		
		particle2.position.x += (int) (((long) dmdsX * invMass2) >> FP.FRACTION_BITS);
		// above replaces particle2.position.x += dmdsX * particle2.getInvMass();
		
		particle2.position.y += (int) (((long) dmdsY * invMass2) >> FP.FRACTION_BITS);
		// above replaces particle2.position.y += dmdsY * particle2.getInvMass();
	}		
	
	SpringParticle getCollisionParticle(){
		return collisionParticle;
	}
	
	/** This class is here to provide collision support for Spring instances where isCollidable is true. */
	class SpringParticle extends Rectangle {
		
		SimpleSpring parent;
		
		private Particle particle1;
		private Particle particle2;
		
		private Vector2f avgVelocity = new Vector2f();
		private Vector2f lambda = new Vector2f();
		private boolean scaleToLength;
		
		private Vector2f rca = new Vector2f();
		private Vector2f rcb = new Vector2f();
		private int s;
		
		private int rectScale;
		private int rectHeight;
		private int fixedEndLimit;
				
		public SpringParticle(
				Particle particle1, 
				Particle particle2, 
				SimpleSpring parent, 
				int rectHeight, 
				int rectScale,
				boolean scaleToLength) {
			super(0,0,0,0,0,false);
			this.particle1 = particle1;
			this.particle2 = particle2;
			this.parent = parent;
			isFixed = parent.getFixed();
			this.rectScale = rectScale;
			this.rectHeight = rectHeight;
			this.scaleToLength = scaleToLength;
			fixedEndLimit = 0;
		}
		
		void setRectScale(int scale) {
			rectScale = scale;
		}
		
		int getRectScale() {
			return rectScale;
		}
		
		void setRectHeight(int height) {
			rectHeight = height;
		}
		
		int getRectHeight() {
			return rectHeight;
		}

		/**
		 * For cases when the SpringConstraint is both collidable and only one of the
		 * two end particles are fixed, this value will dispose of collisions near the
		 * fixed particle, to correct for situations where the collision could never be
		 * resolved.
		 */	
		void setFixedEndLimit(int limit) {
			fixedEndLimit = limit;
		}
		
		int getFixedEndLimit() {
			return fixedEndLimit;
		}

		/** @return the average mass of the two connected particles */
		@Override
		public int getMass() {
			return (particle1.getMass() + particle2.getMass()) >> 1; 
		}
		
		/** @return the average elasticity of the two connected particles */
		@Override
		public int getElasticity() {
			return (particle1.getElasticity() + particle2.getElasticity()) >> 1; 
		}
		
		/** @return the average friction of the two connected particles */
		@Override
		public int getFriction() {
			return (particle1.getFriction() + particle2.getFriction()) >> 1; 
		}
		
		/** @return the average velocity of the two connected particles */
		@Override
		public Vector2f getVelocity(){
			Vector2f p1v =  particle1.getVelocity();
			Vector2f p2v =  particle2.getVelocity();
			
			avgVelocity.x = (p1v.x + p2v.x) >> 1;
			avgVelocity.y = (p1v.y + p2v.y) >> 1;
			return avgVelocity;
		}	
		
		@Override
		public void init() {
		}

	   /** @return the average inverse mass. */
		@Override
		int getInvMass() {
			if (particle1.isFixed && particle2.isFixed) {
				return 0;
			}
			
			return (int) (((long) FP.ONE << FP.FRACTION_BITS) / ((particle1.getMass() + particle2.getMass()) / 2));
			// above replaces return 1 / ((particle1.getMass() + particle2.getMass()) / 2);  
		}
		
		/** called only on collision */
		void updatePosition()  {
			// Since the Particle#isFixed() method is removed, and this cannot override what
			// is returned, we need to update it each iteration
			isFixed = parent.getFixed();
			
			Vector2f center = parent.getCenter();
			position.x = center.x;
			position.y = center.y;
			
			setWidth(scaleToLength 
					? (int) (((long) parent.getCurrLength() * getRectScale()) >> FP.FRACTION_BITS)
					: (int) (((long) parent.getRestLength() * getRectScale()) >> FP.FRACTION_BITS));
// above replaces			setWidth((scaleToLength) ? 
//					parent.getCurrLength() * getRectScale() : 
//					parent.getRestLength() * getRectScale());
			setHeight(getRectHeight());
			setRotation(parent.getRotation());
		}
		
		@Override	
		void respondToCollision(Collision collision, Vector2f mtd, Vector2f velocity, Vector2f n,
				int d, int o) {
			addCollisionEvent(collision);
			if (getFixed() || !collision.you.isSolid) {
				return;
			}
			
			int t = getContactPointParam(collision.you);
			int c1 = (FP.ONE - t);
			int c2 = t;
			
			// if one is fixed then move the other particle the entire way out of 
			// collision. also, dispose of collisions at the sides of the collision particle. The higher
			// the fixedEndLimit value, the more of the collision particle not be effected by collision. 
			if (particle1.isFixed) {
				if (c2 <= getFixedEndLimit()) {
					return;
				}
				
				lambda.x = (int) (((long) mtd.x << FP.FRACTION_BITS) / c2);
				// above replaces lambda.x = mtd.x / c2;
				
				lambda.y = (int) (((long) mtd.y << FP.FRACTION_BITS) / c2);
				// above replaces lambda.y = mtd.y / c2;
				
				particle2.position.x += lambda.x;
				particle2.position.y += lambda.y;
				particle2.setVelocity(velocity);
			} else if (particle2.isFixed) {
				if (c1 <= getFixedEndLimit()) {
					return;
				}

				lambda.x = (int) (((long) mtd.x << FP.FRACTION_BITS) / c1);
				// above replaces lambda.x = mtd.x / c1;
				
				lambda.y = (int) (((long) mtd.y << FP.FRACTION_BITS) / c1);
				// above replaces lambda.y = mtd.y / c1;
				
				particle1.position.x += lambda.x;
				particle1.position.y += lambda.y;
				particle1.setVelocity(velocity);		

			// else both non fixed - move proportionally out of collision
			} else { 
				int c1squared = (int) (((long) c1 * c1) >> FP.FRACTION_BITS);
				int c2squared = (int) (((long) c2 * c2) >> FP.FRACTION_BITS);
				int denom = c1squared + c2squared;
				// above replaces float denom = (c1 * c1 + c2 * c2);
				
				if (denom == 0) {
					return;
				}
				
				lambda.x = (int) (((long) mtd.x << FP.FRACTION_BITS) / denom);
				// above replaces lambda.x = mtd.x / denom;
				
				lambda.y = (int) (((long) mtd.y << FP.FRACTION_BITS) / denom);
				// above replaces lambda.y = mtd.y / denom;
			
				particle1.position.x += (int) (((long) lambda.x * c1) >> FP.FRACTION_BITS);
				// above replaces particle1.position.x += lambda.x * c1;
				
				particle1.position.y += (int) (((long) lambda.y * c1) >> FP.FRACTION_BITS);
				// above replaces particle1.position.y += lambda.y * c1;
				
				particle2.position.x += (int) (((long) lambda.x * c2) >> FP.FRACTION_BITS);
				// above replaces particle2.position.x += lambda.x * c2;
				
				particle2.position.y += (int) (((long) lambda.y * c2) >> FP.FRACTION_BITS);
				// above replaces particle2.position.y += lambda.y * c2;
			
				// if collision is in the middle of collision particle set the velocity of both end 
				// particles
				if (t == FP.ONE_HALF) {
					particle1.setVelocity(velocity);
					particle2.setVelocity(velocity);
				
				// otherwise change the velocity of the particle closest to contact
				} else {
					Particle corrParticle = (t < FP.ONE_HALF) ? particle1 : particle2;
					corrParticle.setVelocity(velocity);
				}
			}
		}
		
		/**
		 * Given a point, returns a parameterized location on this collision particle. Note
		 * this is just treating the collision particle as if it were a line segment (AB).
		 */
		private int closestParamPoint(Vector2f point) {
			int segmentABx = particle2.position.x - particle1.position.x;
			int segmentABy = particle2.position.y - particle1.position.y;
			int tmpX = point.x - particle1.position.x;
			int tmpY = point.y - particle1.position.y;
			
			int dot = (int) (((long) segmentABx * tmpX) >> FP.FRACTION_BITS) + (int) (((long) segmentABy * tmpY) >> FP.FRACTION_BITS);
			// above replaces float dot = segmentABx * tmpX + segmentABy * tmpY;
			
			int dot2 = (int) (((long) segmentABx * segmentABx) >> FP.FRACTION_BITS) + (int) (((long) segmentABy * segmentABy) >> FP.FRACTION_BITS);
			// above replaces float dot2 = segmentABx * segmentABx + segmentABy * segmentABy;
			
			int t = (int) (((long) dot << FP.FRACTION_BITS) / dot2);
			// above replaces float t = dot / dot2;
			
			return FP.clamp(t, 0, FP.ONE);
		}

		/** 
		 * @return a contact location on this collision particle expressed as a parametric
		 * value in [0,1]
		 */
		private int getContactPointParam(Particle particle) {
			int t = 0;
			if (particle instanceof Circle)  {
				t = closestParamPoint(particle.position);
			} else if (particle instanceof Rectangle) {
				// go through the sides of the colliding rectangle as line segments
				int shortestIndex = 0;
				int paramList[] = new int[4];
				int shortestDistance = FP.MAX_VALUE;
				
				for (int i = 0; i < 4; ++i) {
					setCorners((Rectangle)particle, i);
					
					// check for closest points on collision particle to side of rectangle
					int d = closestPtSegmentSegment();
					if (d < shortestDistance) {
						shortestDistance = d;
						shortestIndex = i;
						paramList[i] = s;
					}
				}
				t = paramList[shortestIndex];
			}
			return t;
		}
		
		private void setCorners(Rectangle rectangle, int i) {
			int ae0_x = (int) (((long) rectangle.axes[0].x * rectangle.extents[0]) >> FP.FRACTION_BITS);
			// above replaces float ae0_x = rectangle.axes[0].x * rectangle.extents[0];
			
			int ae0_y = (int) (((long) rectangle.axes[0].y * rectangle.extents[0]) >> FP.FRACTION_BITS);
			// above replaces float ae0_y = rectangle.axes[0].y * rectangle.extents[0];
			
			int ae1_x = (int) (((long) rectangle.axes[1].x * rectangle.extents[1]) >> FP.FRACTION_BITS);
			// above replaces float ae1_x = rectangle.axes[1].x * rectangle.extents[1];
			
			int ae1_y = (int) (((long) rectangle.axes[1].y * rectangle.extents[1]) >> FP.FRACTION_BITS);
			// above replaces float ae1_y = rectangle.axes[1].y * rectangle.extents[1];
			
			int emx = ae0_x - ae1_x;
			int emy = ae0_y - ae1_y;
			int epx = ae0_x + ae1_x;
			int epy = ae0_y + ae1_y;
			
			if (i == 0) {
				// 0 and 1
				rca.x = rectangle.position.x - epx;
				rca.y = rectangle.position.y - epy;
				rcb.x = rectangle.position.x + emx;
				rcb.y = rectangle.position.y + emy;
			} else if (i == 1) {
				// 1 and 2
				rca.x = rectangle.position.x + emx;
				rca.y = rectangle.position.y + emy;
				rcb.x = rectangle.position.x + epx;
				rcb.y = rectangle.position.y + epy;
			} else if (i == 2) {
				// 2 and 3
				rca.x = rectangle.position.x + epx;
				rca.y = rectangle.position.y + epy;
				rcb.x = rectangle.position.x - emx;
				rcb.y = rectangle.position.y - emy;
			} else if (i == 3) {
				// 3 and 0
				rca.x = rectangle.position.x - emx;
				rca.y = rectangle.position.y - emy;
				rcb.x = rectangle.position.x - epx;
				rcb.y = rectangle.position.y - epy;
			}
		}
		
		/** pp1-pq1 will be the collision particle line segment on which we need parameterized s. */
		private int closestPtSegmentSegment() {
			Vector2f pp2 = rca;
			Vector2f pq2 = rcb;
			
			int d1x = particle2.position.x - particle1.position.x;
			int d1y = particle2.position.y - particle1.position.y;
			int d2x = pq2.x - pp2.x;
			int d2y = pq2.y - pp2.y;
			int rX = particle1.position.x - pp2.x;
			int rY = particle1.position.y - pp2.y;
		
			int t;
			
			int a = (int) (((long) d1x * d1x) >> FP.FRACTION_BITS) + (int) (((long) d1y * d1y) >> FP.FRACTION_BITS);
			// above replaces int a = d1x * d1x + d1y * d1y;
			
			int e = (int) (((long) d2x * d2x) >> FP.FRACTION_BITS) + (int) (((long) d2y * d2y) >> FP.FRACTION_BITS);
			// above replaces int e = d2x * d2x + d2y * d2y;
			
			int f = (int) (((long) d2x * rX) >> FP.FRACTION_BITS) + (int) (((long) d2y * rY) >> FP.FRACTION_BITS);
			// above replaces int f = d2x * rX + d2y * rY;
			
			int c = (int) (((long) d1x * rX) >> FP.FRACTION_BITS) + (int) (((long) d1y * rY) >> FP.FRACTION_BITS);
			// above replaces int c = d1x * rX + d1y * rY;
			
			int b = (int) (((long) d1x * d2x) >> FP.FRACTION_BITS) + (int) (((long) d1y * d2y) >> FP.FRACTION_BITS);
			// above replaces int b = d1x * d2x + d1y * d2y;
			
			int denom = (int) (((long) a * e) >> FP.FRACTION_BITS) - (int) (((long) b * b) >> FP.FRACTION_BITS);
			// above replaces int denom = a * e - b * b;
			
			if (denom != 0) {
				s = FP.clamp((int) (((long) b * f) >> FP.FRACTION_BITS) - (int) (((long) c * e) >> FP.FRACTION_BITS), 0, FP.ONE);
				// above replaces s = FP.clamp((b * f - c * e) / denom, 0, FP.ONE);
			} else {
				s = FP.ONE_HALF; // give the midpoint for parallel lines
			}
			
			t = (int) (((long) (((int) (((long) b * s) >> FP.FRACTION_BITS)) + f) << FP.FRACTION_BITS) / e);
			// above replaces t = (b * s + f) / e;
			 
			if (t < 0) {
				t = 0;
				int negCdivByA = (int) (((long) -c << FP.FRACTION_BITS) / a); // replaces -c / a
			 	s = FP.clamp(negCdivByA, 0, FP.ONE);
			} else if (t > 0) {
			 	t = 1;
			 	int bMinusCdivByA = (int) (((long) (b - c) << FP.FRACTION_BITS) / a); // replaces (b - c) / a
			 	s = FP.clamp(bMinusCdivByA, 0, FP.ONE);
			}
			
			int c1x = particle1.position.x + ((int) (((long) d1x * s) >> FP.FRACTION_BITS));
			// above replaces int c1x = particle1.position.x + (d1x * s);
			
			int c1y = particle1.position.y + ((int) (((long) d1y * s) >> FP.FRACTION_BITS));
			// above replaces int c1y = particle1.position.y + (d1y * s);
			
			int c2x = pp2.x + ((int) (((long) d2x * t) >> FP.FRACTION_BITS));
			// above replaces int c2x = pp2.x + (d2x * t);
			
			int c2y = pp2.y + ((int) (((long) d2y * t) >> FP.FRACTION_BITS));
			// above replaces int c2y = pp2.y + (d2y * t);
			
			int c1minusc2x = c1x - c2x;
			int c1minusc2y = c1y - c2y;
			
			return (int) (((long) c1minusc2x * c1minusc2x) >> FP.FRACTION_BITS) + (int) (((long) c1minusc2y * c1minusc2y) >> FP.FRACTION_BITS);
			// above replaces return c1minusc2.x * c1minusc2.x + c1minusc2.y * c1minusc2.y;
		}
	}
	
}