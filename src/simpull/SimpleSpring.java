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
	
/**
 * A Spring-like constraint that connects two particles
 */
public strictfp class SimpleSpring implements IConstraint {
	
	public Particle particle1;
	public Particle particle2;	

	float stiffness;
	float restLength;

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
			float stiffness/*= 0.5*/,
			boolean isCollidable/*= false*/,
			float rectHeight/*= 1*/,
			float rectScale/*= 1*/,
			boolean scaleToLength/*= false*/) {

		this.stiffness = stiffness;
		this.particle1 = particle1;
		this.particle2 = particle2;
		// if the two particles are at the same location, offset slightly
		if (particle1.position.x == particle2.position.x && particle1.position.y == particle2.position.y) {
			particle2.position.x += 0.0001;
		}
		restLength = getCurrLength();
		setCollidable(isCollidable, rectHeight, rectScale, scaleToLength);
	}
	
	public SimpleSpring(
			Particle particle1, 
			Particle particle2, 
			boolean isCollidable/*= false*/,
			float rectHeight/*= 1*/,
			float rectScale/*= 1*/,
			boolean scaleToLength/*= false*/) {
		this(particle1, particle2, 0f, isCollidable, rectHeight, rectScale, scaleToLength);
	}
	
	/**
	 * The rotational value created by the positions of the two particles attached to this
	 * SpringConstraint. You can use this property to in your own painting methods, along with the 
	 * center property. 
	 * 
	 * @returns the current rotation in radians
	 */			
	public float getRotation() {
		Vector2f diff = new Vector2f(particle1.position.x - particle2.position.x,
				particle1.position.y - particle2.position.y);
		return (float)Math.atan2(diff.y, diff.x);
	}
	
	/**
	 * The center position created by the relative positions of the two particles attached to this.
	 * 
	 * @returns A Vector2f representing the center of this
	 */			
	public Vector2f getCenter() {
		return new Vector2f((particle1.position.x + particle2.position.x) / 2f, 
				(particle1.position.y + particle2.position.y) / 2f);
	}
	
	/**
	 * If the isCollidable property is true, you can set the scale of the collidible area
	 * between the two attached particles. Valid values are from 0 to 1. If you set the value to 1, then
	 * the collision area will extend all the way to the two attached particles. Setting the value lower
	 * will result in an collision area that spans a percentage of that distance. Setting the value
	 * higher will cause the collision rectangle to extend past the two end particles.
	 */		 	
	public void setRectScale(float scale) {
		if (getCollisionParticle() == null) {
			return;
		}
		getCollisionParticle().setRectScale(scale);
	}
	
	public float getRectScale() {
		return getCollisionParticle().getRectScale();
	}
	
	/**
	 * Returns the length of the SpringConstraint, the distance between its two 
	 * attached particles.
	 */ 
	public float getCurrLength() {
		Vector2f diff = new Vector2f(particle1.position.x - particle2.position.x,
				particle1.position.y - particle2.position.y);
		return (float)Math.sqrt(diff.x * diff.x + diff.y * diff.y);
	}
	
	/**
	 * If the isCollidable property is true, you can set the height of the 
	 * collidible rectangle between the two attached particles. Valid values are greater 
	 * than 0. If you set the value to 10, then the collision rect will have a height of 10.
	 * The height is perpendicular to the line connecting the two particles
	 */	 
	public float getRectHeight() {
		return getCollisionParticle().getRectHeight();
	}
	
	public void setRectHeight(float h) {
		if (getCollisionParticle() == null) return;
		getCollisionParticle().setRectHeight(h);
	}			
	
	public float getRestLength() {
		return restLength;
	}
	
	/**
	 * The restLength property sets the length. This value will be
	 * the distance between the two particles unless their position is altered by external forces. 
	 * The SpringConstraint will always try to keep the particles this distance apart. Values must 
	 * be > 0.
	 */			
	public void setRestLength(float restLength) {
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
	public float getFixedEndLimit() {
		return collisionParticle.getFixedEndLimit();
	}	
			
	public void setFixedEndLimit(float fixedEndLimit) {
		if (collisionParticle == null) {
			return;
		}
		collisionParticle.setFixedEndLimit(fixedEndLimit);
	}
	
	public void setCollidable(boolean isCollidable, float rectHeight, 
			float rectScale, boolean scaleToLength/*=false*/) {
		this.isCollidable = isCollidable;
		collisionParticle = null;
		if (isCollidable) {
			collisionParticle = new SpringParticle(particle1, particle2, this, rectHeight, rectScale, scaleToLength);			
		}
		
		// FIXME: There is no call to collisionParticle.init() like there is when a constraint is added to a group and therefore some things will not work 
	}
	
	/** @return true if the passed particle is one of the two particles attached */		
	public boolean isConnectedTo(Particle particle) {
		return (particle == particle1 || particle == particle2);
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
		float deltaLength = getCurrLength();			
		float diff = (deltaLength - getRestLength()) 
				/ (deltaLength * (particle1.getInvMass() + particle2.getInvMass()));
		float diffStiff = diff * stiffness;
		Vector2f dmds = new Vector2f((particle1.position.x - particle2.position.x) * diffStiff,
				(particle1.position.y - particle2.position.y) * diffStiff);
			
		particle1.position.x -= dmds.x * particle1.getInvMass();
		particle1.position.y -= dmds.y * particle1.getInvMass();
		
		particle2.position.x += dmds.x * particle2.getInvMass();
		particle2.position.y += dmds.y * particle2.getInvMass();
	}		
	
	SpringParticle getCollisionParticle(){
		return collisionParticle;
	}
	
	/** This class is here to provide collision support for Spring instances where isCollidable is true. */
	class SpringParticle extends Rectangle {
		
		SimpleSpring parent;
		
		private Particle particle1;
		private Particle particle2;
		
		private Vector2f avgVelocity;
		private Vector2f lambda;
		private boolean scaleToLength;
		
		private Vector2f rca;
		private Vector2f rcb;
		private float s;
		
		private float rectScale;
		private float rectHeight;
		private float fixedEndLimit;
				
		public SpringParticle(
				Particle particle1, 
				Particle particle2, 
				SimpleSpring parent, 
				float rectHeight, 
				float rectScale,
				boolean scaleToLength) {
			super(0,0,0,0,0,false);
			this.particle1 = particle1;
			this.particle2 = particle2;
			
			lambda = new Vector2f(0,0);
			avgVelocity = new Vector2f(0,0);
			
			this.parent = parent;
			isFixed = parent.getFixed();
			this.rectScale = rectScale;
			this.rectHeight = rectHeight;
			this.scaleToLength = scaleToLength;
			
			fixedEndLimit = 0;
			rca = new Vector2f();
			rcb = new Vector2f();
		}
		
		void setRectScale(float s) {
			rectScale = s;
		}
		
		float getRectScale() {
			return rectScale;
		}
		
		void setRectHeight(float r) {
			rectHeight = r;
		}
		
		float getRectHeight() {
			return rectHeight;
		}

		/**
		 * For cases when the SpringConstraint is both collidable and only one of the
		 * two end particles are fixed, this value will dispose of collisions near the
		 * fixed particle, to correct for situations where the collision could never be
		 * resolved.
		 */	
		void setFixedEndLimit(float f) {
			fixedEndLimit = f;
		}
		
		float getFixedEndLimit() {
			return fixedEndLimit;
		}

		/** @return the average mass of the two connected particles */
		@Override
		public float getMass() {
			return (particle1.getMass() + particle2.getMass()) / 2; 
		}
		
		/** @return the average elasticity of the two connected particles */
		@Override
		public float getElasticity() {
			return (particle1.getElasticity() + particle2.getElasticity()) / 2; 
		}
		
		/** @return the average friction of the two connected particles */
		@Override
		public float getFriction() {
			return (particle1.getFriction() + particle2.getFriction()) / 2; 
		}
		
		/** @return the average velocity of the two connected particles */
		@Override
		public Vector2f getVelocity(){
			Vector2f p1v=  particle1.getVelocity();
			Vector2f p2v =  particle2.getVelocity();
			
			avgVelocity.x = (p1v.x + p2v.x) / 2f;
			avgVelocity.y = (p1v.y + p2v.y) / 2f;
			return avgVelocity;
		}	
		
		@Override
		public void init() {
		}

	   /** @return the average inverse mass. */
		@Override
		float getInvMass() {
			if (particle1.isFixed && particle2.isFixed) {
				return 0;
			}
			return 1 / ((particle1.getMass() + particle2.getMass()) / 2);  
		}
		
		@Override
		public strictfp void update(float dt2) {
			// Since the Particle#isFixed() method is removed, and this cannot override what
			// is returned, we need to update it each iteration
			isFixed = parent.getFixed();
			super.update(dt2);
		}
		
		/** called only on collision */
		void updatePosition()  {
			Vector2f center = parent.getCenter();
			position.x = center.x;
			position.y = center.y;
			
			setWidth((scaleToLength) ? 
					parent.getCurrLength() * getRectScale() : 
					parent.getRestLength() * getRectScale());
			setHeight(getRectHeight());
			setRotation(parent.getRotation());
		}
		
		@Override	
		void respondToCollision(Collision collision, Vector2f mtd, Vector2f velocity, Vector2f n,
				float d, int o) {
			addCollisionEvent(collision);
			if (getFixed() || !collision.you.isSolid) {
				return;
			}
			
			float t = getContactPointParam(collision.you);
			float c1 = (1 - t);
			float c2 = t;
			
			// if one is fixed then move the other particle the entire way out of 
			// collision. also, dispose of collisions at the sides of the collision particle. The higher
			// the fixedEndLimit value, the more of the collision particle not be effected by collision. 
			if (particle1.isFixed) {
				if (c2 <= getFixedEndLimit()) return;
				lambda.x = mtd.x / c2;
				lambda.y = mtd.y / c2;
				particle2.position.x += lambda.x;
				particle2.position.y += lambda.y;
				particle2.setVelocity(velocity);

			} else if (particle2.isFixed) {
				if (c1 <= getFixedEndLimit()) return;
				lambda.x = mtd.x / c1;
				lambda.y = mtd.y / c1;
				particle1.position.x += lambda.x;
				particle1.position.y += lambda.y;
				particle1.setVelocity(velocity);		

			// else both non fixed - move proportionally out of collision
			} else { 
				float denom = (c1 * c1 + c2 * c2);
				if (denom == 0) {
					return;
				}
				lambda.x = mtd.x / denom;
				lambda.y = mtd.y / denom;
			
				particle1.position.x += lambda.x * c1;
				particle1.position.y += lambda.y * c1;
				
				particle2.position.x += lambda.x * c2;
				particle2.position.y += lambda.y * c2;
			
				// if collision is in the middle of collision particle set the velocity of both end 
				// particles
				if (t == 0.5) {
					particle1.setVelocity(velocity);
					particle2.setVelocity(velocity);
				
				// otherwise change the velocity of the particle closest to contact
				} else {
					Particle corrParticle = (t < 0.5) ? particle1 : particle2;
					corrParticle.setVelocity(velocity);
				}
			}
		}
		
		/**
		 * Given a point, returns a parameterized location on this collision particle. Note
		 * this is just treating the collision particle as if it were a line segment (AB).
		 */
		private float closestParamPoint(Vector2f point) {
			Vector2f segmentAB = new Vector2f(particle2.position.x - particle1.position.x,
					particle2.position.y - particle1.position.y);
			Vector2f tmp = new Vector2f(point.x - particle1.position.x, point.y - particle1.position.y);
			float dot = segmentAB.x * tmp.x + segmentAB.y * tmp.y;
			float dot2 = segmentAB.x * segmentAB.x + segmentAB.y * segmentAB.y;
			float t = dot / dot2;
			return MathUtil.clamp(t, 0, 1);
		}

		/** 
		 * @return a contact location on this collision particle expressed as a parametric
		 * value in [0,1]
		 */
		private float getContactPointParam(Particle particle) {
			float t = 0f;
			if (particle instanceof Circle)  {
				t = closestParamPoint(particle.position);
			} else if (particle instanceof Rectangle) {
				// go through the sides of the colliding rectangle as line segments
				int shortestIndex = 0;
				float paramList[] = new float[4];
				float shortestDistance = Float.POSITIVE_INFINITY;
				
				for (int i = 0; i < 4; ++i) {
					setCorners((Rectangle)particle, i);
					
					// check for closest points on collision particle to side of rectangle
					float d = closestPtSegmentSegment();
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
		
		private void setCorners(Rectangle r, int i) {
			float rx = r.position.x;
			float ry = r.position.y;
			
			Vector2f[] axes = r.getAxes();
			float[] extents = r.getExtents();
			
			float ae0_x = axes[0].x * extents[0];
			float ae0_y = axes[0].y * extents[0];
			float ae1_x = axes[1].x * extents[1];
			float ae1_y = axes[1].y * extents[1];
			
			float emx = ae0_x - ae1_x;
			float emy = ae0_y - ae1_y;
			float epx = ae0_x + ae1_x;
			float epy = ae0_y + ae1_y;
			
			if (i == 0) {
				// 0 and 1
				rca.x = rx - epx;
				rca.y = ry - epy;
				rcb.x = rx + emx;
				rcb.y = ry + emy;
			} else if (i == 1) {
				// 1 and 2
				rca.x = rx + emx;
				rca.y = ry + emy;
				rcb.x = rx + epx;
				rcb.y = ry + epy;
			} else if (i == 2) {
				// 2 and 3
				rca.x = rx + epx;
				rca.y = ry + epy;
				rcb.x = rx - emx;
				rcb.y = ry - emy;
			} else if (i == 3) {
				// 3 and 0
				rca.x = rx - emx;
				rca.y = ry - emy;
				rcb.x = rx - epx;
				rcb.y = ry - epy;
			}
		}
		
		/** pp1-pq1 will be the collision particle line segment on which we need parameterized s. */
		private float closestPtSegmentSegment() {
			Vector2f pp1 = particle1.position;
			Vector2f pq1= particle2.position;
			Vector2f pp2= rca;
			Vector2f pq2 = rcb;
			
			Vector2f d1 = new Vector2f(pq1.x - pp1.x, pq1.y - pp1.y);
			Vector2f d2 = new Vector2f(pq2.x - pp2.x, pq2.y - pp2.y);
			Vector2f r = new Vector2f(pp1.x - pp2.x, pp1.y - pp2.y);
		
			float t;
			float a = d1.x * d1.x + d1.y * d1.y;
			float e = d2.x * d2.x + d2.y * d2.y;
			float f = d2.x * r.x + d2.y * r.y;
			float c = d1.x * r.x + d1.y * r.y;
			float b = d1.x * d2.x + d1.y * d2.y;
			float denom = a * e - b * b;
			
			if (denom != 0.0) {
				s = MathUtil.clamp((b * f - c * e) / denom, 0, 1);
			} else {
				s = 0.5f; // give the midpoint for parallel lines
			}
			t = (b * s + f) / e;
			 
			if (t < 0) {
				t = 0;
			 	s = MathUtil.clamp(-c / a, 0, 1);
			} else if (t > 0) {
			 	t = 1;
			 	s = MathUtil.clamp((b - c) / a, 0, 1);
			}
			 
			Vector2f c1 = new Vector2f(pp1.x + (d1.x * s), pp1.y + (d1.y * s));
			Vector2f c2 = new Vector2f(pp2.x + (d2.x * t), pp2.y + (d2.y * t));
			Vector2f c1minusc2 = new Vector2f(c1.x - c2.x, c1.y - c2.y);
			return c1minusc2.x * c1minusc2.x + c1minusc2.y * c1minusc2.y;
		}
	}
	
}