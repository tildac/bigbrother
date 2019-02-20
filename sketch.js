
// Click mouse to add boids into the system 

let flock;
let img;
let a;
let r, g, b;

function setup() {
  createCanvas(windowWidth,windowHeight);
  stroke(255);
  a = height / 2;

  r = random(255);
  g = random(255);
  b = random(255);

  flock = new Flock();
  // Add an initial set of boids into the system
  for (let i = 0; i < 20; i++) {
    let b = new Boid(width/2,height/2);
    flock.addBoid(b);
  }
}

function draw() {
    background(178, 59, 30);
    flock.run();
    line(0, a, width, a);
    a = a - 0.5;
    if (a < 0) {
    a = height;
    }
    image(img, 650, 200, img.width*2, img.height*2);
    
    if(mouseIsPressed){
        image(img2, mouseX, mouseY, 100, 100);

    strokeWeight(2);
    stroke(r, g, b);
    fill(r, g, b, 127);
    ellipse(950, 480, 700, 700);
}
}

function mousePressed() {
    // Check if mouse is inside the circle
    let d = dist(mouseX, mouseY, 950, 480);
    if (d < 300) {
      // Pick new random color values
      r = random(255);
      g = random(255);
      b = random(255);
    }
}

function preload(){
    img = loadImage('images/panopticon.gif')
    img2 = loadImage("images/shutter.png");

}

// Click mouse in order to add a new boid into the System
function mouseDragged() {
  flock.addBoid(new Boid(mouseX,mouseY));
}

// Flock object
// Does very little, simply manages the array of all the boids

class Flock {

    constructor(){
        //An array for all the boids
        this.boids = []; //This initializes the array 
    }

run() {
    for(let boid of this.boids){
        boid.run(this.boids); //Passing the entire list of boids to each boid individually
    }
}

addBoid(b) {
    this.boids.push(b);
}
}

class Boid {
    constructor(x,y){
        this.acceleration = createVector(0,0);
        this.velocity = createVector(random(-1,1), random(-1,1));
        this.position = createVector(x,y);
        this.r = 1.0;
        this.maxspeed = 3; //Maximum speed
        this.maxforce = 0.05; //Maximum sterring force
    }

    run(boids) {
        this.flock(boids);
        this.update();
        this.borders();
        this.render();
      }
    
      applyForce(force) {
        this.acceleration.add(force);
      }
    
      // We accumulate a new acceleration each time based on three rules
      flock(boids) {
        let sep = this.separate(boids); // Separation - and then continue by weighing these forces
        let ali = this.align(boids); // Alignment
        let coh = this.cohesion(boids); // Cohesion
        sep.mult(1.5);
        ali.mult(1.0);
        coh.mult(1.0);
        this.applyForce(sep); // Here this adds the force vectors to acceleration
        this.applyForce(ali);
        this.applyForce(coh);
      }
    
      update() {
        this.velocity.add(this.acceleration);// Update velocity
        this.velocity.limit(this.maxspeed);
        this.position.add(this.velocity);// Limit speed
        this.acceleration.mult(0);// Reset accelertion to 0 each cycle
      }
    
      // A method that calculates and applies a steering force towards a target
      // STEER = DESIRED MINUS VELOCITY
      seek(target) {
        let desired = p5.Vector.sub(target, this.position); // A vector pointing from the location to the target
        // Normalize desired and scale to maximum speed
        desired.normalize();
        desired.mult(this.maxspeed);
        // Steering = Desired minus Velocity
        let steer = p5.Vector.sub(desired, this.velocity);
        steer.limit(this.maxforce); // Limit to maximum steering force
        return steer;
      }
    
      render() {
        // Draw a triangle rotated in the direction of velocity
        let theta = this.velocity.heading() + radians(90);
        noFill();
        stroke('white');
        push();
        translate(this.position.x, this.position.y);
        rotate(theta);
        beginShape();
        vertex(20, 20);
        vertex(40, 20);
        vertex(40, 40);
        vertex(60, 40);
        vertex(60, 60);
        vertex(20, 60);
        endShape(CLOSE);
        pop();
      }
    
      // Wraparound
      borders() {
        if (this.position.x < -this.r) this.position.x = width + this.r;
        if (this.position.y < -this.r) this.position.y = height + this.r;
        if (this.position.x > width + this.r) this.position.x = -this.r;
        if (this.position.y > height + this.r) this.position.y = -this.r;
      }
    
      // Separation - this method checks for nearby boids and steers away
      separate(boids) {
        let desiredseparation = 25.0;
        let steer = createVector(0, 0);
        let count = 0;
        for (let i = 0; i < boids.length; i++) { // For every boid in the system, check if it's too close
          let d = p5.Vector.dist(this.position, boids[i].position);
          if ((d > 0) && (d < desiredseparation)) { // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
            let diff = p5.Vector.sub(this.position, boids[i].position); // Calculate vector pointing away from neighbor
            diff.normalize();
            diff.div(d); // Weight by distance
            steer.add(diff);
            count++; // Keep track of how many
          }
        }
       
        if (count > 0) {   // Average -- divide by how many
          steer.div(count);
        }
           
        if (steer.mag() > 0) {  // As long as the vector is greater than 0
          steer.normalize();
          steer.mult(this.maxspeed);
          steer.sub(this.velocity);
          steer.limit(this.maxforce);
        }
        return steer;
      }
    
      // Alignment - for every nearby boid in the system, calculate the average velocity
      align(boids) {
        let neighbordist = 25;
        let sum = createVector(0, 0);
        let count = 0;
        for (let i = 0; i < boids.length; i++) {
          let d = p5.Vector.dist(this.position, boids[i].position);
          if ((d > 0) && (d < neighbordist)) {
            sum.add(boids[i].velocity);
            count++;
          }
        }
        if (count > 0) {
          sum.div(count);
          sum.normalize();
          sum.mult(this.maxspeed);
          let steer = p5.Vector.sub(sum, this.velocity);
          steer.limit(this.maxforce);
          return steer;
        } else {
          return createVector(0, 0);
        }
      }
    
      // Cohesion - For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
      cohesion(boids) {
        let neighbordist = 50;
        let sum = createVector(0, 0); // Start with empty vector to accumulate all locations
        let count = 0;
        for (let i = 0; i < boids.length; i++) {
          let d = p5.Vector.dist(this.position, boids[i].position);
          if ((d > 0) && (d < neighbordist)) {
            sum.add(boids[i].position); // Add location
            count++;
          }
        }
        if (count > 0) {
          sum.div(count);
          return this.seek(sum); // Steer towards the location
        } else {
          return createVector(0, 0);
        }
      }
    }
