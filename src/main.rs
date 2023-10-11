extern crate piston_window;
extern crate rand;

use piston_window::*;
use rand::Rng;
use rayon::prelude::*;
use std::time::Instant;

//options
const USE_ADVANCED_COLLISION_DETECTION: bool = false;
const PERFORM_COLLISION_DETECTION: bool = false;

//physics constants
const GRAVITY: f64 = 9.81;
const WALL_DAMPENING: f64 = -0.8;
const SCALE_FACTOR: f64 = 100.0; //100 pizels per unit
                                 //1 unit equals 1 meter

/*
The drag force FF is given by:

F=−0.5×Cd×A×ρ×v2F=−0.5×Cd​×A×ρ×v2

Where:

    CdCd​ is the drag coefficient (depends on the object's shape).
    AA is the reference area of the object.
    ρρ is the fluid density.
    vv is the velocity.

For simplicity, we can lump many of these constants together:

F=−k×v2F=−k×v2

Where kk is some constant.

NOTE: currently we are going to use K and define drag value in the object defintition


*/

//note: need to define a unit mass and a unit size.

fn main() {
    let mut window: PistonWindow = WindowSettings::new("particle simulation", [340, 480])
        .exit_on_esc(true)
        .build()
        .unwrap();

    let mut particles: Vec<Particle> = Vec::new();

    //let mut particle1 = Particle::new([1.0,1.0], [0.2, 0.2], GRAVITY, 1.0);
    //particle.set_horiz_velocity(50.0);
    //particle.set_drag_k(0.09);
    //particles.push(particle);

    //initial height and width
    let height = window.size().height / SCALE_FACTOR;
    let width = window.size().width / SCALE_FACTOR;
    let mut particle = Particle::new([0.1, 0.1], [1.0, 1.0], GRAVITY, 10.0);
    particles.push(particle);

    
    let mut now = Instant::now(); //timer
    for _ in 0..4000 {
        let mut rng = rand::thread_rng();
        let position = [
            rng.gen_range(0.1..=width),  // x-coordinate range
            rng.gen_range(0.1..=height), // y-coordinate range
        ];
        let size = rng.gen_range(0.05..=0.15);
        let mut particle = Particle::new(position, [0.05, 0.05], GRAVITY, 1.0);
        particle.set_horiz_velocity(rng.gen_range(-25..25) as f64);
        particle.set_drag_k(0.19);
        particles.push(particle);

    }
    println!("finished generating particles: {:?}", now.elapsed()); //finish timer.
    while let Some(event) = window.next() {
        let frameTime = Instant::now();
        //shared info between renderer and physics
        let window_height = window.size().height;
        let window_width = window.size().width;

        if let Some(update_args) = event.update_args() {
            let delta_time = update_args.dt as f64;
            //update each particle
            // for object in &mut particles {
            //     object.update(delta_time, window_height, window_width); //this has the error cannot mutate immutable object
            // }
            let now = Instant::now();
            particles.par_iter_mut().for_each(|particle| {
                particle.update(delta_time, window_height, window_width);
            });
            println!("finished updating particles: {:?}", now.elapsed());
            handle_collisions(&mut particles);
        }

        if let Some(args) = event.render_args() {
            let drawTime = Instant::now();
            window.draw_2d(&event, |c, g, device| {
                clear([1.0; 4], g);

                //render each particle
                for object in &particles {
                    ellipse(
                        [1.0, 0.0, 0.0, 1.0], // Red color for the particle
                        [
                            object.position[0] * SCALE_FACTOR,
                            object.position[1] * SCALE_FACTOR,
                            object.size[0] * SCALE_FACTOR,
                            object.size[1] * SCALE_FACTOR,
                        ], // [x, y, w, h]
                        c.transform,
                        g,
                    );
                }
            });
            println!("ellipse draws completed in: {:?}", drawTime.elapsed());
        }
        println!("Frame rendered in: {:?}", frameTime.elapsed());
    }
}

const GRID_SIZE: usize = 1;

struct Grid {
    cells: Vec<Vec<Vec<usize>>>, // A 3D grid of particle indices
    cell_size: f64,
}

impl Grid {
    fn new(cell_size: f64) -> Self {
        Self {
            cells: vec![vec![Vec::new(); GRID_SIZE]; GRID_SIZE],
            cell_size,
        }
    }

    fn insert_particle(&mut self, p: &Particle, index: usize) {
        let x = (p.position[0] / self.cell_size) as usize;
        let y = (p.position[1] / self.cell_size) as usize;
        self.cells[y][x].push(index);
    }

    fn potential_collisions(&self, p: &Particle) -> Vec<usize> {
        let x = (p.position[0] / self.cell_size) as usize;
        let y = (p.position[1] / self.cell_size) as usize;

        let mut nearby_indices = Vec::new();
        for i in (x as isize - 1).max(0)..=(x as isize + 1).min(GRID_SIZE as isize - 1) {
            for j in (y as isize - 1).max(0)..=(y as isize + 1).min(GRID_SIZE as isize - 1) {
                nearby_indices.extend_from_slice(&self.cells[j as usize][i as usize]);
            }
        }
        nearby_indices
    }
}

fn handle_collisions(particles: &mut Vec<Particle>) {
    let now = Instant::now();
    if !PERFORM_COLLISION_DETECTION {
        println!("Collision Calculation Time: {:?}", now.elapsed());
        return;
    }
    if USE_ADVANCED_COLLISION_DETECTION {
        for i in 0..particles.len() {
            let (left, right) = particles.split_at_mut(i + 1);
            let particle1 = &mut left[i];

            for particle2 in right {
                if are_colliding(particle1, particle2) {
                    resolve_collision(particle1, particle2)
                }
            }
        }
    } else {
        // Assuming `particles` is a Vec<Particle>
        let mut potential_collisions = Vec::new();

        // Generate pairs of indices for potential collisions
        for i in 0..particles.len() {
            for j in i + 1..particles.len() {
                // Check some broad-phase condition here to fill `potential_collisions` if necessary
                // if some_broad_phase_condition(particles[i], particles[j]) {
                //     potential_collisions.push((i, j));
                // }
                potential_collisions.push((i, j));
            }
        }

        // Resolve the actual collisions using indices
        for i in 0..particles.len() {
            let (left, right) = particles.split_at_mut(i + 1); // split after the i-th element
            if let Some(p1) = left.last_mut() {
                for p2 in right.iter_mut() {
                    if are_colliding(p1, p2) {
                        resolve_collision(p1, p2);
                    }
                }
            }
        }
    }

    println!("Collision Calculation Time: {:?}", now.elapsed());
}

//particle collision detection
fn are_colliding(p1: &Particle, p2: &Particle) -> bool {
    let dx = p1.position[0] - p2.position[0];
    let dy = p1.position[1] - p2.position[1];
    let distance = (dx * dx + dy * dy).sqrt();
    distance < p1.size[0] / 2.0 + p2.size[0] / 2.0
}

fn resolve_collision(p1: &mut Particle, p2: &mut Particle) {
    // Vector between particle centers
    let dx = p1.position[0] - p2.position[0];
    let dy = p1.position[1] - p2.position[1];
    let distance = (dx * dx + dy * dy).sqrt();

    let collision_vector = [dx / distance, dy / distance];

    // Relative velocity
    let relative_velocity = [
        p1.velocity[0] - p2.velocity[0],
        p1.velocity[1] - p2.velocity[1],
    ];

    // Velocity along the normal (dot product)
    let velocity_along_normal =
        relative_velocity[0] * collision_vector[0] + relative_velocity[1] * collision_vector[1];

    // Don't resolve the collision if the particles are moving apart
    if velocity_along_normal > 0.0 {
        return;
    }

    // Calculate the impulse/scalar value for the collision
    let impulse = -2.0 * velocity_along_normal / (1.0 / p1.mass + 1.0 / p2.mass); // Assuming mass is a field of the Particle struct

    // Apply the impulse to each particle
    p1.velocity[0] += (impulse / p1.mass) * collision_vector[0];
    p1.velocity[1] += (impulse / p1.mass) * collision_vector[1];
    p2.velocity[0] -= (impulse / p2.mass) * collision_vector[0];
    p2.velocity[1] -= (impulse / p2.mass) * collision_vector[1];
}

#[derive(Clone)]
struct Particle {
    position: [f64; 2],
    velocity: [f64; 2],
    size: [f64; 2],
    gravity: f64,
    drag: f64,
    mass: f64,
}

impl Particle {
    pub fn new(position: [f64; 2], size: [f64; 2], gravity: f64, mass: f64) -> Self {
        Particle {
            position: position,
            velocity: [0.0, 0.0],
            size: size,
            gravity: gravity,
            drag: 0.0,
            mass,
        }
    }

    pub fn set_velocity(&mut self, vx: f64, vy: f64) {
        self.velocity = [vx, vy]
    }
    pub fn set_horiz_velocity(&mut self, vx: f64) {
        self.velocity[0] = vx
    }
    pub fn set_vert_velocity(&mut self, vy: f64) {
        self.velocity[1] = vy
    }
    pub fn set_drag_k(&mut self, value: f64) {
        self.drag = value
    }

    pub fn update(&mut self, dt: f64, window_height: f64, window_width: f64) {
        //positions

        //apply gravity (downward force)
        self.velocity[1] += self.gravity * dt;
        self.position[1] += self.velocity[1] * dt;

        //horiz movement
        self.position[0] += self.velocity[0] * dt;

        
        //collisions

        //ground collision
        if self.position[1] >= window_height / scale_factor - self.size[1] {
            self.position[1] = window_height / scale_factor - self.size[1];
            self.velocity[1] = WALL_DAMPENING * self.velocity[1]; // 0.8 is a damping factor for bounce
        }
        //side collision
        if self.position[0] <= 0.0 || self.position[0] >= window_width / scale_factor {
            self.velocity[0] = WALL_DAMPENING * self.velocity[0]; // Reflect and dampen horizontal velocity
        }

        //ceiling collision
        if self.position[1] <= 0.0 {
            self.position[1] = 0.0;
            self.velocity[1] = WALL_DAMPENING * self.velocity[1];
        }

            let k: f64 = self.drag; // Adjust based on how strong you want the drag to be
            self.velocity[0] -= k * self.velocity[0].abs() * self.velocity[0] * dt;
            self.velocity[1] -= k * self.velocity[1].abs() * self.velocity[1] * dt;
      
    }
}
