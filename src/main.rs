extern crate piston_window;

use piston_window::*;
use rand;

const GRAVITY: f64 = 9.8;

const WIDTH: i32 = 800;
const HEIGHT: i32 = 600;
const CELL_SIZE: i32 = 10;
const SPEED_FACTOR: f64 = 3.0;
const AIR_RESISTANCE: f64 = 0.05;
const COLLIDE_LOSS: f64 = 0.4;

#[derive(Clone)]
struct Cell {
    objects: Vec<i64>,
}

struct Grid {
    width: i32,
    height: i32,
    cell_size: i32,
    cells: Vec<Vec<Cell>>,
    num_cells_x: i32,
    num_cells_y: i32,
}

impl Grid {
    fn new(width: i32, height: i32, cell_size: i32) -> Grid {
        let num_cells_x = (width as f64 / cell_size as f64).ceil() as i32;
        let num_cells_y = (height as f64 / cell_size as f64).ceil() as i32;
        let cells = vec![vec![Cell { objects: Vec::new() }; num_cells_y as usize]; num_cells_x as usize];
        Grid {
            width,
            height,
            cell_size,
            cells,
            num_cells_x,
            num_cells_y,
        }
    }
    fn get_cell_here(&self, x: i32, y: i32) -> &Cell {
        &self.cells[(x as f64 / CELL_SIZE as f64).floor() as usize][(y as f64 / CELL_SIZE as f64).floor() as usize]
    }
    fn reset(&mut self) {
        for x in 0..self.num_cells_x {
            for y in 0..self.num_cells_y {
                self.cells[x as usize][y as usize].objects.clear();
            }
        }
    }
    fn add_obj(&mut self, obj: Circle, obj_id: i64) {
        let (x, y) = obj.find_grid_pos(self.cell_size);
        self.cells[x as usize][y as usize].objects.push(obj_id);
    }
    fn check_collisions(&mut self, circles: &mut Vec<Circle>) {
        fn fix_collision(circles: &mut Vec<Circle>, i1: usize, i2: usize) {
            let dx = circles[i1].pinfo.pos.x - circles[i2].pinfo.pos.x;
            let dy = circles[i1].pinfo.pos.y - circles[i2].pinfo.pos.y;
            let distance = (dx * dx + dy * dy).sqrt();
            let overlap = circles[i1].radius + circles[i2].radius - distance;
            let ratio = overlap / distance;
            {
                let obj1 = &mut circles[i1];
                obj1.pinfo.pos.x += dx * ratio / 2.0;
                obj1.pinfo.pos.y += dy * ratio / 2.0;
                let normal = Double { x: dx / distance, y: dy / distance };
                let dp_normal = obj1.pinfo.vel.x * normal.x + obj1.pinfo.vel.y * normal.y;
                obj1.pinfo.vel.x = dp_normal * normal.x;
                obj1.pinfo.vel.y = dp_normal * normal.y;
                obj1.pinfo.acc.x = 0.0;
                obj1.pinfo.acc.y = 0.0;
            }
            {
                let obj2 = &mut circles[i2];
                obj2.pinfo.pos.x -= dx * ratio / 2.0;
                obj2.pinfo.pos.y -= dy * ratio / 2.0;
                let normal = Double { x: -dx / distance, y: -dy / distance };
                let dp_normal = obj2.pinfo.vel.x * normal.x + obj2.pinfo.vel.y * normal.y;
                obj2.pinfo.vel.x = -dp_normal * normal.x;
                obj2.pinfo.vel.y = -dp_normal * normal.y;
                obj2.pinfo.acc.x = 0.0;
                obj2.pinfo.acc.y = 0.0;
            }
        }

        struct Collision {
            obj1: i64,
            obj2: i64,
        }

        let mut collisions: Vec<Collision> = Vec::new();

        for x in 0..self.num_cells_x {
            for y in 0..self.num_cells_y {
                for obj_id in &self.cells[x as usize][y as usize].objects {
                    let obj = circles[*obj_id as usize];
                    let (x, y) = circles[*obj_id as usize].find_grid_pos(self.cell_size);
                    for i in -1..2 {
                        for j in -1..2 {
                            if x + i >= 0 && x + i < self.num_cells_x && y + j >= 0 && y + j < self.num_cells_y {
                                for other_obj_id in &self.cells[(x + i) as usize][(y + j) as usize].objects {
                                    if *other_obj_id != *obj_id {
                                        let other_obj = circles[*other_obj_id as usize];
                                        if obj.check_collision(&other_obj) {
                                            collisions.push(Collision {
                                                obj1: *obj_id,
                                                obj2: *other_obj_id,
                                            });
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        for collision in collisions {
            fix_collision(circles, collision.obj1 as usize, collision.obj2 as usize);
        }
    }
}

#[derive(Clone, Copy)]
struct PhysicsInfo {
    pos: Double,
    vel: Double,
    acc: Double,
    mas: f64,
}

#[derive(Clone, Copy)]
struct Double {
    x: f64,
    y: f64,
}

#[derive(Clone, Copy)]
struct Circle {
    radius: f64,
    pinfo: PhysicsInfo,
}

impl Circle {
    fn update(&mut self, dt: f64) {
        fn opposite_sign(x: f64) -> f64 {
            if x > 0.0 {
                -1.0
            } else {
                1.0
            }
        }

        if self.pinfo.pos.x + self.radius + 1.0 >= WIDTH as f64 {
            self.pinfo.vel.x = -self.pinfo.vel.x + opposite_sign(-self.pinfo.vel.x) * COLLIDE_LOSS * self.pinfo.vel.x;
            self.pinfo.acc.x = -self.pinfo.acc.x + opposite_sign(-self.pinfo.acc.x) * COLLIDE_LOSS * self.pinfo.acc.x;
        }
        if self.pinfo.pos.x - self.radius - 1.0 <= 0.0 {
            self.pinfo.vel.x = -self.pinfo.vel.x + opposite_sign(-self.pinfo.vel.x) * COLLIDE_LOSS * self.pinfo.vel.x;
            self.pinfo.acc.x = -self.pinfo.acc.x + opposite_sign(-self.pinfo.acc.x) * COLLIDE_LOSS * self.pinfo.acc.x;
        }
        if GRAVITY == 0.0 && self.pinfo.pos.y + self.radius + 1.0 >= HEIGHT as f64 {
            self.pinfo.vel.y = -self.pinfo.vel.y + opposite_sign(-self.pinfo.vel.y) * COLLIDE_LOSS * self.pinfo.vel.y;
            self.pinfo.acc.y = -self.pinfo.acc.y + opposite_sign(-self.pinfo.acc.y) * COLLIDE_LOSS * self.pinfo.acc.y;
        }
        if self.pinfo.pos.y - self.radius - 1.0 <= 0.0 {
            self.pinfo.vel.y = -self.pinfo.vel.y + opposite_sign(-self.pinfo.vel.y) * COLLIDE_LOSS * self.pinfo.vel.y;
            self.pinfo.acc.y = -self.pinfo.acc.y + opposite_sign(-self.pinfo.acc.y) * COLLIDE_LOSS * self.pinfo.acc.y;
        }

        if GRAVITY > 0.0 {
            self.pinfo.acc.y = GRAVITY;
            if self.touching_ground() {
                self.pinfo.vel.y = -self.pinfo.vel.y + opposite_sign(-self.pinfo.vel.y) * COLLIDE_LOSS * self.pinfo.vel.y;
            }
        }

        self.pinfo.acc.x += (opposite_sign(self.pinfo.acc.x) * AIR_RESISTANCE * self.pinfo.vel.x * self.pinfo.vel.x) * dt;
        self.pinfo.acc.y += (opposite_sign(self.pinfo.acc.y) * AIR_RESISTANCE * self.pinfo.vel.y * self.pinfo.vel.y) * dt;

        self.pinfo.vel.y += (self.pinfo.acc.y) * dt;
        self.pinfo.vel.x += (self.pinfo.acc.x) * dt;

        self.pinfo.vel.x *= 1.0 - AIR_RESISTANCE * dt;

        self.pinfo.pos.y += self.pinfo.vel.y * dt;
        self.pinfo.pos.x += self.pinfo.vel.x * dt;

        self.pinfo.pos.x = f64::max(f64::min(self.pinfo.pos.x, (WIDTH as f64 - 1.0 - self.radius) as f64), self.radius);
        self.pinfo.pos.y = f64::max(f64::min(self.pinfo.pos.y, (HEIGHT as f64 - 1.0 - self.radius) as f64), self.radius);
    }

    fn find_grid_pos(&self, cell_size: i32) -> (i32, i32) {
        let x = (self.pinfo.pos.x / cell_size as f64).floor() as i32;
        let y = (self.pinfo.pos.y / cell_size as f64).floor() as i32;
        (x, y)
    }

    fn touching_ground(&self) -> bool {
        self.pinfo.pos.y + self.radius + 1.0 >= HEIGHT as f64
    }

    fn check_collision(&self, other: &Circle) -> bool {
        let dx = self.pinfo.pos.x - other.pinfo.pos.x;
        let dy = self.pinfo.pos.y - other.pinfo.pos.y;
        let distance = (dx * dx + dy * dy).sqrt();
        distance <= self.radius + other.radius
    }
}

fn main() {
    let mut window: PistonWindow = WindowSettings::new("Rusty Physics", [WIDTH as u32, HEIGHT as u32])
        .exit_on_esc(true)
        .build()
        .unwrap();

    let mut grid = Grid::new(WIDTH, HEIGHT, CELL_SIZE);
    let mut circles: Vec<Circle> = Vec::new();

    for _ in 0..30 {
        circles.push(Circle {
            radius: 10.0,
            pinfo: PhysicsInfo {
                pos: Double {
                    x: WIDTH as f64 * rand::random::<f64>(),
                    y: HEIGHT as f64 * rand::random::<f64>(),
                },
                vel: Double { x: 0.0, y: 0.0 },
                acc: Double { x: rand::random::<f64>() * 5.0, y: rand::random::<f64>() * 5.0 },
                mas: 1.0,
            },
        });
    }

    while let Some(event) = window.next() {
        window.draw_2d(&event, |context, graphics, _| {
            clear([1.0; 4], graphics);
            grid.reset();

            let mut i: i64 = 0;
            for circle in &mut circles {
                circle.update(1.0 / 60.0 * SPEED_FACTOR);
                grid.add_obj(*circle, i);
                ellipse(
                    [1.0, 0.0, 0.0, 1.0],
                    [
                        circle.pinfo.pos.x - circle.radius,
                        circle.pinfo.pos.y - circle.radius,
                        circle.radius * 2.0,
                        circle.radius * 2.0,
                    ],
                    context.transform,
                    graphics,
                );

                i += 1;
            }
            grid.check_collisions(&mut circles);
        });
    }
}