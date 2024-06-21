extern crate piston_window;

use piston_window::*;

const GRAVITY: f64 = 9.8;

const WIDTH: i32 = 800;
const HEIGHT: i32 = 600;
const CELL_SIZE: i32 = 10;

#[derive(Clone)]
struct Cell {
    objects: Vec<i64>,
}

struct Grid {
    width: i32,
    height: i32,
    cell_size: i32,
    cells: Vec<Vec<Cell>>,
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
        }
    }
    fn get(&self, x: i32, y: i32) -> &Cell {
        &self.cells[(x as f64 / CELL_SIZE as f64).ceil() as usize][(y as f64 / CELL_SIZE as f64).ceil() as usize]
    }
    fn reset(&mut self) {
        for x in 0..self.width {
            for y in 0..self.height {
                self.cells[x as usize][y as usize].objects.clear();
            }
        }
    }
    fn add_obj(&mut self, obj: Circle, obj_id: i64) {
        let (x, y) = obj.find_grid_pos(self.cell_size);
        self.cells[x as usize][y as usize].objects.push(obj_id);
    }
    fn check_collisions(&mut self) {
        
    }
}

#[derive(Clone, Copy)]
struct PhysicsInfo {
    pos: Double,
    vel: Double,
    acc: Double,
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
        if self.touching_ground() {
            self.pinfo.acc.y = 0.0;
            self.pinfo.vel.y = 0.0;
        } else {
            self.pinfo.acc.y = GRAVITY;
            self.pinfo.vel.y += self.pinfo.acc.y * dt;
        }
        self.pinfo.pos.y += self.pinfo.vel.y * dt;
    }

    fn find_grid_pos(&self, cell_size: i32) -> (i32, i32) {
        let x = (self.pinfo.pos.x / cell_size as f64) as i32;
        let y = (self.pinfo.pos.y / cell_size as f64) as i32;
        (x, y)
    }

    fn touching_ground(&self) -> bool {
        self.pinfo.pos.y + self.radius >= HEIGHT as f64
    }

    fn check_collision(&self, other: &Circle) -> bool {
        let dx = self.pinfo.pos.x - other.pinfo.pos.x;
        let dy = self.pinfo.pos.y - other.pinfo.pos.y;
        let distance = (dx * dx + dy * dy).sqrt();
        distance <= self.radius + other.radius
    }

    fn fix_collision(&mut self, other: &mut Circle) {
        let dx = self.pinfo.pos.x - other.pinfo.pos.x;
        let dy = self.pinfo.pos.y - other.pinfo.pos.y;
        let distance = (dx * dx + dy * dy).sqrt();
        let overlap = self.radius + other.radius - distance;
        let ratio = overlap / distance;
        self.pinfo.pos.x += dx * ratio / 2.0;
        self.pinfo.pos.y += dy * ratio / 2.0;
        other.pinfo.pos.x -= dx * ratio / 2.0;
        other.pinfo.pos.y -= dy * ratio / 2.0;
    }
}

fn main() {
    let mut window: PistonWindow = WindowSettings::new("Rusty Physics", [WIDTH as u32, HEIGHT as u32])
        .exit_on_esc(true)
        .build()
        .unwrap();

    let mut grid = Grid::new(WIDTH, HEIGHT, CELL_SIZE);
    let mut circles: Vec<Circle> = Vec::new();
    circles.push(Circle {
        radius: 10.0,
        pinfo: PhysicsInfo {
            pos: Double {
                x: WIDTH as f64 / 2.0,
                y: HEIGHT as f64 / 2.0,
            },
            vel: Double { x: 0.0, y: 0.0 },
            acc: Double { x: 0.0, y: 0.0 },
        },
    });

    while let Some(event) = window.next() {
        window.draw_2d(&event, |context, graphics, _| {
            clear([1.0; 4], graphics);

            let mut i: i64 = 0;
            for circle in &mut circles {
                grid.reset();
                circle.update(1.0 / 60.0);
                grid.add_obj(*circle, i);
                grid.check_collisions();
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
        });
    }
}