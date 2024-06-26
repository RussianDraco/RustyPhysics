extern crate piston_window;
extern crate find_folder;

use piston_window::*;
use rand;

use std::f64::consts::PI;

const GRAVITY: f64 = 9.8;
const WIDTH: i32 = 800;
const HEIGHT: i32 = 600;
const CELL_SIZE: i32 = 50;
const SPEED_FACTOR: f64 = 3.0;
const AIR_RESISTANCE: f64 = 0.05;
const COLLIDE_LOSS: f64 = 0.4;
const SPRING_CONST: f64 = 0.1;
const DAMP_CONST: f64 = 0.05;
const DEFAULT_RADIUS: f64 = 10.0;
const RADIUS_MIN: f64 = 10.0;
const RADIUS_MAX: f64 = 20.0;
const DEFAULT_COLOR: [f32; 4] = [1.0, 0.0, 0.0, 1.0];

const CIRCLE_NUMBER: usize = 30;

#[derive(Clone)]
struct Cell {
    objects: Vec<i64>,
}

struct Grid {
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
            cell_size,
            cells,
            num_cells_x,
            num_cells_y,
        }
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
            let brownian: bool = circles[i1].pinfo.pos.dist(circles[i2].pinfo.pos) < 1.0;

            {
                let obj1 = &mut circles[i1];
                if brownian {
                    obj1.pinfo.pos.x += rand::random::<f64>() * 2.0 - 1.0;
                    obj1.pinfo.pos.y += rand::random::<f64>() * 2.0 - 1.0;
                } else {
                    obj1.pinfo.pos.x += dx * ratio / 2.0;
                    obj1.pinfo.pos.y += dy * ratio / 2.0;
                    let normal = Double { x: dx / distance, y: dy / distance };
                    let dp_normal = obj1.pinfo.vel.x * normal.x + obj1.pinfo.vel.y * normal.y;
                    obj1.pinfo.vel.x = dp_normal * normal.x;
                    obj1.pinfo.vel.y = dp_normal * normal.y;
                    obj1.pinfo.acc.x = 0.0;
                    obj1.pinfo.acc.y = 0.0;
                }
            }
            {
                let obj2 = &mut circles[i2];
                if brownian {
                    obj2.pinfo.pos.x += rand::random::<f64>() * 2.0 - 1.0;
                    obj2.pinfo.pos.y += rand::random::<f64>() * 2.0 - 1.0;
                } else {
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
}

#[derive(Clone, Copy)]
struct Double {
    x: f64,
    y: f64,
}

impl Double {
    fn magnitude(&self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }
    fn dist(&self, other: Double) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }
}

impl std::ops::Sub for Double {
    type Output = Double;
    fn sub(self, other: Double) -> Double {
        Double {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl std::ops::Mul<f64> for Double {
    type Output = Double;
    fn mul(self, other: f64) -> Double {
        Double {
            x: self.x * other,
            y: self.y * other,
        }
    }
}

#[derive(Clone, Copy)]
struct Circle {
    radius: f64,
    pinfo: PhysicsInfo,
    color: [f32; 4],
    is_dragged: bool,
}

impl Circle {
    fn update(&mut self, dt: f64, mouse_pos: Double) {
        if self.is_dragged {
            self.pinfo.vel.x = (mouse_pos.x - self.pinfo.pos.x) / dt;
            self.pinfo.vel.y = (mouse_pos.y - self.pinfo.pos.y) / dt;

            self.pinfo.pos.y += self.pinfo.vel.y * dt;
            self.pinfo.pos.x += self.pinfo.vel.x * dt;

            self.pinfo.pos.x = f64::max(f64::min(self.pinfo.pos.x, (WIDTH as f64 - 1.0 - self.radius) as f64), self.radius);
            self.pinfo.pos.y = f64::max(f64::min(self.pinfo.pos.y, (HEIGHT as f64 - 1.0 - self.radius) as f64), self.radius);

            return;
        }

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

struct Link {
    c1: usize,
    c2: usize,
    rest_length: f64,
}

struct StaticLink {
    c1: usize,
    c2: usize,
    rest_length: f64,
}

fn create_rope(circles: &mut Vec<Circle>, staticlinks: &mut Vec<StaticLink>, anchor_pos: Double, rope_length: f64, segment_num: i64, attachments: bool) {
    let segmental_node_radius = 1.0;
    for i in 0..segment_num+2 {
        let pos = Double {
            x: anchor_pos.x + (i as f64 * rope_length / segment_num as f64) * 0.0,
            y: anchor_pos.y + (i as f64 * rope_length / segment_num as f64) * 1.0,
        };
        if i==0||i==segment_num+1 {
            circles.push(Circle {
                radius: DEFAULT_RADIUS,
                pinfo: PhysicsInfo {
                    pos,
                    vel: Double { x: 0.0, y: 0.0 },
                    acc: Double { x: 0.0, y: 0.0 },
                },
                color: DEFAULT_COLOR,
                is_dragged: false,
            });
        } else {
            circles.push(Circle {
                radius: segmental_node_radius,
                pinfo: PhysicsInfo {
                    pos,
                    vel: Double { x: 0.0, y: 0.0 },
                    acc: Double { x: 0.0, y: 0.0 },
                },
                color: [0.0, 0.0, 0.0, 0.0],
                is_dragged: false,
            });
        }
        if i > 0 {
            staticlinks.push(StaticLink {
                c1: circles.len() - 2,
                c2: circles.len() - 1,
                rest_length: rope_length / segment_num as f64,
            });
        }
    }
}

fn create_softbody(circles: &mut Vec<Circle>, links: &mut Vec<StaticLink>, num_of_circles: usize, radius: f64, sub_radius: f64, pos: Double) {
    let circum = 2.0 * PI * radius;
    let rest_len = circum / num_of_circles as f64;
    let circles_len = circles.len();
    for i in 0..num_of_circles {
        circles.push(Circle {
            radius: sub_radius,
            pinfo: PhysicsInfo {
                pos: Double {
                    x: radius * (i as f64 * 2.0 * PI / num_of_circles as f64).cos() + pos.x,
                    y: radius * (i as f64 * 2.0 * PI / num_of_circles as f64).sin() + pos.y,
                },
                vel: Double { x: 0.0, y: 0.0 },
                acc: Double { x: 0.0, y: 0.0 },
            },
            color: DEFAULT_COLOR,
            is_dragged: false,
        });
        if i > 0 {
            links.push(StaticLink {
                c1: i + circles_len - 1,
                c2: i + circles_len,
                rest_length: rest_len,
            });
        }
    }
    links.push(StaticLink {
        c1: circles_len,
        c2: circles_len + num_of_circles - 1,
        rest_length: rest_len,
    });
}

fn create_spring_softbody(circles: &mut Vec<Circle>, links: &mut Vec<Link>, num_of_circles: usize, radius: f64, sub_radius: f64, pos: Double) {
    let circum = 2.0 * PI * radius;
    let rest_len = circum / num_of_circles as f64;
    let circles_len = circles.len();
    for i in 0..num_of_circles {
        circles.push(Circle {
            radius: sub_radius,
            pinfo: PhysicsInfo {
                pos: Double {
                    x: radius * (i as f64 * 2.0 * PI / num_of_circles as f64).cos() + pos.x,
                    y: radius * (i as f64 * 2.0 * PI / num_of_circles as f64).sin() + pos.y,
                },
                vel: Double { x: 0.0, y: 0.0 },
                acc: Double { x: 0.0, y: 0.0 },
            },
            color: DEFAULT_COLOR,
            is_dragged: false,
        });
        if i > 0 {
            links.push(Link {
                c1: i + circles_len - 1,
                c2: i + circles_len,
                rest_length: rest_len,
            });
        }
    }
    links.push(Link {
        c1: circles_len,
        c2: circles_len + num_of_circles - 1,
        rest_length: rest_len,
    });
}

fn apply_spring_force(circles: &mut Vec<Circle>, c1: usize, c2: usize, rest_length: f64) {
    let c1_pos = circles[c1].pinfo.pos;
    let c2_pos = circles[c2].pinfo.pos;
    let displacement = c2_pos - c1_pos;
    let distance = displacement.magnitude();
    let direction = Double {
        x: displacement.x / distance,
        y: displacement.y / distance,
    };
    let spring_force = (distance - rest_length) * SPRING_CONST;
    let damping_force = (circles[c2].pinfo.vel - circles[c1].pinfo.vel) * DAMP_CONST * 0.1;
    let force = direction * spring_force;

    {
        let c1 = &mut circles[c1];
        c1.pinfo.acc.x += force.x - damping_force.x;
        c1.pinfo.acc.y += force.y - damping_force.y;
    }
    {
        let c2 = &mut circles[c2];
        c2.pinfo.acc.x -= force.x - damping_force.x;
        c2.pinfo.acc.y -= force.y - damping_force.y;
    }
}

fn apply_static_link(circles: &mut Vec<Circle>, c1: usize, c2: usize, rest_length: f64) {
    let neccesary_diff = circles[c1].pinfo.pos.dist(circles[c2].pinfo.pos) - rest_length;

    let offset = Double {
        x: neccesary_diff * 0.5,
        y: neccesary_diff * 0.5,
    };

    let c1_move_x_right: bool = circles[c1].pinfo.pos.x < circles[c2].pinfo.pos.x;
    let c1_move_y_up: bool = circles[c1].pinfo.pos.y < circles[c2].pinfo.pos.y;

    {
        let c1 = &mut circles[c1];
            if c1_move_x_right {
                c1.pinfo.pos.x += offset.x;
            } else {
                c1.pinfo.pos.x -= offset.x;
            }
            if c1_move_y_up {
                c1.pinfo.pos.y += offset.y;
            } else {
                c1.pinfo.pos.y -= offset.y;
            }
    }
    {
        let c2 = &mut circles[c2];
            if c1_move_x_right {
                c2.pinfo.pos.x -= offset.x;
            } else {
                c2.pinfo.pos.x += offset.x;
            }
            if c1_move_y_up {
                c2.pinfo.pos.y -= offset.y;
            } else {
                c2.pinfo.pos.y += offset.y;
            }
    }
}

fn draw_text(
    ctx: &Context,
    graphics: &mut G2d,
    glyphs: &mut Glyphs,
    color: [f32; 4],
    pos: Double,
    text: &str
) {
    text::Text::new_color(color, 20)
        .draw(
            text,
            glyphs,
            &ctx.draw_state,
            ctx.transform.trans(pos.x as f64, pos.y as f64),
            graphics,
        )
        .unwrap();
}

struct UserTerminal {
    display_text: String,
    input_text: String,
    cursor_pos: Double,
    cursor_mode: String,

}
/*
User Terminal Commands:
    help | Display types of help texts
        help text | Display commands that run on terminal texts
            help circle
                circle -radius -R -G -B -A -X -Y | Create a circle with (flags) radius radius and color R G B A at position X Y
        
        help mouse | Display commands that change modes for the mouse
            help circlemode
                circlemode -radius -R -G -B -A | Change mouse mode to make circles with (flags) radius radius and color R G B A
*/
impl UserTerminal {
    fn eval_cursor_click(&mut self, circles: &mut Vec<Circle>, links: &mut Vec<Link>, staticlinks: &mut Vec<StaticLink>) {
        if self.cursor_mode.starts_with("circle") {
            let mut args = self.cursor_mode.split(",");
            args.next();
            let radius = args.next().unwrap().parse().unwrap();
            let r = args.next().unwrap().parse().unwrap();
            let g = args.next().unwrap().parse().unwrap();
            let b = args.next().unwrap().parse().unwrap();
            let a = args.next().unwrap().parse().unwrap();
            circles.push(Circle {
                radius,
                pinfo: PhysicsInfo {
                    pos: Double { x: self.cursor_pos.x, y: self.cursor_pos.y },
                    vel: Double { x: 0.0, y: 0.0 },
                    acc: Double { x: 0.0, y: 0.0 },
                },
                color: [r, g, b, a],
                is_dragged: false,
            });
            println!("CIRCLEMODE: Creating circle with radius: {}, color: {:?}", radius, [r, g, b, a]);
        }
    }

    fn eval_cursor_mode(&mut self, event: &Event, context: &Context, graphics: &mut G2d) {
        if self.cursor_mode.starts_with("circle") {
            let mut args = self.cursor_mode.split(",");
            args.next();
            let radius: f64 = args.next().unwrap().parse().unwrap();
            let r = args.next().unwrap().parse().unwrap();
            let g = args.next().unwrap().parse().unwrap();
            let b = args.next().unwrap().parse().unwrap();
            let a: f32 = args.next().unwrap().parse().unwrap();

            ellipse(
                [r, g, b, a / 2.0],
                [
                    self.cursor_pos.x - radius,
                    self.cursor_pos.y - radius,
                    radius * 2.0,
                    radius * 2.0,
                ],
                context.transform,
                graphics,
            );
        }
    }

    fn execute_input(&mut self, circles: &mut Vec<Circle>, links: &mut Vec<Link>, staticlinks: &mut Vec<StaticLink>) {
        match self.input_text.trim() {
            "help" => {self.display_text = String::from("help text-Display text commands|help mouse-Display mouse commands");}


            "help text" => {self.display_text = String::from("help +circle");}

            "help circle" => {self.display_text = String::from("circle -radius -r -g -b -a -x -y");}
            s if s.starts_with("circle ") || s == "circle" => {
                let mut args = s.split_whitespace();
                args.next();
                let mut radius = DEFAULT_RADIUS;
                let mut color = DEFAULT_COLOR;
                let mut pos = Double { x: WIDTH as f64 / 2.0, y: HEIGHT as f64 / 2.0 };
                while let Some(arg) = args.next() {
                    match arg {
                        "-radius" => {radius = args.next().unwrap().parse().unwrap();}
                        "-r" => {color[0] = args.next().unwrap().parse().unwrap();}
                        "-g" => {color[1] = args.next().unwrap().parse().unwrap();}
                        "-b" => {color[2] = args.next().unwrap().parse().unwrap();}
                        "-a" => {color[3] = args.next().unwrap().parse().unwrap();}
                        "-x" => {pos.x = args.next().unwrap().parse().unwrap();}
                        "-y" => {pos.y = args.next().unwrap().parse().unwrap();}
                        _ => {}
                    }
                }
                println!("Creating circle with radius: {}, color: {:?}", radius, color);
                circles.push(Circle {
                    radius,
                    pinfo: PhysicsInfo {
                        pos,
                        vel: Double { x: 0.0, y: 0.0 },
                        acc: Double { x: 0.0, y: 0.0 },
                    },
                    color,
                    is_dragged: false,
                });
            }

            "help mouse" => {self.display_text = String::from("help +circlemode");}

            "help circlemode" => {self.display_text = String::from("circlemode -radius -r -g -b -a");}
            s if s.starts_with("circlemode") => {
                let mut args = s.split_whitespace();
                args.next();
                let mut radius = DEFAULT_RADIUS;
                let mut color = DEFAULT_COLOR;
                while let Some(arg) = args.next() {
                    match arg {
                        "-radius" => {radius = args.next().unwrap().parse().unwrap();}
                        "-r" => {color[0] = args.next().unwrap().parse().unwrap();}
                        "-g" => {color[1] = args.next().unwrap().parse().unwrap();}
                        "-b" => {color[2] = args.next().unwrap().parse().unwrap();}
                        "-a" => {color[3] = args.next().unwrap().parse().unwrap();}
                        _ => {}
                    }
                }
                println!("Changing cursor mode to circle with radius: {}, color: {:?}", radius, color);
                self.cursor_mode = format!("circle,{},{},{},{},{}", radius, color[0], color[1], color[2], color[3]);
            }

            _ => {}
        }

        self.input_text.clear();
    }

    fn handle_events(&mut self, event: &Event, circles: &mut Vec<Circle>, links: &mut Vec<Link>, staticlinks: &mut Vec<StaticLink>) {
        if let Some(Button::Keyboard(key)) = event.press_args() {
            match key {
                Key::Return => {self.execute_input(circles, links, staticlinks);}
                Key::Backspace => {self.input_text.pop();}
                Key::A => {self.input_text.push('a');}
                Key::B => {self.input_text.push('b');}
                Key::C => {self.input_text.push('c');}
                Key::D => {self.input_text.push('d');}
                Key::E => {self.input_text.push('e');}
                Key::F => {self.input_text.push('f');}
                Key::G => {self.input_text.push('g');}
                Key::H => {self.input_text.push('h');}
                Key::I => {self.input_text.push('i');}
                Key::J => {self.input_text.push('j');}
                Key::K => {self.input_text.push('k');}
                Key::L => {self.input_text.push('l');}
                Key::M => {self.input_text.push('m');}
                Key::N => {self.input_text.push('n');}
                Key::O => {self.input_text.push('o');}
                Key::P => {self.input_text.push('p');}
                Key::Q => {self.input_text.push('q');}
                Key::R => {self.input_text.push('r');}
                Key::S => {self.input_text.push('s');}
                Key::T => {self.input_text.push('t');}
                Key::U => {self.input_text.push('u');}
                Key::V => {self.input_text.push('v');}
                Key::W => {self.input_text.push('w');}
                Key::X => {self.input_text.push('x');}
                Key::Y => {self.input_text.push('y');}
                Key::Z => {self.input_text.push('z');}
                Key::D0 => {self.input_text.push('0');}
                Key::D1 => {self.input_text.push('1');}
                Key::D2 => {self.input_text.push('2');}
                Key::D3 => {self.input_text.push('3');}
                Key::D4 => {self.input_text.push('4');}
                Key::D5 => {self.input_text.push('5');}
                Key::D6 => {self.input_text.push('6');}
                Key::D7 => {self.input_text.push('7');}
                Key::D8 => {self.input_text.push('8');}
                Key::D9 => {self.input_text.push('9');}
                Key::Space => {self.input_text.push(' ');}
                Key::Minus => {self.input_text.push('-');}
                Key::Period => {self.input_text.push('.');}
                _ => {}
            }
        }
        
    }
}


fn main() {
    let mut window: PistonWindow = WindowSettings::new("Rusty Physics", [WIDTH as u32, HEIGHT as u32])
        .exit_on_esc(true)
        .build()
        .unwrap();

    let assets = find_folder::Search::ParentsThenKids(3, 3)
        .for_folder("assets")
        .unwrap();
    let ref font = assets.join("roboto.ttf");
    let mut glyphs = window.load_font(font).unwrap();

    let mut terminal = UserTerminal {
        display_text: String::from("TYPE help TO START"),
        input_text: String::from(""),
        cursor_pos: Double { x: -1.0, y: -1.0 },
        cursor_mode: String::from(""),
    };

    let mut grid = Grid::new(WIDTH, HEIGHT, CELL_SIZE);
    let mut circles: Vec<Circle> = Vec::new();
    let mut links: Vec<Link> = Vec::new();
    let mut staticlinks: Vec<StaticLink> = Vec::new();

    for _ in 0..CIRCLE_NUMBER {
        circles.push(Circle {
            radius: rand::random::<f64>() * (RADIUS_MAX - RADIUS_MIN) + RADIUS_MIN,
            pinfo: PhysicsInfo {
                pos: Double {
                    x: WIDTH as f64 * rand::random::<f64>(),
                    y: HEIGHT as f64 * rand::random::<f64>(),
                },
                vel: Double { x: 0.0, y: 0.0 },
                acc: Double { x: rand::random::<f64>() * 5.0, y: rand::random::<f64>() * 5.0 },
            },
            color: [rand::random::<f32>(), rand::random::<f32>(), rand::random::<f32>(), 1.0],
            is_dragged: false,
        });
    }

    //create_rope(&mut circles, &mut staticlinks, Double { x: WIDTH as f64 / 2.0, y: HEIGHT as f64 / 2.0}, 110.0, 8, true);

    //create_spring_softbody(&mut circles, &mut links, 30, 100.0, 10.0, Double { x: WIDTH as f64 / 2.0, y: HEIGHT as f64 / 2.0});


    let mut mouse_position = Double { x: 0.0, y: 0.0 };

    while let Some(event) = window.next() {
        terminal.handle_events(&event, &mut circles, &mut links, &mut staticlinks);
        if let Some(pos) = event.mouse_cursor_args() {
            mouse_position = Double { x: pos[0], y: pos[1] };
            terminal.cursor_pos = Double { x: pos[0], y: pos[1] };
        }
        if let Some(button) = event.press_args() {
            if button == Button::Mouse(MouseButton::Left) {
                terminal.eval_cursor_click(&mut circles, &mut links, &mut staticlinks);

                for circle in &mut circles {
                    let dx = mouse_position.x - circle.pinfo.pos.x;
                    let dy = mouse_position.y - circle.pinfo.pos.y;
                    let distance = (dx * dx + dy * dy).sqrt();
                    if distance <= circle.radius {
                        circle.is_dragged = true;
                    }
                }
            }
        }
        if let Some(button) = event.release_args() {
            if button == Button::Mouse(MouseButton::Left) {
                for circle in &mut circles {
                    circle.is_dragged = false;
                }
            }
        }

        window.draw_2d(&event, |context, graphics, device| {
            clear([1.0; 4], graphics);

            grid.reset();

            let mut i: i64 = 0;
            for circle in &mut circles {
                circle.update(1.0 / 60.0 * SPEED_FACTOR, mouse_position);
                grid.add_obj(*circle, i);
                ellipse(
                    circle.color,
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

            for link in &links {
                apply_spring_force(&mut circles, link.c1, link.c2, link.rest_length);
                line(
                    [0.0, 0.0, 0.0, 1.0],
                    1.0,
                    [
                        circles[link.c1].pinfo.pos.x,
                        circles[link.c1].pinfo.pos.y,
                        circles[link.c2].pinfo.pos.x,
                        circles[link.c2].pinfo.pos.y,
                    ],
                    context.transform,
                    graphics,
                );
            }

            for slink in &staticlinks {
                apply_static_link(&mut circles, slink.c1, slink.c2, slink.rest_length);
                line(
                    [0.3, 0.3, 0.3, 1.0],
                    1.0,
                    [
                        circles[slink.c1].pinfo.pos.x,
                        circles[slink.c1].pinfo.pos.y,
                        circles[slink.c2].pinfo.pos.x,
                        circles[slink.c2].pinfo.pos.y,
                    ],
                    context.transform,
                    graphics,
                );
            }

            terminal.eval_cursor_mode(&event, &context, graphics);

            draw_text(&context, graphics, &mut glyphs, [0.0, 0.0, 0.0, 1.0], Double { x: 0.0, y: 20.0 }, &terminal.display_text);
            rectangle(
                [0.0, 0.0, 0.0, 0.5],
                [0.0, 25.0, WIDTH as f64, 25.0],
                context.transform,
                graphics,
            );
            draw_text(&context, graphics, &mut glyphs, [0.0, 0.0, 0.0, 1.0], Double { x: 0.0, y: 45.0 }, &terminal.input_text);

            glyphs.factory.encoder.flush(device);
        });
    }
}
