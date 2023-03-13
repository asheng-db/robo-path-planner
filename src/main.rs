extern crate glutin_window;
extern crate graphics;
extern crate opengl_graphics;
extern crate piston;

use glutin_window::GlutinWindow as Window;
use opengl_graphics::{GlGraphics, OpenGL};
use piston::event_loop::{EventSettings, Events};
use piston::input::{RenderArgs, RenderEvent, UpdateArgs, UpdateEvent};
use piston::window::WindowSettings;

pub mod playground;
use playground::{Playground, Rect};

pub mod planner;
use planner::Planner;

pub struct App {
    gl: GlGraphics, // OpenGL drawing backend.
    playground: Playground,
    planner: Planner,
    t: f64,
}

impl App {
    fn render(&mut self, args: &RenderArgs) {
        // Import _everything_ for now because otherwise we get trait method errors.
        use graphics::*;
        self.gl.draw(args.viewport(), |c, gl| {
            clear(color::WHITE, gl);

            let scale = [
                args.window_size[0] / (self.playground.size.0 as f64),
                args.window_size[1] / (self.playground.size.1 as f64),
            ];

            // Render obstacles
            for r in self.playground.get_obstacles() {
                let [ax, ay] = math::mul([r.anchor.0 as f64, r.anchor.1 as f64], scale);
                let [sx, sy] = math::mul([r.size.0 as f64, r.size.1 as f64], scale);
                let r = rectangle::rectangle_by_corners(ax, ay, ax + sx, ay + sy);
                rectangle(color::BLACK, r, c.transform, gl);
            }

            // Render start/goal
            let [sx, sy] = math::mul(
                [
                    self.playground.start.0 as f64,
                    self.playground.start.1 as f64,
                ],
                scale,
            );
            let [gx, gy] = math::mul(
                [self.playground.goal.0 as f64, self.playground.goal.1 as f64],
                scale,
            );
            let r = 10.0 * (scale[0].powf(2.0) + scale[1].powf(2.0)).sqrt();
            let start = ellipse::circle(sx, sy, r);
            let goal = ellipse::circle(gx, gy, r);
            ellipse(color::RED, start, c.transform, gl);
            ellipse(color::RED, goal, c.transform, gl);

            // Render path
            let mut lp = [sx, sy];
            for next_pose in &self.planner.path {
                let np = math::mul([next_pose.x as f64, next_pose.y as f64], scale);
                line_from_to(
                    color::GREEN,
                    1.0,
                    [lp[0], lp[1]],
                    [np[0], np[1]],
                    c.transform,
                    gl,
                );
                lp = np;
            }

            // Render actor
            let [acx, acy] = math::mul(
                [self.planner.pose.x as f64, self.planner.pose.y as f64],
                scale,
            );
            let [asx, asy] = math::mul(
                [self.planner.size.0 as f64, self.planner.size.1 as f64],
                scale,
            );
            let transform = c
                .transform
                .trans(acx, acy)
                .rot_deg(self.planner.pose.t as f64);
            let r = rectangle::rectangle_by_corners(asx / -2.0, asy / -2.0, asx, asy);
            rectangle(color::BLUE, r, transform, gl);
        });
    }

    fn update(&mut self, args: &UpdateArgs) {
        self.t += args.dt;
        self.planner.update_pos(self.t);
    }
}

fn main() {
    let opengl_version = OpenGL::V3_2;
    let initial_size = (800, 800);
    let mut window: Window = WindowSettings::new("playground", initial_size)
        .graphics_api(opengl_version)
        .exit_on_esc(true)
        .build()
        .unwrap();

    let start = (50, 50);
    let goal = (750, 50);
    let playground = Playground::new((initial_size.0 as i32, initial_size.1 as i32), start, goal);
    let planner = Planner::new(&playground);
    let mut app = App {
        gl: GlGraphics::new(opengl_version),
        playground,
        planner,
        t: 0.0,
    };

    let obstacles = [
        // vertical barrier 1
        Rect {
            anchor: (200, 0),
            size: (100, 650),
        },
        Rect {
            anchor: (200, 750),
            size: (100, 100),
        },
        // vertical barrier 2
        Rect {
            anchor: (650, 0),
            size: (50, 100),
        },
        Rect {
            anchor: (650, 200),
            size: (50, 100),
        },
        // vertical barrier 3
        Rect {
            anchor: (500, 50),
            size: (50, 200),
        },
        // horizontal barrier 1
        Rect {
            anchor: (300, 500),
            size: (350, 100),
        },
        Rect {
            anchor: (750, 500),
            size: (300, 100),
        },
        // horizontal barrier 2
        Rect {
            anchor: (300, 300),
            size: (50, 100),
        },
        Rect {
            anchor: (450, 300),
            size: (400, 100),
        },
    ];
    for o in obstacles {
        app.playground.add_obstacles(o);
    }

    app.planner.compute_path(&app.playground);

    let mut events = Events::new(EventSettings::new());
    while let Some(e) = events.next(&mut window) {
        if let Some(args) = e.render_args() {
            app.render(&args);
        }

        if let Some(args) = e.update_args() {
            app.update(&args);
        }
    }
}
