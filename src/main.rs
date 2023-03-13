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

pub mod actor;
use actor::Actor;

pub struct App {
    gl: GlGraphics, // OpenGL drawing backend.
    playground: Playground,
    actor: Actor,
    pub start: (i32, i32),
    pub goal: (i32, i32),
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
            {
                let [sx, sy] = math::mul(
                    [
                        self.start.0 as f64,
                        self.start.1 as f64,
                    ],
                    scale,
                );
                let [gx, gy] = math::mul(
                    [self.goal.0 as f64, self.goal.1 as f64],
                    scale,
                );
                let r = 10.0 * (scale[0].powf(2.0) + scale[1].powf(2.0)).sqrt();
                let start = ellipse::circle(sx, sy, r);
                let goal = ellipse::circle(gx, gy, r);
                ellipse(color::RED, start, c.transform, gl);
                ellipse(color::RED, goal, c.transform, gl);
            }

            // Render actor
            {
                let [acx, acy] =
                    math::mul([self.actor.pose.x as f64, self.actor.pose.y as f64], scale);
                let [asx, asy] =
                    math::mul([self.actor.size.0 as f64, self.actor.size.1 as f64], scale);
                let transform = c
                    .transform
                    .trans(acx, acy)
                    .rot_deg(self.actor.pose.t as f64);
                let actor = rectangle::centered([0.0, 0.0, asx / 2.0, asy / 2.0]);
                rectangle(color::BLUE, actor, transform, gl);
            }
        });
    }

    fn update(&mut self, _args: &UpdateArgs) {
        // TODO
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

    let playground = Playground::new((initial_size.0 as i32, initial_size.1 as i32));
    let start = (50, 50);
    let goal = (50, 50);
    let actor = Actor::new(start);
    let mut app = App {
        gl: GlGraphics::new(opengl_version),
        playground,
        actor,
        start,
        goal,
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
