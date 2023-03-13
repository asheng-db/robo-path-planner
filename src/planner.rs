extern crate splines;

use rand::Rng;
use splines::{Interpolation, Key, Spline};
use std::{cmp, collections::HashMap};

use crate::playground::{Playground, Rect};

// pose values are stored as integers to (hopefully) simplify graph search
// i.e. we basically have a search grid size of 1
// Otherwise we'd likely need an oct-tree.
#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct Pose {
    pub x: i32, // Cartesian coordinates in playground frame
    pub y: i32,
    pub t: i32, // degrees
}

pub struct Planner {
    pub pose: Pose,
    pub size: (i32, i32),
    pub path: Vec<Pose>,
    splines: Option<(Spline<f64, f64>, Spline<f64, f64>, Spline<f64, f64>)>,
}

impl Planner {
    pub fn new(playground: &Playground) -> Self {
        return Self {
            pose: Pose {
                x: playground.start.0,
                y: playground.start.1,
                t: 0,
            },
            size: (15, 30),
            path: vec![],
            splines: None,
        };
    }

    pub fn compute_path(&mut self, playground: &Playground) {
        if self.splines.is_none() {
            let path = self.rrt_to_goal(playground);
            let path = self.compact_path(playground, &path);
            self.splines = Some(Self::build_spline(&path));
            self.path = path;
        }
    }

    pub fn update_pos(&mut self, t: f64) {
        const PIXELS_PER_SEC: f64 = 24.0;
        let t = t * PIXELS_PER_SEC;
        match &self.splines {
            None => (),
            Some((sx, sy, st)) => {
                self.pose = Pose {
                    x: sx.clamped_sample(t).unwrap() as i32,
                    y: sy.clamped_sample(t).unwrap() as i32,
                    t: st.clamped_sample(t).unwrap() as i32,
                }
            }
        };
    }

    fn build_spline(path: &Vec<Pose>) -> (Spline<f64, f64>, Spline<f64, f64>, Spline<f64, f64>) {
        let mut last: Option<Pose> = None;
        let mut dist = 0.0;
        let mut keys = (Vec::new(), Vec::new(), Vec::new());
        for pose in path {
            match last {
                None => (),
                Some(prev) => dist += Self::euclid_dist(&prev, pose),
            }
            last = Some(pose.clone());
            keys.0
                .push(Key::new(dist, pose.x as f64, Interpolation::Linear));
            keys.1
                .push(Key::new(dist, pose.y as f64, Interpolation::Linear));
            keys.2
                .push(Key::new(dist, pose.t as f64, Interpolation::Linear));
        }
        return (
            Spline::from_vec(keys.0),
            Spline::from_vec(keys.1),
            Spline::from_vec(keys.2),
        );
    }

    fn compact_path(&self, playground: &Playground, path: &Vec<Pose>) -> Vec<Pose> {
        let mut acc: Vec<Pose> = Vec::new();
        acc.push(path[0]);
        for i in 2..path.len() {
            if !self.is_valid_path(playground, acc.last().unwrap(), &path[i]) {
                acc.push(path[i - 1]);
            }
        }
        acc.push(path.last().unwrap().clone());
        return acc;
    }

    // Rapid Random Tree pathfinder
    fn rrt_to_goal(&self, playground: &Playground) -> Vec<Pose> {
        const GRID_SIZE: i32 = 10;
        const GOAL_SELECT: f64 = 0.01;
        let mut rng = rand::thread_rng();

        let start = Pose {
            x: playground.start.0,
            y: playground.start.1,
            t: 0,
        };
        let goal = Pose {
            x: playground.goal.0,
            y: playground.goal.1,
            t: 0,
        };

        let mut visited_to_parent: HashMap<Pose, Pose> = HashMap::new();
        visited_to_parent.insert(start, start);
        loop {
            let rpose = match rng.gen_bool(GOAL_SELECT) {
                true => goal,
                false => Pose {
                    x: rng.gen_range(0..playground.size.0 / GRID_SIZE) * GRID_SIZE,
                    y: rng.gen_range(0..playground.size.1 / GRID_SIZE) * GRID_SIZE,
                    t: rng.gen_range(0..36) * GRID_SIZE,
                },
            };
            if visited_to_parent.contains_key(&rpose) || !self.is_valid_pose(playground, &rpose) {
                continue;
            }

            let mut nearest: Option<Pose> = None;
            let mut nearest_dist = (playground.size.0 + playground.size.1) as f64;
            for c in visited_to_parent.keys() {
                let dist = Self::euclid_dist(&rpose, &c);
                if dist >= nearest_dist || !self.is_valid_path(playground, &rpose, &c) {
                    continue;
                }

                nearest = Some(c.clone());
                nearest_dist = dist;
            }

            match nearest {
                None => continue,
                Some(n) => {
                    visited_to_parent.insert(rpose, n);
                    if rpose == goal {
                        break;
                    }
                }
            }
        }

        let mut ret = Vec::new();
        let mut x = goal;
        while x != visited_to_parent[&x] {
            ret.push(x);
            x = visited_to_parent[&x];
        }
        ret.push(x);
        ret.reverse();
        // print!("{:?}\n", ret);
        return ret;
    }

    fn euclid_dist(from: &Pose, to: &Pose) -> f64 {
        let dx = (from.x - to.x) as f64;
        let dy = (from.y - to.y) as f64;
        return (dx.powf(2.0) + dy.powf(2.0)).sqrt();
    }

    fn is_valid_path(&self, playground: &Playground, f: &Pose, t: &Pose) -> bool {
        let ft = (f.t as f64).to_radians();
        let tt = (t.t as f64).to_radians();

        let xs = cmp::max(
            (((self.size.0 as f64) * ft.cos()).abs() + ((self.size.1 as f64) * ft.sin()).abs())
                as i32,
            (((self.size.0 as f64) * tt.cos()).abs() + ((self.size.1 as f64) * tt.sin()).abs())
                as i32,
        );
        let ys = cmp::max(
            (((self.size.0 as f64) * ft.sin()).abs() + ((self.size.1 as f64) * ft.cos()).abs())
                as i32,
            (((self.size.0 as f64) * tt.sin()).abs() + ((self.size.1 as f64) * tt.cos()).abs())
                as i32,
        );
        let rect = Rect {
            anchor: (cmp::min(f.x, t.x) - xs / 2, cmp::min(f.y, t.y) - ys / 2),
            size: ((f.x - t.x).abs() + xs, (f.y - t.y).abs() + ys),
        };
        return !playground.is_collision(&rect);
    }

    fn is_valid_pose(&self, playground: &Playground, pose: &Pose) -> bool {
        let mut hitboxes: Vec<Rect> = Vec::new();

        // TODO: Generate a set of hitboxes conforming to the shape rather than a giant rectangle.
        // TODO: See if we can make the giant rectangle more concisely with a rotation matrix.
        let t = (pose.t as f64).to_radians();
        let xs = (((self.size.0 as f64) * t.cos()).abs() + ((self.size.1 as f64) * t.sin()).abs())
            as i32;
        let ys = (((self.size.0 as f64) * t.sin()).abs() + ((self.size.1 as f64) * t.cos()).abs())
            as i32;
        hitboxes.push(Rect {
            anchor: (pose.x - xs / 2, pose.y - ys / 2),
            size: (xs, ys),
        });

        for hitbox in hitboxes {
            if playground.is_collision(&hitbox) {
                return false;
            }
        }
        return true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_spline() {
        let path = vec![
            Pose { x: 0, y: 0, t: 0 },
            Pose {
                x: 100,
                y: 0,
                t: 128,
            },
        ];
        let splines = Planner::build_spline(&path);
        assert_eq!(splines.0.clamped_sample(0.0), Some(0.0));
        assert_eq!(splines.1.clamped_sample(0.0), Some(0.0));
        assert_eq!(splines.2.clamped_sample(0.0), Some(0.0));

        assert_eq!(splines.0.clamped_sample(50.0), Some(50.0));
        assert_eq!(splines.1.clamped_sample(50.0), Some(0.0));
        assert_eq!(splines.2.clamped_sample(50.0), Some(64.0));

        assert_eq!(splines.0.clamped_sample(100.0), Some(100.0));
        assert_eq!(splines.1.clamped_sample(100.0), Some(0.0));
        assert_eq!(splines.2.clamped_sample(100.0), Some(128.0));
    }

    #[test]
    fn compact_path() {
        let playground = Playground::new((800, 800), (50, 50), (750, 750));
        let actor = Planner::new(&playground);
        let path = actor.rrt_to_goal(&playground);
        let path = actor.compact_path(&playground, &path);
        assert_eq!(path.len(), 2);
    }

    #[test]
    fn rrt_to_goal() {
        let playground = Playground::new((800, 800), (50, 50), (750, 750));
        let actor = Planner::new(&playground);
        let path = actor.rrt_to_goal(&playground);
        assert_ne!(path.len(), 0);
    }

    #[test]
    fn rrt_to_goal_obstacle() {
        let mut playground = Playground::new((800, 800), (50, 50), (750, 750));
        playground.add_obstacles(Rect {
            anchor: (100, 100),
            size: (600, 600),
        });
        let actor = Planner::new(&playground);
        let path = actor.rrt_to_goal(&playground);
        assert_ne!(path.len(), 0);
    }

    #[test]
    fn euclid_dist() {
        let target = Pose {
            x: 750,
            y: 50,
            t: 0,
        };
        assert!(
            Planner::euclid_dist(
                &target,
                &Pose {
                    x: 751,
                    y: 51,
                    t: 0
                }
            ) < 10.0
        );
        assert!(
            Planner::euclid_dist(
                &target,
                &Pose {
                    x: 751,
                    y: 25,
                    t: 0
                }
            ) > 10.0
        );
        assert!(
            Planner::euclid_dist(
                &target,
                &Pose {
                    x: 751,
                    y: 51,
                    t: 180
                }
            ) < 10.0
        );
    }

    #[test]
    fn is_valid_path() {
        let mut playground = Playground::new((500, 500), (0, 0), (0, 0));
        playground.add_obstacles(Rect {
            anchor: (100, 100),
            size: (300, 300),
        });

        let actor = Planner {
            pose: Pose { x: 0, y: 0, t: 0 },
            size: (40, 40),
            path: vec![],
            splines: None,
        };
        assert!(actor.is_valid_path(
            &playground,
            &Pose {
                x: 50,
                y: 50,
                t: 90
            },
            &Pose {
                x: 450,
                y: 50,
                t: 90
            }
        ));
        assert!(!actor.is_valid_path(
            &playground,
            &Pose {
                x: 50,
                y: 50,
                t: 90
            },
            &Pose {
                x: 450,
                y: 450,
                t: 90
            }
        ));
    }

    #[test]
    fn is_valid_pose() {
        let mut playground = Playground::new((500, 500), (0, 0), (0, 0));
        playground.add_obstacles(Rect {
            anchor: (100, 100),
            size: (300, 300),
        });
        // Vertical line
        let actor = Planner {
            pose: Pose { x: 0, y: 0, t: 0 },
            size: (1, 128),
            path: vec![],
            splines: None,
        };

        assert!(actor.is_valid_pose(
            &playground,
            &Pose {
                x: 50,
                y: 250,
                t: 0
            },
        ));
        assert!(!actor.is_valid_pose(
            &playground,
            &Pose {
                x: 50,
                y: 250,
                t: 90
            },
        ));

        assert!(!actor.is_valid_pose(
            &playground,
            &Pose {
                x: 250,
                y: 50,
                t: 0
            },
        ));
        assert!(actor.is_valid_pose(
            &playground,
            &Pose {
                x: 250,
                y: 50,
                t: 90
            },
        ));

        assert!(!actor.is_valid_pose(
            &playground,
            &Pose {
                x: 250,
                y: 250,
                t: 0
            },
        ));
    }
}
