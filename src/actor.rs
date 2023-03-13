use std::collections::{BinaryHeap, HashMap};

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

pub struct Actor {
    // Pose: (x,y) in playground frame; theta in degrees
    pub pose: Pose,
    pub size: (i32, i32),
}

impl Actor {
    const DRIVE_STEP: f64 = 5.0;
    const TURN_STEP: i32 = 3;

    pub fn new(position: (i32, i32)) -> Self {
        return Self {
            pose: Pose {
                x: position.0,
                y: position.1,
                t: 0,
            },
            size: (13, 21),
        };
    }

    // A-star from start to goal, with each playground pixel as a grid tile.
    fn path_to_goal(&self, playground: &Playground, start: Pose, goal: Pose) -> Vec<Pose> {
        const THRESHOLD: f64 = Actor::DRIVE_STEP;

        // heap is (estimated_cost, true_cost, vertex, parent)
        let mut frontier: BinaryHeap<(i32, i32, Pose, Pose)> = BinaryHeap::new();
        let mut visited_to_parent: HashMap<Pose, Pose> = HashMap::new();

        frontier.push((0, 0, start, start));
        while frontier.len() > 0 {
            match frontier.pop() {
                Some((_, cost, curr, prev)) => {
                    if visited_to_parent.contains_key(&curr) {
                        continue;
                    }
                    visited_to_parent.insert(curr, prev);

                    if Self::euclid_dist(&curr, &goal) < THRESHOLD {
                        let mut acc: Vec<Pose> = Vec::new();
                        let mut x = curr;
                        while x != visited_to_parent[&x] {
                            acc.push(x);
                            x = visited_to_parent[&x];
                        }
                        acc.push(start);
                        acc.reverse();
                        return acc;
                    }
                    for n in Self::pose_neighbors(&curr) {
                        if !self.is_valid_pose(&n, playground) || visited_to_parent.contains_key(&n)
                        {
                            continue;
                        }
                        let true_cost = cost + 1;
                        let estimate = Self::euclid_dist(&n, &goal) as i32;
                        // Multiply by -1 since Rust's heap is a max heap
                        frontier.push((-1 * (true_cost + estimate), true_cost, n, curr));
                    }
                }
                None => panic!("No path found!"),
            }
        }
        return vec![];
    }

    fn euclid_dist(from: &Pose, to: &Pose) -> f64 {
        let dx = (from.x - to.x) as f64;
        let dy = (from.y - to.y) as f64;
        let dt = ((from.t - to.t) / Actor::TURN_STEP) as f64;
        return (dx.powf(2.0) + dy.powf(2.0) + dt.powf(2.0)).sqrt();
    }

    // TODO: Consider making this do Ackermann steering (like a car)
    fn pose_neighbors(pose: &Pose) -> Vec<Pose> {
        let mut neighbors: Vec<Pose> = Vec::new();
        {
            let t = (pose.t as f64).to_radians();
            let dx = Actor::DRIVE_STEP * t.sin();
            let dy = Actor::DRIVE_STEP * t.cos();
            let dt = Actor::TURN_STEP;
            // forward
            neighbors.push(Pose {
                x: pose.x + (dx as i32),
                y: pose.y + (dy as i32),
                ..*pose
            });
            // backward
            neighbors.push(Pose {
                x: pose.x - (dx as i32),
                y: pose.y - (dy as i32),
                ..*pose
            });
            // right
            neighbors.push(Pose {
                t: (pose.t + dt) % 360,
                ..*pose
            });
            // left
            neighbors.push(Pose {
                t: (pose.t - dt) % 360,
                ..*pose
            });
        }
        return neighbors;
    }

    fn is_valid_pose(&self, pose: &Pose, playground: &Playground) -> bool {
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
    fn path_to_goal() {
        let playground = Playground::new((800, 800));
        let start = Pose { x: 50, y: 50, t: 0 };
        let goal = Pose {
            x: 750,
            y: 750,
            t: 0,
        };
        let actor = Actor::new((start.x, start.y));

        let path = actor.path_to_goal(&playground, start, goal);
        assert_ne!(path.len(), 0);
        assert!(path.len() > (Actor::euclid_dist(&start, &goal) / Actor::DRIVE_STEP) as usize);
    }

    #[test]
    fn euclid_dist() {
        let target = Pose {
            x: 750,
            y: 50,
            t: 0,
        };
        assert!(
            Actor::euclid_dist(
                &target,
                &Pose {
                    x: 751,
                    y: 51,
                    t: 0
                }
            ) < 10.0
        );
        assert!(
            Actor::euclid_dist(
                &target,
                &Pose {
                    x: 751,
                    y: 25,
                    t: 0
                }
            ) > 10.0
        );
        assert!(
            Actor::euclid_dist(
                &target,
                &Pose {
                    x: 751,
                    y: 51,
                    t: 180
                }
            ) > 10.0
        );
    }

    #[test]
    fn pose_neighbors() {
        let neighbors = Actor::pose_neighbors(&Pose { x: 12, y: 12, t: 0 });
        assert_eq!(neighbors.len(), 4);
        assert_eq!(neighbors[0], Pose { x: 12, y: 17, t: 0 });
        assert_eq!(neighbors[1], Pose { x: 12, y: 7, t: 0 });
        assert_eq!(neighbors[2], Pose { x: 12, y: 12, t: 3 });
        assert_eq!(
            neighbors[3],
            Pose {
                x: 12,
                y: 12,
                t: -3,
            }
        );
    }

    #[test]
    fn pose_neighbors_overflow() {
        let neighbors = Actor::pose_neighbors(&Pose {
            x: 12,
            y: 12,
            t: 359,
        });
        assert_eq!(neighbors.len(), 4);
        assert_eq!(neighbors[2], Pose { x: 12, y: 12, t: 2 });
        assert_eq!(
            neighbors[3],
            Pose {
                x: 12,
                y: 12,
                t: 356,
            }
        );
    }

    #[test]
    fn is_valid_pose() {
        let mut playground = Playground::new((500, 500));
        playground.add_obstacles(Rect {
            anchor: (100, 100),
            size: (300, 300),
        });
        // Vertical line
        let actor = Actor {
            pose: Pose { x: 0, y: 0, t: 0 },
            size: (1, 128),
        };

        assert!(actor.is_valid_pose(
            &Pose {
                x: 50,
                y: 250,
                t: 0
            },
            &playground
        ));
        assert!(!actor.is_valid_pose(
            &Pose {
                x: 50,
                y: 250,
                t: 90
            },
            &playground
        ));

        assert!(!actor.is_valid_pose(
            &Pose {
                x: 250,
                y: 50,
                t: 0
            },
            &playground
        ));
        assert!(actor.is_valid_pose(
            &Pose {
                x: 250,
                y: 50,
                t: 90
            },
            &playground
        ));

        assert!(!actor.is_valid_pose(
            &Pose {
                x: 250,
                y: 250,
                t: 0
            },
            &playground
        ));
    }
}
