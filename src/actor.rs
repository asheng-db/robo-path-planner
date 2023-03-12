use crate::playground::{Playground, Rect};

// pose values are stored as integers to (hopefully) simplify graph search
// Otherwise we'd likely need an oct-tree.
#[derive(Debug, PartialEq)]
pub struct Pose {
    pub x: i32,
    pub y: i32,
    pub t: i32, // degrees
}

pub struct Actor {
    // Pose: (x,y) in playground frame; theta in degrees
    pub pose: Pose,
    pub size: (i32, i32),
}

impl Actor {
    const DRIVE_SPEED: f64 = 10.0; // pixels per frame
    const TURN_SPEED: i32 = 3; // degrees per frame

    pub fn new(position: (i32, i32)) -> Self {
        return Self {
            pose: Pose {
                x: position.0,
                y: position.1,
                t: 45,
            },
            size: (13, 21),
        };
    }

    // TODO: Consider making this do Ackermann steering (like a car)
    fn pose_neighbors(&self, pose: &Pose, playground: &Playground) -> Vec<Pose> {
        let mut raw_neighbors: Vec<Pose> = Vec::new();
        {
            let t = (pose.t as f64).to_radians();
            let dx = Actor::DRIVE_SPEED * t.sin();
            let dy = Actor::DRIVE_SPEED * t.cos();
            let dt = Actor::TURN_SPEED;
            // forward
            raw_neighbors.push(Pose {
                x: pose.x + (dx as i32),
                y: pose.y + (dy as i32),
                ..*pose
            });
            // backward
            raw_neighbors.push(Pose {
                x: pose.x - (dx as i32),
                y: pose.y - (dy as i32),
                ..*pose
            });
            // right
            raw_neighbors.push(Pose {
                t: pose.t + dt,
                ..*pose
            });
            // left
            raw_neighbors.push(Pose {
                t: pose.t - dt,
                ..*pose
            });
        }

        let mut ret: Vec<Pose> = Vec::new();
        for n in raw_neighbors {
            if self.is_valid_pose(&n, playground) {
                ret.push(n);
            }
        }
        return ret;
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
    fn pose_neighbors() {
        let playground = Playground::new((500, 500));
        let actor = Actor {
            pose: Pose { x: 12, y: 12, t: 0 },
            size: (6, 18),
        };

        let neighbors = actor.pose_neighbors(&actor.pose, &playground);
        assert_eq!(neighbors.len(), 3);
        assert_eq!(neighbors[0], Pose { x: 12, y: 22, t: 0 });
        assert_eq!(neighbors[1], Pose { x: 12, y: 12, t: 3 });
        assert_eq!(
            neighbors[2],
            Pose {
                x: 12,
                y: 12,
                t: -3,
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
