extern crate quadtree_rs;

use quadtree_rs::area::AreaBuilder;
use quadtree_rs::{point::Point, Quadtree};

#[derive(Debug, PartialEq)]
pub struct Rect {
    pub anchor: (i32, i32), // The top-left corner
    pub size: (i32, i32),
}

pub struct Playground {
    pub size: (i32, i32), // (x,y) bounds. 0,0 is the top-left corner.
    obstacles: Quadtree<i32, u32>,
    obstacle_counter: u32, // TODO: Find a use for the obstacle IDs
    pub start: (i32, i32),
    pub goal: (i32, i32),
}

impl Playground {
    pub fn new(size: (i32, i32)) -> Self {
        return Self {
            obstacles: Quadtree::new(16),
            obstacle_counter: 0,
            size: size,
            start: (size.0 / 16, size.1 / 16),
            goal: (15 * size.0 / 16, size.1 / 16),
        };
    }

    // input format is (x,y) for (top_left_corner, bottom_right_corner)
    // Allows adding obstacles that overlap the bounds of the playground.
    pub fn add_obstacles(&mut self, o: Rect) {
        let region = AreaBuilder::default()
            .anchor(Point {
                x: o.anchor.0,
                y: o.anchor.1,
            })
            .dimensions((o.size.0, o.size.1))
            .build()
            .unwrap();
        self.obstacles.insert(region, self.obstacle_counter);
        self.obstacle_counter += 1;
    }

    // Returns the obstacle boxes in the playground's reference frame
    // Format is the same as add_obstacle_box()
    pub fn get_obstacles(&self) -> Vec<Rect> {
        let mut acc: Vec<Rect> = Vec::new();
        for region in self.obstacles.regions() {
            acc.push(Rect {
                anchor: (region.anchor().x, region.anchor().y),
                size: (region.width(), region.height()),
            });
        }
        return acc;
    }

    // Does the input rectangle intersect any of the obstacles (or playground bounds?)
    pub fn is_collision(&self, r: &Rect) -> bool {
        if r.anchor.0 < 0
            || r.anchor.1 < 0
            || r.anchor.0 + r.size.0 >= self.size.0
            || r.anchor.1 + r.size.1 >= self.size.1
        {
            return true;
        }

        let region = AreaBuilder::default()
            .anchor(Point {
                x: r.anchor.0,
                y: r.anchor.1,
            })
            .dimensions((r.size.0, r.size.1))
            .build()
            .unwrap();
        let mut query = self.obstacles.query(region);
        return !query.next().is_none();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn add_obstacle() {
        let mut p = Playground::new((500, 500));
        let anchor = (0, 0);
        let size = (123, 456);
        p.add_obstacles(Rect { anchor, size });

        let v = p.get_obstacles();
        assert_eq!(v.len(), 1);
        assert_eq!(v[0].anchor, anchor);
        assert_eq!(v[0].size, size);
    }

    #[test]
    fn add_obstacle_exceeding_bounds() {
        let mut p = Playground::new((500, 500));
        let anchor = (0, 0);
        let size = (1024, 1024);
        p.add_obstacles(Rect { anchor, size });

        let v = p.get_obstacles();
        assert_eq!(v.len(), 1);
        assert_eq!(v[0].anchor, anchor);
        assert_eq!(v[0].size, size);
    }

    #[test]
    fn is_collision() {
        let mut p = Playground::new((500, 500));
        p.add_obstacles(Rect {
            anchor: (100, 100),
            size: (300, 300),
        });

        assert!(!p.is_collision(&Rect {
            anchor: (10, 10),
            size: (20, 20)
        }));
        assert!(!p.is_collision(&Rect {
            anchor: (450, 450),
            size: (20, 20)
        }));

        assert!(p.is_collision(&Rect {
            anchor: (50, 50),
            size: (100, 100)
        }));
        assert!(p.is_collision(&Rect {
            anchor: (-10, -10),
            size: (20, 20)
        }));
        assert!(p.is_collision(&Rect {
            anchor: (250, 250),
            size: (100, 100)
        }));
        assert!(p.is_collision(&Rect {
            anchor: (450, 450),
            size: (100, 100)
        }));
    }
}
