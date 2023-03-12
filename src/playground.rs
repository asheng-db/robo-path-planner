extern crate quadtree_rs;

use quadtree_rs::area::{AreaBuilder};
use quadtree_rs::{point::Point, Quadtree};

#[derive(Debug,PartialEq)]
pub struct Rect {
    pub anchor: (u32, u32), // The top-left corner
    pub size: (u32, u32),
}

pub struct Playground {
    pub size: (u32,u32), // (x,y) bounds. 0,0 is the top-left corner.
    obstacles: Quadtree::<u32, u32>,
    obstacle_counter: u32, // TODO: Find a use for the obstacle IDs
}

impl Playground {
    pub fn new(size: (u32,u32)) -> Self {
        return Self {
            obstacles: Quadtree::new(16),
            obstacle_counter: 0,
            size: size,
        };
    }

    // input format is (x,y) for (top_left_corner, bottom_right_corner)
    // Allows adding obstacles that overlap the bounds of the playground.
    pub fn add_obstacles(&mut self, o: Rect) {
        let region = AreaBuilder::default()
            .anchor(Point { x: o.anchor.0, y: o.anchor.1 })
            .dimensions((o.size.0, o.size.1))
            .build().unwrap();
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
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn add_obstacle() {
        let mut p = Playground::new((500, 500));
        let anchor = (0,0);
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
        let anchor = (0,0);
        let size = (1024, 1024);
        p.add_obstacles(Rect { anchor, size });

        let v = p.get_obstacles();
        assert_eq!(v.len(), 1);
        assert_eq!(v[0].anchor, anchor);
        assert_eq!(v[0].size, size);
    }
}