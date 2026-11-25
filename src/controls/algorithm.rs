use std::f64::consts::PI;

use crate::shared::types::{DriveCommand, Point2, VehicleSpec, VehicleState};

pub struct Controller {
    pub vehicle: VehicleSpec,
    pub index: usize,
}

impl Controller {
    /// `vehicle` has the wheelbase and the speed and steering limits.
    pub fn new(vehicle: VehicleSpec) -> Self {
        Self { vehicle, index: 0 }
    }

    /// Attempts to find an intersection between the circle of radius `r` centered at `c` and the line between `u` and `v`.
    /// Returns `None` if no intersections are found.
    /// Returns the intersection point closest to `v` otherwise.
    pub fn circ_line_intersect(&self, u: &Point2, v: &Point2, c: &Point2, r: f64) -> Option<Point2> {
        let u_ = Point2 {
            x: u.x - c.x,
            y: u.y - c.y
        };
        let v_ = Point2 {
            x: v.x - c.x,
            y: v.y - c.y
        };

        let dy = v_.y - u_.y;
        let dx = v_.x - u_.x;
        let dist = dx.hypot(dy);

        let D = u_.x * v_.y - u_.y * v_.x;
        let discriminant = (r * r) * (dist * dist) - D * D;

        if discriminant < 0.0 {
            return None;
        }

        let sol1 = Point2 {
            x: (D * dy + dy.signum() * dx * discriminant.sqrt()) / (dist * dist) + c.x,
            y: (-D * dx + dy.abs() * discriminant.sqrt()) / (dist * dist) + c.y
        };
        let sol2 = Point2 {
            x: (D * dy - dy.signum() * dx * discriminant.sqrt()) / (dist * dist) + c.x,
            y: (-D * dx - dy.abs() * discriminant.sqrt()) / (dist * dist) + c.y
        };

        let min_x = u.x.min(v.x);
        let min_y = u.y.min(v.y);
        let max_x = u.x.max(v.x);
        let max_y = u.y.max(v.y);

        let sol1 = if min_x <= sol1.x && sol1.x <= max_x && min_y <= sol1.y && sol1.y <= max_y {
            Some(sol1)
        } else {
            None
        };
        let sol2 = if min_x <= sol2.x && sol2.x <= max_x && min_y <= sol2.y && sol2.y <= max_y {
            Some(sol2)
        } else {
            None
        };

        if sol1.is_none() && sol2.is_none() {
            None
        } else if sol1.is_some() && sol2.is_none() {
            Some(sol1.expect("sol1 should exist"))
        } else if sol2.is_some() && sol1.is_none() {
            Some(sol2.expect("sol2 should exist"))
        } else if sol1.expect("sol1 should exist").distance(v_) < sol2.expect("sol2 should exist").distance(v_) {
            Some(sol1.expect("sol1 should exist"))
        } else {
            Some(sol2.expect("sol2 should exist"))
        }
    }

    /// Speed and steering to follow `path`. Positive steering is a left turn.
    /// The numbers you return already have to sit inside the car's speed and steering caps.
    /// The node publishes them unchanged. `dt_s` is the time since the last call, in seconds.
    /// This starter always stops. See Controls in `docs/project.md`.
    pub fn command(&mut self, _state: &VehicleState, _path: &[Point2], _dt_s: f64) -> DriveCommand {
        // println!("HI");
        
        let L = 1.0;

        if self.index >= _path.len() {
            return DriveCommand::STOP;
        }

        let goal_pt = if self.index == _path.len() - 1 {
            _path[self.index]
        } else {
            let mut i = self.index;
            let mut target = None;
            while i < _path.len() - 1 && target.is_none() {
                target = self.circ_line_intersect(&_path[i], &_path[i+1], &_state.pose.position, L);
                i = i + 1;
            }

            if let Some(target) = target {
                // println!("target");
                self.index = i;
                target
            } else {
                // println!("{}", self.index);
                _path[self.index + 1]
            }
        };

        // println!("{} {}", goal_pt.x, goal_pt.y);
        
        let global_ang = (goal_pt.y - _state.pose.position.y).atan2(goal_pt.x - _state.pose.position.x);
        let local_ang = (global_ang - (_state.pose.yaw_rad + PI) % (2.0 * PI) - PI + 7.0 * PI) % (2.0 * PI) - PI;

        // println!("{global_ang} {} {} {local_ang}", global_ang - _state.pose.yaw_rad, (global_ang - (_state.pose.yaw_rad + PI) % (2.0 * PI)) % (2.0 * PI));
        // println!("{}", local_ang);

        DriveCommand {
            speed_mps: self.vehicle.max_speed_mps,
            // speed_mps: 1.0,
            steering_rad: local_ang.clamp(-self.vehicle.max_steering_rad, self.vehicle.max_steering_rad),
        }
    }
}
