use std::{cmp::Ordering, collections::{BinaryHeap, HashMap}};

use crate::{
    shared::{grid::Grid, types::{Path, Point2, VehicleState}},
};

#[derive(Copy, Clone, PartialEq)]
struct Step {
    c: usize,
    r: usize,
    g_score: usize,  // g score (from start)
    h_score: usize,  // h score (from end)
}

impl Step {
    fn fc(&self) -> usize {
        self.g_score + self.h_score
    }
}

impl Ord for Step {
    fn cmp(&self, other: &Self) -> Ordering {
        other.fc().cmp(&self.fc())
    }
}

impl PartialOrd for Step {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Eq for Step {}

#[derive(Default)]
pub struct Planner {}

fn octile_dist(c1: usize, r1: usize, c2: usize, r2: usize) -> usize {
    let dx = c1.abs_diff(c2);
    let dy = r1.abs_diff(r2);
    10 * dx.max(dy) + 4 * dx.min(dy)
}

fn adj(map: &Grid, c: usize, r: usize) -> Vec<(usize, usize)> {
    let mut v = vec![];
    // let mut cv = vec![c];
    // let mut rv = vec![r];
    // if 0 < c { cv.push(c-1); }
    // if c < map.width - 1 { cv.push(c+1); }
    // if 0 < r { rv.push(r-1); }
    // if r < map.height - 1 { rv.push(r+1); }
    // for c_ in &cv {
    //     for r_ in &rv {
    //         v.push((c_.clone(), r_.clone()));
    //     }
    // }
    if 0 < c { v.push((c-1, r)); }
    if c < map.width - 1 { v.push((c+1, r)); }
    if 0 < r { v.push((c, r-1)); }
    if r < map.height - 1 { v.push((c, r+1)) }
    v
}

impl Planner {

    fn d(&self, c1: usize, r1: usize, c2: usize, r2: usize) -> usize {
        octile_dist(c1, r1, c2, r2)
    }

    fn h(&self, c1: usize, r1: usize, c2: usize, r2: usize) -> usize {
        octile_dist(c1, r1, c2, r2)
    }

    fn construct_path(&self, _map: &Grid, parent: HashMap<(usize, usize), (usize, usize)>, end: (usize, usize)) -> Path {
        let mut path: Path = vec![];

        let mut curr = Some(end);
        while curr.is_some() {
            let (c, r) = curr.expect("curr should not be none");
            path.push(_map.inv_cell(c, r));
            curr = parent.get(&(c, r)).copied();
        }

        path.reverse();
        path
    }

    /// A route from the car to `goal`, or `None` if there is no way through.
    /// `clearance_m` is the circle from `VehicleSpec::clearance_m`.
    /// `goal_tolerance_m` is how close the last point has to be to the goal.
    /// A waypoint has to pass `Grid::clear(point, clearance_m)`. A step has to pass
    /// `Grid::segment_clear` at that same radius. `blocked` is only the cell cutoff.
    /// `validate_path` returns an error, not "no path", when clearance fails.
    /// This starter always returns `None`. See Planning in `docs/project.md`.
    pub fn plan(
        &mut self,
        _map: &Grid,
        _state: &VehicleState,
        _goal: Point2,
        _clearance_m: f64,
        _goal_tolerance_m: f64,
    ) -> Option<Path> {
        let mut pq: BinaryHeap<Step> = BinaryHeap::new();
        let mut g_scores: HashMap<(usize, usize), usize> = HashMap::new();
        let mut parent: HashMap<(usize, usize), (usize, usize)> = HashMap::new();

        let Some((cg, rg)) = _map.cell(_goal) else { return None; };
        let Some((c, r)) = _map.cell(_state.pose.position) else { return None; };

        g_scores.insert((c, r), 0);
        pq.push(Step {
            c, r,
            g_score: 0,
            h_score: self.h(c, r, cg, rg)
        });

        while !pq.is_empty() {
            let Step { c, r, g_score, h_score: _ } = pq.pop().expect("pq top element should exist");
            
            println!("{c}, {r}");

            if g_score != *g_scores.get(&(c, r)).unwrap_or(&usize::MAX) { continue; }
            
            let cur_point = _map.inv_cell(c, r);
            let dist = _goal.distance(cur_point);

            if dist <= _goal_tolerance_m {
                return Some(self.construct_path(_map, parent, (c, r)));
            }

            for (c_, r_) in adj(_map, c, r) {
                if !_map.clear(_map.inv_cell(c_, r_), _clearance_m) { continue; }

                let att_g_score = g_score + self.d(c, r, c_, r_);
                if att_g_score < *g_scores.get(&(c_, r_)).unwrap_or(&usize::MAX) {
                    g_scores.insert((c_, r_), att_g_score);
                    parent.insert((c_, r_), (c, r));
                    pq.push(Step {
                        c: c_,
                        r: r_,
                        g_score: att_g_score,
                        h_score: self.h(c_, r_, cg, rg)
                    });
                }
            }
        }

        None
    }
}
