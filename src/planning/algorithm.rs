use std::{cmp::Ordering, collections::{BinaryHeap, HashMap, HashSet, VecDeque}};

use crate::{
    shared::{grid::Grid, types::{Path, Point2, VehicleState}},
};

#[derive(Copy, Clone, PartialEq)]
struct Step {
    c: usize,
    r: usize,
    d: usize,
}

impl Ord for Step {
    fn cmp(&self, other: &Self) -> Ordering {
        other.d.cmp(&self.d)
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
    let mut cv = vec![c];
    let mut rv = vec![r];
    if 0 < c { cv.push(c-1); }
    if c < map.width - 1 { cv.push(c+1); }
    if 0 < r { rv.push(r-1); }
    if r < map.height - 1 { rv.push(r+1); }
    for c_ in &cv {
        for r_ in &rv {
            v.push((c_.clone(), r_.clone()));
        }
    }
    // if 0 < c { v.push((c-1, r)); }
    // if c < map.width - 1 { v.push((c+1, r)); }
    // if 0 < r { v.push((c, r-1)); }
    // if r < map.height - 1 { v.push((c, r+1)) }
    v
}

#[derive(Hash, PartialEq, Eq, Clone, Copy)]
struct Vertex {
    c: usize,
    r: usize
}

impl Planner {

    fn d(&self, c1: usize, r1: usize, c2: usize, r2: usize) -> usize {
        octile_dist(c1, r1, c2, r2)
    }

    fn d_vertex(&self, v1: Vertex, v2: Vertex) -> usize {
        self.d(v1.c, v1.r, v2.c, v2.r)
    }

    fn construct_path(&self, _map: &Grid, parent: &HashMap<Vertex, Vertex>, end: Vertex) -> Path {
        let mut path: Path = vec![];

        let mut curr = Some(end);
        // let mut i = 0;
        while curr.is_some() {
            let v = curr.expect("curr should not be None");
            // println!("{} {}", v.c, v.r);
            path.push(_map.inv_cell(v.c, v.r));
            curr = parent.get(&v).copied();
        }

        path.reverse();
        path
    }

    fn sample_free(
        &self,
        _map: &Grid,
        _clearance_m: f64
    ) -> Vertex {
        // could be improved for saturated map case
        let mut v;
        loop {
            v = Vertex {
                c: rand::random_range(0.._map.width),
                r: rand::random_range(0.._map.height)
            };
            let p = _map.center(v.c, v.r).expect("should be a valid point");
            if _map.clear(p, _clearance_m) {
                break;
            }
        }
        v
    }

    fn nearest_dijkstras(
        &self,
        _map: &Grid,
        vertex: &Vertex,
        costs: &HashMap<Vertex, usize>
    ) -> Option<Vertex> {
        let mut pq: BinaryHeap<Step> = BinaryHeap::new();
        let mut vis: HashSet<Vertex> = HashSet::new();

        pq.push(Step { c: vertex.c, r: vertex.r, d: 0 });

        while !pq.is_empty() {
            let Step { c, r, d: _ } = pq.pop().expect("pq must have top element");

            let vert = Vertex { c, r };

            if vis.contains(&vert) { continue; }
            if costs.contains_key(&vert) { return Some(vert); }
            vis.insert(vert);

            for (c_, r_) in adj(_map, c, r) {
                pq.push(Step { c: c_, r: r_, d: self.d(c, r, c_, r_) });
            }
        }

        None
    }

    fn nearest_vertex_search(
        &self,
        vertex: &Vertex,
        costs: &HashMap<Vertex, usize>
    ) -> Option<Vertex> {
        let mut mn_dist = usize::MAX;
        let mut mn_vert = None;

        for vert in costs.keys() {
            let d = self.d_vertex(*vertex, *vert);

            if d < mn_dist {
                mn_dist = d;
                mn_vert = Some(vert);
            }
        }

        mn_vert.copied()
    }

    fn nearest(
        &self,
        _map: &Grid,
        vertex: &Vertex,
        costs: &HashMap<Vertex, usize>
    ) -> Option<Vertex> {
        let cells_per_element = _map.width * _map.height / costs.len();
        if cells_per_element < costs.len() {
            self.nearest_dijkstras(_map, vertex, costs)
        } else {
            self.nearest_vertex_search(vertex, costs)
        }
    }

    fn steer(
        &self,
        _map: &Grid,
        v1: Vertex,
        v2: Vertex,
        radius: f64,
    ) -> Option<Vertex> {
        let Some(p1) = _map.center(v1.c, v1.r) else { return None; };
        let Some(p2) = _map.center(v2.c, v2.r) else { return None; };
        
        let d = p1.distance(p2);  // around p1
        
        if d <= radius {
            return Some(v2);
        }

        // r < d now

        let pct = radius / d;

        let pres = Point2 {
            x: p1.x * (1.0 - pct) + p2.x * pct,
            y: p1.y * (1.0 - pct) + p2.y * pct
        };

        let Some((c, r)) = _map.cell(pres) else { return None; };

        Some(Vertex { c, r })
    }

    fn near(
        &self,
        _map: &Grid,
        origin: Vertex,
        costs: &HashMap<Vertex, usize>,
        radius: f64
    ) -> Option<Vec<Vertex>> {
        let mut res = vec![];

        let mut q: VecDeque<Vertex> = VecDeque::new();
        let mut vis: HashSet<Vertex> = HashSet::new();

        let Some(p_origin) = _map.center(origin.c, origin.r) else { return None; };

        q.push_back(origin);
        while !q.is_empty() {
            let x = q.pop_front().expect("q really should have front element.");

            if vis.contains(&x) { continue; }

            let p_x = _map.center(x.c, x.r).expect("we should've selected only indices that are in range");
            if p_origin.distance(p_x) > radius { continue; }

            vis.insert(x);
            
            if costs.contains_key(&x) {
                res.push(x);
            }
            
            for (c_, r_) in adj(_map, x.c, x.r) {
                q.push_back(Vertex { c: c_, r: r_ });
            }
        }

        return Some(res);
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
        let mut costs: HashMap<Vertex, usize> = HashMap::new();
        let mut parent: HashMap<Vertex, Vertex> = HashMap::new();

        let Some((gc, gr)) = _map.cell(_goal) else {
            // println!("6");
            return None;
        };
        let g_vert = Vertex { c: gc, r: gr };
        
        let start;
        match _map.cell(_state.pose.position) {
            Some(cell) => { start = Vertex { c: cell.0, r: cell.1 }; }
            None => {
                println!("4");
                return None;
            }
        };

        costs.insert(start, 0);

        let dim = 2.0;
        let area = (_map.width as f64 * _map.resolution_m) * (_map.height as f64 * _map.resolution_m);
        
        let gamma = 2.1 * ((1.0 + 1.0 / dim) as f64).powf(1.0 / dim) * (area / std::f64::consts::PI).powf(1.0 / dim);

        let rrt_iters = 30;
        for _ in 0..rrt_iters {
            let n = costs.len().max(2) as f64;

            let steer_radius = 30.0 * _map.resolution_m;
            let near_radius = (gamma * (n.ln() / n).powf(1.0 / dim)).min(steer_radius);

            // println!("{steer_radius}, {near_radius}");

            let x_rand;
            if rand::random_ratio(1, 7) {
                x_rand = g_vert;
            } else {
                x_rand = self.sample_free(_map, _clearance_m);
            }
            let Some(x_nearest) = self.nearest(_map, &x_rand, &costs) else {
                // println!("5");
                return None;
            };
            let x_new = self.steer(_map, x_nearest, x_rand, steer_radius).expect("x_new really shouldn't be None");

            let px_new = _map.center(x_new.c, x_new.r).expect("x_new should be in range");
            let px_nearest = _map.center(x_nearest.c, x_nearest.r).expect("x_nearest should be in range");

            if !costs.contains_key(&x_new) && _map.segment_clear(px_nearest, px_new, _clearance_m) {
                let mut nearby = self.near(_map, x_new, &costs, near_radius).expect("x_new should've been in range");

                nearby.push(x_nearest);

                let mut mn_x = None;
                let mut mn_c = usize::MAX;

                for x_near in &nearby {
                    let att_c = self.d_vertex(*x_near, x_new) + costs.get(x_near).expect("x_near really should be in range");
                    let px_near = _map.center(x_near.c, x_near.r).expect("x_near should really be in range");
                    if _map.segment_clear(px_near, px_new, _clearance_m) && att_c < mn_c {
                        mn_c = att_c;
                        mn_x = Some(x_near);
                    }
                }

                let mn_x = mn_x.expect("mn_x really should have been set the first time around");

                parent.insert(x_new, *mn_x);
                // if x_new == *mn_x {
                //     println!("{} {}", costs.contains_key(&x_new), costs.contains_key(mn_x));
                // }

                costs.insert(x_new, mn_c);
                
                for x_near in &nearby {
                    let att_c = self.d_vertex(x_new, *x_near) + costs.get(&x_new).expect("x_new should really have a cost");
                    let px_near = _map.center(x_near.c, x_near.r).expect("x_near should really be in range");

                    if x_near != mn_x && _map.segment_clear(px_new, px_near, _clearance_m) && att_c < *costs.get(x_near).expect("x_near should really have a cost") {
                        parent.insert(*x_near, x_new);
                        costs.insert(*x_near, att_c);
                    }
                }
            }
        }
        
        let mut mn_x: Option<Vertex> = self.nearest(_map, &g_vert, &costs);
        let mut mn_d = f64::MAX;

        // println!("HI1");
        // for g_near in self.near(_map, g_vert, &costs, _goal_tolerance_m).expect("goal not in range should've already been eliminated") {
        //     let pg_near = _map.center(g_near.c, g_near.r).expect("g_near really should be in range");
        //     let d = pg_near.distance(_goal);
        //     if d < mn_d {
        //         mn_d = d;
        //         mn_x = Some(g_near);
        //     }
        // }
        // println!("HI2");

        if let Some(mn_x) = mn_x {
            let p_mn_x = _map.center(mn_x.c, mn_x.r).expect("mn_x should be in range");
            
            if p_mn_x.distance(_goal) <= _goal_tolerance_m {
                Some(self.construct_path(_map, &parent, mn_x))
            } else {
                None
                // parent.insert(g_vert, mn_x);
                // Some(self.construct_path(_map, &parent, g_vert))
            }
        } else {
            None
        }

        // let Some(mn_x) = mn_x else {
        //     // println!("7");
        //     return None;
        // };

        // println!("HI3");
        // println!("({gc}, {gr})");
        // Some(self.construct_path(_map, &parent, mn_x))
    }
}
