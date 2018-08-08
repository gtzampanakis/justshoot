#![allow(dead_code)]
#![allow(unused_variables)]

use geometry::{
    JVector3,
    JUnitQuaternion,
    calc_norm_apprch_v,
    calc_interpolated_vector,
    calc_interpolated_quaternion,
};

pub mod consts {
    /* This is not intended to be used directly. Rather, values should be
     * copied to WorldConf or to any other place in which they are needed. */
    pub const PI: f64 = ::std::f64::consts::PI;
    pub const POOL_BALL_RADIUS: f64 = 57.2 / 1000. / 2.;
    pub const POOL_BALL_WEIGHT: f64 = 165. / 1000.;

    pub const ANGLE_MODICUM: f64 = 2. * ::std::f64::consts::PI / 10000.;
}

pub struct WorldConf {
    /* Common diameters:
        carom:   61.5 mm
        pool:    57.2 mm
        snooker: 52.5 mm 

       Common weights:
        pool:    165 g
    */
    pub ball_radius: f64,
    pub ball_weight: f64,
    pub ball_ball_rest: f64,
}

#[derive(Clone)]
pub struct Ball {
    pub pos: JVector3,
    pub u: JVector3,
    pub rot: JUnitQuaternion,
    pub urot: JUnitQuaternion,
}

impl Ball {
    fn apply_velocities(&mut self, ts: f64) {
        self.pos += self.u * ts;
        self.rot =  self.urot.powf(ts) * self.rot;
    }
}

pub struct SimulationState {
    t: f64,
    balls: Vec<Ball>,
}

impl SimulationState {
    fn from_simulator(simulator: &Simulator) -> Self {
        SimulationState {
            t: simulator.t,
            balls: simulator.balls.clone(),
        }
    }
}

pub struct SimulationStateSeq {
    pub states: Vec<SimulationState>,
}

impl SimulationStateSeq {

    pub fn calc_interpolated_at(&self, t: f64) -> SimulationState {
        let sl = self.states.len();

        if sl == 0 {
            panic!("calc_interpolated_at called on empty SimulationStateSeq");
        }

        if t <= self.states[0].t {
            SimulationState {
                t: t,
                balls: self.states[0].balls.clone(),
            }
        }
        else if t >= self.states[sl-1].t {
            SimulationState {
                t: t,
                balls: self.states[sl-1].balls.clone(),
            }
        }
        else {
            // At this point we have at least two states. If there was only one
            // state then one of the two preceding conditions would have been
            // true.
            let mut interpolated_balls;

            for i in 0..sl-1 {
                let (state1, state2) = (&self.states[i], &self.states[i+1]);

                if state1.t <= t && state2.t >= t {
                    interpolated_balls = Vec::new();

                    for i in 0..state1.balls.len() {
                        interpolated_balls.push(Ball{
                            pos: calc_interpolated_vector(
                                &state1.balls[i].pos,
                                &state2.balls[i].pos,
                                state1.t,
                                state2.t,
                                t
                            ),
                            rot: calc_interpolated_quaternion(
                                &state1.balls[i].rot,
                                &state2.balls[i].rot,
                                state1.t,
                                state2.t,
                                t
                            ),
                            // It doesn't matter what we put in the velocities
                            // because this information will only be used for
                            // drawing. We are wasting some CPU cycles and some
                            // memory but it doesn't matter because
                            // interpolation is not in our critical path, I
                            // think.
                            u: JVector3::zeros(),
                            urot: JUnitQuaternion::identity(),
                        });
                    }

                    return SimulationState {
                        t: t,
                        balls: interpolated_balls,
                    }
                }
            }

            panic!("calc_interpolated_at finished without finding a result. This is unexpected.");
        }
    }

}

struct CollisionEvent {
    i: usize, // index of ball_a
    j: usize, // index of ball_b
    unit_normal: JVector3,
}

pub struct Simulator {
    pub balls: Vec<Ball>,
    pub world_conf: WorldConf,
// timestep. Keep it here to retain the option of altering its value
// dynamically.
    ts: f64,
    pub t: f64,
    t_hard_limit: f64,
}

impl Simulator {

    pub fn new(
        balls: Vec<Ball>,
        world_conf: WorldConf,
        ts: f64,
    ) -> Self {
        Simulator {
            balls: balls,
            world_conf: world_conf,
            ts: ts,
            t: 0.,
            t_hard_limit: 30.,
        }
    }

    fn apply_ball_velocities(&mut self) {
        for ball in self.balls.iter_mut() {
            ball.apply_velocities(self.ts);
        }
    }

    pub fn run_complete_simulation(&mut self) -> SimulationStateSeq {
        let mut states = Vec::new();

        loop {
            self.progress();
            if self.t >= self.t_hard_limit {
                break;
            }
            let simulation_state = SimulationState{
                t: self.t,
                balls: self.balls.clone(),
            };
            states.push(simulation_state);
        }

        SimulationStateSeq{states: states}
    }

    pub fn progress(&mut self) -> SimulationState {
        self.apply_ball_velocities();
        self.check_ball_to_ball_collisions();
        self.t += self.ts;

        SimulationState::from_simulator(self)
    }

    fn adjust_ball_to_ball_collisions(&mut self, coll_ev: &CollisionEvent) {
        // The balls exchange the velocity vector components that coincide with
        // the normal vector of the collision.

        let comp_a: JVector3;
        let comp_b: JVector3;

        {
            let ball_a = &self.balls[coll_ev.i];
            let ball_b = &self.balls[coll_ev.j];

            comp_a =   ball_a.u.dot(&coll_ev.unit_normal) * coll_ev.unit_normal;
            comp_b = - ball_b.u.dot(&coll_ev.unit_normal) * coll_ev.unit_normal;
        }

        {
            let ball_a = &mut self.balls[coll_ev.i];
            ball_a.u -= comp_a;
            ball_a.u += comp_b * self.world_conf.ball_ball_rest;
        }

        {
            let ball_b = &mut self.balls[coll_ev.j];
            ball_b.u -= comp_b;
            ball_b.u += comp_a * self.world_conf.ball_ball_rest;
        }

    }

    fn check_ball_to_ball_collisions(&mut self) {
        // We only care about collisions with balls that are approaching each
        // other. Non-approaching balls colliding is an artifact of the
        // simulation process, which allows balls to penetrate each other.

        let n_balls = self.balls.len();

        for i in 0 .. n_balls-1 {
            for j in i+1 .. n_balls {

                let mut coll_ev_maybe: Option<CollisionEvent> = None;
                {
                    let ball_a = &self.balls[i];
                    let ball_b = &self.balls[j];

                    let norm_apprch_v = calc_norm_apprch_v(
                        &ball_a.pos,
                        &ball_b.pos,
                        &ball_a.u,
                        &ball_b.u,
                    );

                    let r = ball_b.pos - ball_a.pos;
                    let r_norm = r.norm();

                    // println!("a_pos: {:?}", ball_a.pos);
                    // println!("b_pos: {:?}", ball_b.pos);
                    // println!("{:?}", r_norm);
                    // println!("");

                    if r_norm > 0. {
                        // Avoids the division-by-zero case where balls are in the
                        // same place.
                        if norm_apprch_v > 0. {
                            // Balls are approaching.
                            if r_norm < 2. * self.world_conf.ball_radius {
                                // Balls are colliding.
                                coll_ev_maybe = Some(
                                    CollisionEvent {
                                        i: i,
                                        j: j,
                                        unit_normal: r / r_norm,
                                    }
                                );
                            }
                        }
                    }
                }

                if let Some(coll_ev) = coll_ev_maybe {
                    self.adjust_ball_to_ball_collisions(&coll_ev);
                }

            }
        }

    }
}

struct Drawer {

}

impl Drawer {

}

#[cfg(test)]
mod tests {
    use consts;
    use Ball;
    use Simulator;
    use WorldConf;
    use geometry::{JVector3, JUnitQuaternion};

    #[test]
    fn test_quaternions() {
        let q1 = JUnitQuaternion::identity();
        assert_eq!(q1, q1 * q1);
        assert_eq!(q1, q1.powf(12345.));

        let q2 = JUnitQuaternion::from_euler_angles(0., 0., 2. * consts::PI);
    }

    fn setup_test_check_ball_to_ball_collisions() -> Simulator {
        let world_conf = WorldConf {
            ball_radius: consts::POOL_BALL_RADIUS,
            ball_weight: consts::POOL_BALL_WEIGHT,
        };

        let balls = vec![
            Ball {
                pos: JVector3::new(0., 0., 0.),
                u: JVector3::new(1., 0.001, 0.),
                rot: JUnitQuaternion::identity(),
                urot: JUnitQuaternion::identity(),
            },
            Ball {
                pos: JVector3::new(1., 0., 0.),
                u: JVector3::new(0., 0., 0.),
                rot: JUnitQuaternion::identity(),
                urot: JUnitQuaternion::identity(),
            },
        ];

        let simulator = Simulator::new(
            balls,
            world_conf,
            1e-3,
        );

        simulator
    }

    #[test]
    fn test_check_ball_to_ball_collisions() {
        let mut simulator = setup_test_check_ball_to_ball_collisions();

        for stepi in 0..1000 {
            simulator.progress();
            println!("{:?} {:?}", simulator.balls[0].pos, simulator.balls[1].pos);
            println!("{:?} {:?}", simulator.balls[0].u, simulator.balls[1].u);
            println!("");
        }
    }
}
