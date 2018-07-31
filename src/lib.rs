#![allow(dead_code)]
#![allow(unused_variables)]

mod geometry;

use geometry::{JVector3, JUnitQuaternion, calc_norm_apprch_v};

mod consts {
    /* This is not intended to be used directly. Rather, values should be
     * copied to WorldConf or to any other place in which they are needed. */
    pub const PI: f64 = ::std::f64::consts::PI;
    pub const POOL_BALL_RADIUS: f64 = 57.2 / 1000. / 2.;
    pub const POOL_BALL_WEIGHT: f64 = 165. / 1000.;

    pub const ANGLE_MODICUM: f64 = 2. * ::std::f64::consts::PI / 10000.;
}

struct WorldConf {
    /* Common diameters:
        carom:   61.5 mm
        pool:    57.2 mm
        snooker: 52.5 mm 

       Common weights:
        pool:    165 g
    */
    ball_radius: f64,
    ball_weight: f64,
}

struct Ball {
    pos: JVector3,
    u: JVector3,
    rot: JUnitQuaternion,
    urot: JUnitQuaternion,
}

impl Ball {
    fn apply_velocities(&mut self, ts: f64) {
        self.pos += self.u * ts;
        self.rot =  self.urot.powf(ts) * self.rot;
    }
}

struct CollisionEvent {
    i: usize, // index of ball_a
    j: usize, // index of ball_b
    unit_normal: JVector3,
}

struct Simulator {
    balls: Vec<Ball>,
    world_conf: WorldConf,
// timestep. Keep it here to retain the option of altering its value
// dynamically.
    ts: f64,
}

impl Simulator {
    fn apply_ball_velocities(&mut self) {
        for ball in self.balls.iter_mut() {
            ball.apply_velocities(self.ts);
        }
    }

    fn progress(&mut self) {
        self.apply_ball_velocities();
        self.check_ball_to_ball_collisions();
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
            ball_a.u += comp_b;
        }

        {
            let ball_b = &mut self.balls[coll_ev.j];
            ball_b.u -= comp_b;
            ball_b.u += comp_a;
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

        let simulator = Simulator {
            balls: balls,
            world_conf: world_conf,
            ts: 1e-3,
        };

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
