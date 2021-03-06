#![allow(dead_code)]
#![allow(unused_variables)]

use geometry::{
    JVector3,
    JUnitVector3,
    JUnitQuaternion,
    calc_norm_apprch_v,
    calc_interpolated_vector,
    calc_interpolated_quaternion,
    rotate_point,
};

pub mod consts {
    /* This is not intended to be used directly. Rather, values should be
     * copied to WorldConf or to any other place in which they are needed. */
    pub const PI: f64 = ::std::f64::consts::PI;
    pub const GRAVITY: f64 = -9.81;
    pub const POOL_BALL_RADIUS: f64 = 57.2 / 1000. / 2.;
    pub const POOL_BALL_WEIGHT: f64 = 165. / 1000.;

    pub const BALL_BALL_REST: f64 = 0.95;
    pub const BALL_CLOTH_REST: f64 = 0.50;

    pub const BALL_SPOT_RADIUS_FACTOR: f64 = 0.1;

    pub const ANGLE_MODICUM: f64 = 2. * ::std::f64::consts::PI / 10000.;

    /* Here are some data taken from http://billiards.colostate.edu/threads/physics.html:
    
    ball diameter: 2.25 in
    ball mass: 6 oz
    ball mass moment of inertia: 2/5 mR2
    ball-ball coefficient of friction (μ): 0.03-0.08
    ball-ball coefficient of restitution (e): 0.92-0.98
    ball-cloth coefficient of rolling resistance (μ): 0.005 - 0.015
    ball-cloth coefficient of sliding friction (μ): 0.15-0.4 (typical value: 0.2)
    ball-cloth spin deceleration rate: 5-15 rad/sec2
    ball-rail coefficient of restitution (e): 0.6-0.9
    ball-table coefficient of restitution (e): 0.5
    cue-tip-ball coefficient of friction (μ): 0.6
    cue-tip-ball coefficient of restitution (e): 0.71-0.75 (leather tip), 0.81-0.87 (phenolic tip) 

    touch: 1.5 mph = 2.2 fps
    slow: 3 mph = 4.4 fps
    medium-soft: 5 mph = 7.3 fps
    medium: 7 mph = 10 fps
    medium-fast: 8 mph = 12 fps
    fast: 12 mph = 18 fps
    power: 15-20 mph = 22-29 fps
    powerful break: 25-30 mph = 36-44 fps

    */
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
    pub ball_cloth_rest: f64,
    pub ball_spot_poss: Vec<JUnitVector3>,
    pub ball_spot_radius_factor: f64,
    pub gravity: f64,
}

pub struct DebugConf {
    should_print_collisions: bool,
}

#[derive(Clone)]
pub struct Ball {
    pub pos: JVector3,
    pub u: JVector3,
    pub rot: JUnitQuaternion,
    // Using separate axis and angle for the rotation velocity. We cannot
    // simply use a unit quaternion because it will not allow us to represent a
    // rotation faster than 2*pi radians per second.
    pub urot_axis: JUnitVector3,
    pub urot_angle: f64,
}

impl Ball {
    fn apply_velocities(&mut self, ts: f64) {
        // println!("apply_velocities: pos.z: {:?}, u.z: {:?}", self.pos.z, self.u.z);
        self.pos += self.u * ts;
        let angle = self.urot_angle * ts;
        let urot = JUnitQuaternion::from_axis_angle(
            &self.urot_axis, angle);
        self.rot = urot * self.rot;
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
                            urot_axis: JUnitVector3::new_normalize(JVector3::new(1., 0., 0.)),
                            urot_angle: 0.,
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

struct BallBallCollisionEvent {
    i: usize, // index of ball_a
    j: usize, // index of ball_b
    unit_normal: JVector3,
}

struct BallClothCollisionEvent {
    i: usize, // index of ball
    unit_normal: JVector3,
}

pub struct Simulator {
    pub balls: Vec<Ball>,
    pub world_conf: WorldConf,
    pub debug_conf: DebugConf,
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
            debug_conf: DebugConf {
                should_print_collisions: false,
            },
            ts: ts,
            t: 0.,
            t_hard_limit: 30.,
        }
    }

    fn apply_gravity(&mut self) {
        for ball in self.balls.iter_mut() {
            if ball.pos.z > self.world_conf.ball_radius {
                let before = ball.u.z;
                ball.u.z += self.world_conf.gravity * self.ts;
                // println!("apply_gravity: ball.u.z before: {:?} ball.u.z after: {:?}",
                //          before, ball.u.z);
            }
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
        self.check_ball_to_ball_collisions();
        self.check_ball_to_cloth_collisions();
        // check_snap_to_cloth can only fullfil its purpose if it is called
        // after collisions to the cloth have applied changes to the velocities
        // but before the velocities have made changes to the ball positions.
        self.check_snap_to_cloth();
        self.apply_gravity();

        // Let's keep this in the end because it is the only function that
        // changes positions. No concrete reason, just so that the whole
        // process is easier to reason about.
        self.apply_ball_velocities();
        // println!("");
        self.t += self.ts;

        SimulationState::from_simulator(self)
    }

    fn adjust_for_ball_to_ball_collisions(&mut self, coll_ev: &BallBallCollisionEvent) {
        // The balls exchange the velocity vector components that coincide with
        // the normal vector of the collision.

        let comp_a: JVector3;
        let comp_b: JVector3;

        {
            let ball_a = &self.balls[coll_ev.i];
            let ball_b = &self.balls[coll_ev.j];

            comp_a = ball_a.u.dot(&coll_ev.unit_normal) * coll_ev.unit_normal;
            comp_b = ball_b.u.dot(&coll_ev.unit_normal) * coll_ev.unit_normal;

            // println!("{:?}", comp_a);
            // println!("{:?}", comp_b);
            // println!("");
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

    fn adjust_for_ball_to_cloth_collisions(&mut self, coll_ev: &BallClothCollisionEvent) {
        let ball = &mut self.balls[coll_ev.i];
        let comp = ball.u.dot(&coll_ev.unit_normal) * coll_ev.unit_normal;
        let before = ball.u.z;
        ball.u -= comp;
        ball.u -= comp * self.world_conf.ball_cloth_rest;
        // println!("adjust_for_ball_to_cloth_collisions: ball.u.z before: {:?} ball.u.z after: {:?}",
        //          before, ball.u.z);
    }

    fn check_snap_to_cloth(&mut self) {
        // There is a phenomenon that necessarily occurs due to the step-wise
        // fashion in which we have to do the updates to the velocities.
        // Consider this case:
        //
        // * The ball is moving towards the cloth with velocity u1.
        // * The ball bounces against the cloth which causes its velocity to be
        //      u2 = - u1 * r
        //       or
        //      u1 = - u2 / r
        //   where r is the restitution.
        // * Gravity is applied to the ball and its velocity becomes
        //      u3 = u2 + g*ts
        //
        // There will unavoidably exist the possibility that 
        //      u1 = u3
        //       or
        //      - u2 / r = u2 + g*ts
        //       or
        //      - u2 = r * u2 + r*g*ts
        //       or
        //      u2 = - g*ts * r/(r+1)
        //
        // Since g is negative u2 will be positive.
        //
        // This will cause the velocity to alternate between u1 and u2
        // forever. As can be seen, this case can occur no matter what the
        // values of r, g, ts. But it can only occur when the value of u2 is
        // very small. r/(r+1) cannot be more than 1 so the phenomenon can only
        // occur when u2 is equal or less to g*ts.
        //
        // We can take into account the value of r to find a tighter threshold
        // but I think it is better to avoid such complications.
        //
        // Snapping the ball to the cloth when u2 is equal or less to this
        // value will avoid the phenomenon.
        // 
        // It is important that this function is called after collisions have
        // modified the velocities but before any changes to the positions have
        // been made.
        let snap_threshold = -self.world_conf.gravity * self.ts;
        for ball in self.balls.iter_mut() {
            if (
                ball.pos.z <= self.world_conf.ball_radius // This means ball.u corresponds to 
                                                          // the u2 mentioned in the analysis above.
                    &&
                ball.u.z > 0.
                    &&
                ball.u.z <= snap_threshold
            ) {
                println!("{:?}", ball.pos.z);
                ball.pos.z = self.world_conf.ball_radius;
                ball.u.z = 0.;
            }
        }
    }

    fn check_ball_to_ball_collisions(&mut self) {
        // We only care about collisions with balls that are approaching each
        // other. Non-approaching balls colliding is an artifact of the
        // simulation process, which allows balls to penetrate each other.

        let n_balls = self.balls.len();

        for i in 0 .. n_balls-1 {
            for j in i+1 .. n_balls {

                let mut coll_ev_maybe: Option<BallBallCollisionEvent> = None;
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
                            if r_norm <= 2. * self.world_conf.ball_radius {
                                // Balls are colliding.
                                coll_ev_maybe = Some(
                                    BallBallCollisionEvent {
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
                    self.adjust_for_ball_to_ball_collisions(&coll_ev);
                }

            }
        }

    }

    fn check_ball_to_cloth_collisions(&mut self) {
        let n_balls = self.balls.len();
        for i in 0 .. n_balls {
            let mut coll_ev_maybe: Option<BallClothCollisionEvent> = None;
            {
                let ball = &self.balls[i];

                if ball.u.z <= 0. {
                    // Ball is approaching the cloth.
                    if ball.pos.z <= self.world_conf.ball_radius {
                        // Ball is colliding with the cloth.
                        coll_ev_maybe = Some(
                            BallClothCollisionEvent {
                                i: i,
                                unit_normal: JVector3::new(0., 0., 1.,),
                            }
                        );
                        // println!("{:?}", ball.pos.z);
                    }
                }
            }

            if let Some(coll_ev) = coll_ev_maybe {
                if self.debug_conf.should_print_collisions {
                    println!("Ball-to-cloth collision. Ball: {:?}", coll_ev.i);
                }
                self.adjust_for_ball_to_cloth_collisions(&coll_ev);
                self.adjust_ball_for_spin(i, &coll_ev.unit_normal);
            }
        }
    }

    fn adjust_ball_for_spin(&mut self, ball_i: usize, unit_normal: &JVector3) {
        let ball = &mut self.balls[ball_i];
        let a_dot_n = ball.urot_axis.dot(unit_normal);
        if a_dot_n > 0. {
            let k = self.world_conf.ball_radius / a_dot_n;
            let x = k * ball.urot_axis.unwrap();
            let rotated_center = rotate_point(
                &(ball.pos - x),
                &JUnitQuaternion::from_axis_angle(
                    &JUnitVector3::new_normalize(JVector3::new(unit_normal.x, unit_normal.y, unit_normal.z)),
                    ball.urot_angle * a_dot_n * 0.0001
                ),
            ) + x;
            ball.pos = rotated_center;
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
