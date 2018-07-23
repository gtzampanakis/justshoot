extern crate nalgebra as na;

mod geometry;

use geometry::{JVector3, JUnitQuaternion};

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
}

struct Ball {
    radius: f64,
    weight: f64,
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
    }
}

#[cfg(test)]
mod tests {
    use consts;
    use geometry::JUnitQuaternion;

    #[test]
    fn test_quaternions() {
        let q1 = JUnitQuaternion::identity();
        assert_eq!(q1, q1 * q1);
        assert_eq!(q1, q1.powf(12345.));

        let q2 = JUnitQuaternion::from_euler_angles(0., 0., 2. * consts::PI);
    }
}
