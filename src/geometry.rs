extern crate nalgebra as na;

pub type JVector3 = self::na::Vector3<f64>;
pub type JGVector3 = self::na::Vector3<f32>; // Use this for graphics, ggez uses f32.
pub type JQuaternion = self::na::geometry::Quaternion<f64>;
pub type JUnitQuaternion = self::na::geometry::UnitQuaternion<f64>;

pub fn calc_norm_apprch_v (
    p1: &JVector3,
    p2: &JVector3,
    u1: &JVector3,
    u2: &JVector3,
) -> f64 {
    // Returns the norm of the approach velocity of the two points.

    let r = p2 - p1;

    // Approach velcocity due to p1 =   u1 * r / |r|
    // Approach velcocity due to p2 = - u2 * r / |r|
    // Aggregate approach velocity  = (u1 - u2) * r / |r|

    (u1 - u2).dot(&r) / r.norm()
}

// For the interpolation we can also use the velocities that are stored in the
// ball to calculate in a simpler way. Essenitally we would find the same
// result, but without the slerp/nlerp hack.

pub fn calc_interpolated_vector(
    v1: &JVector3,
    v2: &JVector3,
    t1: f64,
    t2: f64,
    t:  f64,
) -> JVector3 {
    let w = (t - t1) / (t2 - t1);
    (1. - w) * v1 + w * v2
}

pub fn calc_interpolated_quaternion(
    q1: &JUnitQuaternion,
    q2: &JUnitQuaternion,
    t1: f64,
    t2: f64,
    t:  f64,
) -> JUnitQuaternion {
    // We will use slerp. However, slerp as provided by nalgebra fails when the
    // angle between the quaternions is 180 degrees. My best guess is that this
    // happens because in this case the algorithm does not know which way to
    // turn. From the nalgebra docs it appears that nlerp does not have this
    // problem. So let's use nlerp for this rare case.
    let w = (t - t1) / (t2 - t1);
    match q1.try_slerp(&q2, w, 0.) {
        Some(slerped) => slerped,
        None => q1.nlerp(&q2, w),
    }
}

#[cfg(test)]
mod tests {

    fn close_enough(f1: f64, f2: f64) {
        assert!((f1 - f2).abs() < 1e-15)
    }

    #[test]
    fn test_norm_apprch_v_1() {
        use JVector3;
        use geometry::calc_norm_apprch_v;

        let p1 = JVector3::new(0., 0., 0.);
        let p2 = JVector3::new(1., 1., 1.);

        let u1 = JVector3::new(1., 1., 1.);
        let u2 = JVector3::new(-1., -1., -1.);

        close_enough(
            calc_norm_apprch_v(&p1, &p2, &u1, &u2),
            2. * u1.norm()
        );
    }

    #[test]
    fn test_norm_apprch_v_2() {
        use JVector3;
        use geometry::calc_norm_apprch_v;

        let p1 = JVector3::new(0., 0., 0.);
        let p2 = JVector3::new(1., 1., 1.);

        let u1 = JVector3::new(-1., -1., -1.);
        let u2 = JVector3::new(1., 1., 1.);

        close_enough(
            calc_norm_apprch_v(&p1, &p2, &u1, &u2),
            - 2. * u1.norm()
        );
    }

    #[test]
    fn test_norm_apprch_v_3() {
        use JVector3;
        use geometry::calc_norm_apprch_v;

        let p1 = JVector3::new(0., 0., 0.);
        let p2 = JVector3::new(1., 1., 1.);

        let u1 = JVector3::new(1., 1., 1.);
        let u2 = JVector3::new(1., 1., 1.);

        close_enough(
            calc_norm_apprch_v(&p1, &p2, &u1, &u2),
            0.
        );
    }

}
