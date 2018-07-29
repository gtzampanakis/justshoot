extern crate nalgebra as na;

use self::na::Vector3 as NVector3;
use self::na::geometry::UnitQuaternion as NUnitQuaternion;

pub type JVector3 = NVector3<f64>;
pub type JUnitQuaternion = NUnitQuaternion<f64>;

pub fn posv(p1: &JVector3, p2: &JVector3) -> JVector3 {
    // Returns the vector that joins p1 to p2.
    return p2 - p1;
}

pub fn norm_apprch_v (
    p1: &JVector3,
    p2: &JVector3,
    u1: &JVector3,
    u2: &JVector3,
) -> f64 {
    // Returns the squared norm of the approach velocity of the two points.

    let r = posv(p1, p2);

    // Approach velcocity due to p1 =   u1 * r / |r|
    // Approach velcocity due to p2 = - u2 * r / |r|
    // Aggregate approach velocity  = (u1 - u2) * r / |r|

    (u1 - u2).dot(&r) / r.norm()
}

#[cfg(test)]
mod tests {

    fn close_enough(f1: f64, f2: f64) {
        assert!((f1 - f2).abs() < 1e-15)
    }

    #[test]
    fn test_norm_apprch_v_1() {
        use JVector3;
        use geometry::norm_apprch_v;

        let p1 = JVector3::new(0., 0., 0.);
        let p2 = JVector3::new(1., 1., 1.);

        let u1 = JVector3::new(1., 1., 1.);
        let u2 = JVector3::new(-1., -1., -1.);

        close_enough(
            norm_apprch_v(&p1, &p2, &u1, &u2),
            2. * u1.norm()
        );
    }

    #[test]
    fn test_norm_apprch_v_2() {
        use JVector3;
        use geometry::norm_apprch_v;

        let p1 = JVector3::new(0., 0., 0.);
        let p2 = JVector3::new(1., 1., 1.);

        let u1 = JVector3::new(-1., -1., -1.);
        let u2 = JVector3::new(1., 1., 1.);

        close_enough(
            norm_apprch_v(&p1, &p2, &u1, &u2),
            - 2. * u1.norm()
        );
    }

    #[test]
    fn test_norm_apprch_v_3() {
        use JVector3;
        use geometry::norm_apprch_v;

        let p1 = JVector3::new(0., 0., 0.);
        let p2 = JVector3::new(1., 1., 1.);

        let u1 = JVector3::new(1., 1., 1.);
        let u2 = JVector3::new(1., 1., 1.);

        close_enough(
            norm_apprch_v(&p1, &p2, &u1, &u2),
            0.
        );
    }

}
