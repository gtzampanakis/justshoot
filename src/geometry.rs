extern crate nalgebra as na;

use self::na::Vector3 as NVector3;
use self::na::geometry::UnitQuaternion as NUnitQuaternion;

pub type JVector3 = NVector3<f64>;
pub type JUnitQuaternion = NUnitQuaternion<f64>;

