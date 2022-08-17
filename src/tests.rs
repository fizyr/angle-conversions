use approx::assert_relative_eq;
use nalgebra as na;

use crate::{
	convert_axis_angle_to_euler_xyz,
	convert_axis_angle_to_euler_yxz,
	convert_axis_angle_to_euler_zxy,
	convert_axis_angle_to_euler_zyx,
	convert_axis_angle_to_euler_yzx,
	convert_axis_angle_to_euler_xzy,
	convert_euler_xyz_to_axis_angle,
	convert_euler_yxz_to_axis_angle,
	convert_euler_zxy_to_axis_angle,
	convert_euler_zyx_to_axis_angle,
	convert_euler_yzx_to_axis_angle,
	convert_euler_xzy_to_axis_angle

};

#[test]
fn test_convert_axis_angle_to_euler() {

	// Using random values (vector is normalized).
	let rx = 0.35000955;
	let ry = 0.73582662;
	let rz = 0.57970035;
	let angle = 0.26805072;

	// Expected values are computed using https://www.andre-gaschler.com/rotationconverter/

	let (euler_rx, euler_ry, euler_rz) = convert_axis_angle_to_euler_xyz(rx, ry, rz, angle);
	assert_relative_eq!(euler_rx, 0.0791835, epsilon = 1e-6);
	assert_relative_eq!(euler_ry, 0.2035335, epsilon = 1e-6);
	assert_relative_eq!(euler_rz, 0.1479187, epsilon = 1e-6);

	let (euler_ry, euler_rx, euler_rz) = convert_axis_angle_to_euler_yxz(rx, ry, rz, angle);
	assert_relative_eq!(euler_ry, 0.2041556, epsilon = 1e-6);
	assert_relative_eq!(euler_rx, 0.0775457, epsilon = 1e-6);
	assert_relative_eq!(euler_rz, 0.1639563, epsilon = 1e-6);

	let (euler_rz, euler_rx, euler_ry) = convert_axis_angle_to_euler_zxy(rx, ry, rz, angle);
	assert_relative_eq!(euler_rz, 0.1457008, epsilon = 1e-6);
	assert_relative_eq!(euler_rx, 0.1081443, epsilon = 1e-6);
	assert_relative_eq!(euler_ry, 0.1898811, epsilon = 1e-6);

	let (euler_rz, euler_ry, euler_rx) = convert_axis_angle_to_euler_zyx(rx, ry, rz, angle);
	assert_relative_eq!(euler_rz, 0.1664424, epsilon = 1e-6);
	assert_relative_eq!(euler_ry, 0.1887585, epsilon = 1e-6);
	assert_relative_eq!(euler_rx, 0.1101078, epsilon = 1e-6);

	let (euler_ry, euler_rz, euler_rx) = convert_axis_angle_to_euler_yzx(rx, ry, rz, angle);
	assert_relative_eq!(euler_ry, 0.1913399, epsilon = 1e-6);
	assert_relative_eq!(euler_rz, 0.1634591, epsilon = 1e-6);
	assert_relative_eq!(euler_rx, 0.0785954, epsilon = 1e-6);

	let (euler_rx, euler_rz, euler_ry) = convert_axis_angle_to_euler_xzy(rx, ry, rz, angle);
	assert_relative_eq!(euler_rx, 0.1092933, epsilon = 1e-6);
	assert_relative_eq!(euler_rz, 0.1448436, epsilon = 1e-6);
	assert_relative_eq!(euler_ry, 0.2057181, epsilon = 1e-6);
}

#[test]
fn test_convert_euler_to_axis_angle() {

	// Using random values.
	let euler_rx = 0.750436300174;
	let euler_ry = 1.623036649214;
	let euler_rz = 1.064572425829;

	// Expected values are computed using https://www.andre-gaschler.com/rotationconverter/

	let (rx, ry, rz, angle) = convert_euler_xyz_to_axis_angle(euler_rx, euler_ry, euler_rz);
	assert_relative_eq!(rx,    0.615986, epsilon = 1e-6);
	assert_relative_eq!(ry,    0.4989121, epsilon = 1e-6);
	assert_relative_eq!(rz,    0.6096294, epsilon = 1e-6);
	assert_relative_eq!(angle, 2.2813373, epsilon = 1e-6);

	let (rx, ry, rz, angle) = convert_euler_yxz_to_axis_angle(euler_rx, euler_ry, euler_rz);
	assert_relative_eq!(rx,    0.7702762, epsilon = 1e-6);
	assert_relative_eq!(ry,    0.6238781, epsilon = 1e-6);
	assert_relative_eq!(rz,    0.1321012, epsilon = 1e-6);
	assert_relative_eq!(angle, 1.6274563, epsilon = 1e-6);

	let (rx, ry, rz, angle) = convert_euler_zxy_to_axis_angle(euler_rx, euler_ry, euler_rz);
	assert_relative_eq!(rx,   -0.1376612, epsilon = 1e-6);
	assert_relative_eq!(ry,    0.7806416, epsilon = 1e-6);
	assert_relative_eq!(rz,    0.6096294, epsilon = 1e-6);
	assert_relative_eq!(angle, 2.2813373, epsilon = 1e-6);

	let (rx, ry, rz, angle) = convert_euler_zyx_to_axis_angle(euler_rx, euler_ry, euler_rz);
	assert_relative_eq!(rx,   -0.1721422, epsilon = 1e-6);
	assert_relative_eq!(ry,    0.9761743, epsilon = 1e-6);
	assert_relative_eq!(rz,    0.1321012, epsilon = 1e-6);
	assert_relative_eq!(angle, 1.6274563, epsilon = 1e-6);

	let (rx, ry, rz, angle) = convert_euler_yzx_to_axis_angle(euler_rx, euler_ry, euler_rz);
	assert_relative_eq!(rx,    0.615986, epsilon = 1e-6);
	assert_relative_eq!(ry,    0.7806416, epsilon = 1e-6);
	assert_relative_eq!(rz,    0.1056407, epsilon = 1e-6);
	assert_relative_eq!(angle, 2.2813373, epsilon = 1e-6);

	let (rx, ry, rz, angle) = convert_euler_xzy_to_axis_angle(euler_rx, euler_ry, euler_rz);
	assert_relative_eq!(rx,   -0.1721422, epsilon = 1e-6);
	assert_relative_eq!(ry,    0.6238781, epsilon = 1e-6);
	assert_relative_eq!(rz,    0.7623275, epsilon = 1e-6);
	assert_relative_eq!(angle, 1.6274563, epsilon = 1e-6);
}

fn rand(random: &mut impl rand::Rng, min: f64, max: f64) -> f64 {
	use rand::distributions::Distribution;

	let dist = rand::distributions::Uniform::new(min, max);
	dist.sample(random)
}

fn rotation(axis_x: f64, axis_y: f64, axis_z: f64, angle: f64) -> na::UnitQuaternion<f64> {
	let axis = na::Vector3::new(axis_x, axis_y, axis_z);
	let axis = na::UnitVector3::new_normalize(axis);
	na::UnitQuaternion::from_axis_angle(&axis, angle)
}

fn rot_x(angle: f64) -> na::UnitQuaternion<f64> {
	rotation(1.0, 0.0, 0.0, angle)
}

fn rot_y(angle: f64) -> na::UnitQuaternion<f64> {
	rotation(0.0, 1.0, 0.0, angle)
}

fn rot_z(angle: f64) -> na::UnitQuaternion<f64> {
	rotation(0.0, 0.0, 1.0, angle)
}

macro_rules! assert_vec {
	($vec_a:expr, $vec_b: expr, $epsilon:expr) => {
		{
			let vec_a: nalgebra::Vector<_, _, _> = $vec_a;
			let vec_b: nalgebra::Vector<_, _, _> = $vec_b;
			let distance = (vec_a - vec_b).norm();
			let epsilon = $epsilon;
			if distance > epsilon {
				::assert2::assert!($vec_a == $vec_b, "distance ({distance}) > epsilon ({epsilon})")
			}
		}
	}
}

fn random_samples() -> impl Iterator<Item = (f64, f64, f64, f64)> {
	let mut random: rand::rngs::StdRng = rand::SeedableRng::seed_from_u64(0);
	(0..1_000).into_iter().map(move |_| {
		let axis_x = rand(&mut random, 0.0, 1.0);
		let axis_y = rand(&mut random, 0.0, 1.0);
		let axis_z = rand(&mut random, 0.0, 1.0);
		let angle = rand(&mut random, 0.0, std::f64::consts::TAU);
		(axis_x, axis_y, axis_z, angle)
	})
}

fn random_euler_samples() -> impl Iterator<Item = (f64, f64, f64)> {
	let mut random: rand::rngs::StdRng = rand::SeedableRng::seed_from_u64(0);
	(0..1_000).into_iter().map(move |_| {
		let euler_rx = rand(&mut random, 0.0, std::f64::consts::TAU);
		let euler_ry = rand(&mut random, 0.0, std::f64::consts::TAU);
		let euler_rz = rand(&mut random, 0.0, std::f64::consts::TAU);
		(euler_rx, euler_ry, euler_rz)
	})
}

#[test]
fn test_random_xyz() {
	for (axis_x, axis_y, axis_z, angle) in random_samples() {
		let rotation = rotation(axis_x, axis_y, axis_z, angle);
		let (rx, ry, rz) = convert_axis_angle_to_euler_xyz(axis_x, axis_y, axis_z, angle);
		assert_vec!((rot_x(rx) * rot_y(ry) * rot_z(rz)).scaled_axis(), rotation.scaled_axis(), 1e-10);
	}
}

#[test]
fn test_random_yxz() {
	for (axis_x, axis_y, axis_z, angle) in random_samples() {
		let rotation = rotation(axis_x, axis_y, axis_z, angle);
		let (ry, rx, rz) = convert_axis_angle_to_euler_yxz(axis_x, axis_y, axis_z, angle);
		assert_vec!((rot_y(ry) * rot_x(rx) * rot_z(rz)).scaled_axis(), rotation.scaled_axis(), 1e-10);
	}
}

#[test]
fn test_random_zxy() {
	for (axis_x, axis_y, axis_z, angle) in random_samples() {
		let rotation = rotation(axis_x, axis_y, axis_z, angle);
		let (rz, rx, ry) = convert_axis_angle_to_euler_zxy(axis_x, axis_y, axis_z, angle);
		assert_vec!((rot_z(rz) * rot_x(rx) * rot_y(ry)).scaled_axis(), rotation.scaled_axis(), 1e-10);
	}
}

#[test]
fn test_random_zyx() {
	for (axis_x, axis_y, axis_z, angle) in random_samples() {
		let rotation = rotation(axis_x, axis_y, axis_z, angle);
		let (rz, ry, rx) = convert_axis_angle_to_euler_zyx(axis_x, axis_y, axis_z, angle);
		assert_vec!((rot_z(rz) * rot_y(ry) * rot_x(rx)).scaled_axis(), rotation.scaled_axis(), 1e-10);
	}
}

#[test]
fn test_random_yzx() {
	for (axis_x, axis_y, axis_z, angle) in random_samples() {
		let rotation = rotation(axis_x, axis_y, axis_z, angle);
		let (ry, rz, rx) = convert_axis_angle_to_euler_yzx(axis_x, axis_y, axis_z, angle);
		assert_vec!((rot_y(ry) * rot_z(rz) * rot_x(rx)).scaled_axis(), rotation.scaled_axis(), 1e-10);
	}
}

#[test]
fn test_random_xzy() {
	for (axis_x, axis_y, axis_z, angle) in random_samples() {
		let rotation = rotation(axis_x, axis_y, axis_z, angle);
		let (rx, rz, ry) = convert_axis_angle_to_euler_xzy(axis_x, axis_y, axis_z, angle);
		assert_vec!((rot_x(rx) * rot_z(rz) * rot_y(ry)).scaled_axis(), rotation.scaled_axis(), 1e-10);
	}
}

#[test]
fn test_random_euler_xyz() {
	for (euler_rx, euler_ry, euler_rz) in random_euler_samples() {
		let (rx, ry, rz, angle) = convert_euler_xyz_to_axis_angle(euler_rx, euler_ry, euler_rz);
		let rotation = rotation(rx, ry, rz, angle);
		assert_vec!((rot_x(euler_rx) * rot_y(euler_ry) * rot_z(euler_rz)).scaled_axis(), rotation.scaled_axis(), 1e-10);
	}
}

#[test]
fn test_random_euler_yxz() {
	for (euler_rx, euler_ry, euler_rz) in random_euler_samples() {
		let (rx, ry, rz, angle) = convert_euler_yxz_to_axis_angle(euler_rx, euler_ry, euler_rz);
		let rotation = rotation(rx, ry, rz, angle);
		assert_vec!((rot_y(euler_ry) * rot_x(euler_rx) * rot_z(euler_rz)).scaled_axis(), rotation.scaled_axis(), 1e-10);
	}
}

#[test]
fn test_random_euler_zxy() {
	for (euler_rx, euler_ry, euler_rz) in random_euler_samples() {
		let (rx, ry, rz, angle) = convert_euler_zxy_to_axis_angle(euler_rx, euler_ry, euler_rz);
		let rotation = rotation(rx, ry, rz, angle);
		assert_vec!((rot_z(euler_rz) * rot_x(euler_rx) * rot_y(euler_ry)).scaled_axis(), rotation.scaled_axis(), 1e-10);
	}
}

#[test]
fn test_random_euler_zyx() {
	for (euler_rx, euler_ry, euler_rz) in random_euler_samples() {
		let (rx, ry, rz, angle) = convert_euler_zyx_to_axis_angle(euler_rx, euler_ry, euler_rz);
		let rotation = rotation(rx, ry, rz, angle);
		assert_vec!((rot_z(euler_rz) * rot_y(euler_ry) * rot_x(euler_rx)).scaled_axis(), rotation.scaled_axis(), 1e-10);
	}
}

#[test]
fn test_random_euler_yzx() {
	for (euler_rx, euler_ry, euler_rz) in random_euler_samples() {
		let (rx, ry, rz, angle) = convert_euler_yzx_to_axis_angle(euler_rx, euler_ry, euler_rz);
		let rotation = rotation(rx, ry, rz, angle);
		assert_vec!((rot_y(euler_ry) * rot_z(euler_rz) * rot_x(euler_rx)).scaled_axis(), rotation.scaled_axis(), 1e-10);
	}
}

#[test]
fn test_random_euler_xzy() {
	for (euler_rx, euler_ry, euler_rz) in random_euler_samples() {
		let (rx, ry, rz, angle) = convert_euler_xzy_to_axis_angle(euler_rx, euler_ry, euler_rz);
		let rotation = rotation(rx, ry, rz, angle);
		assert_vec!((rot_x(euler_rx) * rot_z(euler_rz) * rot_y(euler_ry)).scaled_axis(), rotation.scaled_axis(), 1e-10);
	}
}

