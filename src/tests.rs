use approx::assert_relative_eq;
use nalgebra as na;

use crate::{
	convert_axis_angle_to_euler_xyz,
	convert_axis_angle_to_euler_yxz,
	convert_axis_angle_to_euler_zxy,
	convert_axis_angle_to_euler_zyx,
	convert_axis_angle_to_euler_yzx,
	convert_axis_angle_to_euler_xzy
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
