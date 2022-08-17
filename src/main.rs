//! Conversions are derived from https://github.com/mrdoob/three.js/blob/4f0ff2cd014fc02b1213aaa0df842b09708eee39/src/math/Euler.js#L103.

#![allow(clippy::needless_return)]
#![allow(clippy::excessive_precision)]


mod math;

#[cfg(test)]
mod tests;

use math::{sqrt, sin, cos, atan2, asin, clamp};

/// Convert from AxisAngle to Euler XYZ.
///
/// All rotations are expected and returned as radians.
///
/// # Arguments
///
/// * `rx` - The X component of the axis angle vector.
/// * `ry` - The Y component of the axis angle vector.
/// * `rz` - The Z component of the axis angle vector.
/// * `angle` - The rotation around the axis angle vector to apply in radians.
///
/// Known robots that use this format:
/// - Staubli (NOTE: this example assumes radians values, but this robot uses degrees)
fn convert_axis_angle_to_euler_xyz(rx: f64, ry: f64, rz: f64, angle: f64) -> (f64, f64, f64) {
	let axis_norm = sqrt(rx * rx + ry * ry + rz * rz);

	// Compute the quaternion
	let sin_angle = sin(0.5 * angle);
	let qw = cos(0.5 * angle);
	let qx = rx / axis_norm * sin_angle;
	let qy = ry / axis_norm * sin_angle;
	let qz = rz / axis_norm * sin_angle;

	let euler_x = atan2(2.0 * (qw * qx - qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
	let euler_y = asin(clamp(2.0 * (qx * qz + qw * qy), -1.0, 1.0));
	let euler_z = atan2(2.0 * (qw * qz - qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

	return (euler_x, euler_y, euler_z);
}

/// Convert from Euler XYZ to AxisAngle.
///
/// All rotations are expected and returned as radians.
///
/// # Arguments
///
/// * `euler_rx` - The rotation around the X axis.
/// * `euler_ry` - The rotation around the Y axis.
/// * `euler_rz` - The rotation around the Z axis.
///
/// Known robots that use this format:
/// - Staubli (NOTE: this example assumes radians values, but this robot uses degrees)
fn convert_euler_xyz_to_axis_angle(euler_rx: f64, euler_ry: f64, euler_rz: f64) -> (f64, f64, f64, f64) {
	let cx = cos(euler_rx / 2.0);
	let cy = cos(euler_ry / 2.0);
	let cz = cos(euler_rz / 2.0);
	let sx = sin(euler_rx / 2.0);
	let sy = sin(euler_ry / 2.0);
	let sz = sin(euler_rz / 2.0);

	// Compute the quaternion
	let qw = -(sx * sy * sz) + (cx * cy * cz);
	let qx =  (sx * cy * cz) + (sy * sz * cx);
	let qy = -(sx * sz * cy) + (sy * cx * cz);
	let qz =  (sx * sy * cz) + (sz * cx * cy);

	// Compute axis angle
	let rx = qx / sqrt(qx * qx + qy * qy + qz * qz);
	let ry = qy / sqrt(qx * qx + qy * qy + qz * qz);
	let rz = qz / sqrt(qx * qx + qy * qy + qz * qz);
	let angle = 2.0 * atan2(sqrt(qx * qx + qy * qy + qz * qz), qw);

	return (rx, ry, rz, angle);
}


/// Convert from AxisAngle to Euler YXZ.
///
/// All rotations are expected and returned as radians.
///
/// # Arguments
///
/// * `rx` - The X component of the axis angle vector.
/// * `ry` - The Y component of the axis angle vector.
/// * `rz` - The Z component of the axis angle vector.
/// * `angle` - The rotation around the axis angle vector to apply in radians.
///
/// Known robots that use this format:
fn convert_axis_angle_to_euler_yxz(rx: f64, ry: f64, rz: f64, angle: f64) -> (f64, f64, f64) {
	let axis_norm = sqrt(rx * rx + ry * ry + rz * rz);

	// Compute the quaternion
	let sin_angle = sin(0.5 * angle);
	let qw = cos(0.5 * angle);
	let qx = rx / axis_norm * sin_angle;
	let qy = ry / axis_norm * sin_angle;
	let qz = rz / axis_norm * sin_angle;

	let euler_x = asin(clamp(2.0 * (qw * qx - qy * qz), -1.0, 1.0));
	let euler_y = atan2(2.0 * (qx * qz + qw * qy), 1.0 - 2.0 * (qx * qx + qy * qy));
	let euler_z = atan2(2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qx * qx + qz * qz));

	return (euler_y, euler_x, euler_z);
}

/// Convert from Euler YXZ to AxisAngle.
///
/// All rotations are expected and returned as radians.
///
/// # Arguments
///
/// * `euler_rx` - The rotation around the X axis.
/// * `euler_ry` - The rotation around the Y axis.
/// * `euler_rz` - The rotation around the Z axis.
///
/// Known robots that use this format:
fn convert_euler_yxz_to_axis_angle(euler_rx: f64, euler_ry: f64, euler_rz: f64) -> (f64, f64, f64, f64) {
	let cx = cos(euler_rx / 2.0);
	let cy = cos(euler_ry / 2.0);
	let cz = cos(euler_rz / 2.0);
	let sx = sin(euler_rx / 2.0);
	let sy = sin(euler_ry / 2.0);
	let sz = sin(euler_rz / 2.0);

	// Compute the quaternion
	let qw =  (sy * sx * sz) + (cy * cx * cz);
	let qx =  (sy * sz * cx) + (sx * cy * cz);
	let qy =  (sy * cx * cz) - (sx * sz * cy);
	let qz = -(sy * sx * cz) + (sz * cy * cx);

	// Compute axis angle
	let rx = qx / sqrt(qx * qx + qy * qy + qz * qz);
	let ry = qy / sqrt(qx * qx + qy * qy + qz * qz);
	let rz = qz / sqrt(qx * qx + qy * qy + qz * qz);
	let angle = 2.0 * atan2(sqrt(qx * qx + qy * qy + qz * qz), qw);

	return (rx, ry, rz, angle);
}


/// Convert from AxisAngle to Euler ZXY.
///
/// All rotations are expected and returned as radians.
///
/// # Arguments
///
/// * `rx` - The X component of the axis angle vector.
/// * `ry` - The Y component of the axis angle vector.
/// * `rz` - The Z component of the axis angle vector.
/// * `angle` - The rotation around the axis angle vector to apply in radians.
///
/// Known robots that use this format:
fn convert_axis_angle_to_euler_zxy(rx: f64, ry: f64, rz: f64, angle: f64) -> (f64, f64, f64) {
	let axis_norm = sqrt(rx * rx + ry * ry + rz * rz);

	// Compute the quaternion
	let sin_angle = sin(0.5 * angle);
	let qw = cos(0.5 * angle);
	let qx = rx / axis_norm * sin_angle;
	let qy = ry / axis_norm * sin_angle;
	let qz = rz / axis_norm * sin_angle;

	let euler_x = asin(clamp(2.0 * (qy * qz + qw * qx), -1.0, 1.0));
	let euler_y = atan2(2.0 * (qw * qy - qx * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
	let euler_z = atan2(2.0 * (qw * qz - qx * qy), 1.0 - 2.0 * (qx * qx + qz * qz));

	return (euler_z, euler_x, euler_y);
}

/// Convert from Euler YXZ to AxisAngle.
///
/// All rotations are expected and returned as radians.
///
/// # Arguments
///
/// * `euler_rx` - The rotation around the X axis.
/// * `euler_ry` - The rotation around the Y axis.
/// * `euler_rz` - The rotation around the Z axis.
///
/// Known robots that use this format:
fn convert_euler_zxy_to_axis_angle(euler_rx: f64, euler_ry: f64, euler_rz: f64) -> (f64, f64, f64, f64) {
	let cx = cos(euler_rx / 2.0);
	let cy = cos(euler_ry / 2.0);
	let cz = cos(euler_rz / 2.0);
	let sx = sin(euler_rx / 2.0);
	let sy = sin(euler_ry / 2.0);
	let sz = sin(euler_rz / 2.0);

	// Compute the quaternion
	let qw = -(sz * sx * sy) + (cz * cx * cy);
	let qx = -(sz * sy * cx) + (sx * cz * cy);
	let qy =  (sz * sx * cy) + (sy * cz * cx);
	let qz =  (sz * cx * cy) + (sx * sy * cz);

	// Compute axis angle
	let rx = qx / sqrt(qx * qx + qy * qy + qz * qz);
	let ry = qy / sqrt(qx * qx + qy * qy + qz * qz);
	let rz = qz / sqrt(qx * qx + qy * qy + qz * qz);
	let angle = 2.0 * atan2(sqrt(qx * qx + qy * qy + qz * qz), qw);

	return (rx, ry, rz, angle);
}

/// Convert from AxisAngle to Euler ZYX.
///
/// All rotations are expected and returned as radians.
///
/// # Arguments
///
/// * `rx` - The X component of the axis angle vector.
/// * `ry` - The Y component of the axis angle vector.
/// * `rz` - The Z component of the axis angle vector.
/// * `angle` - The rotation around the axis angle vector to apply in radians.
///
/// Known robots that use this format:
/// - Mitsubishi Industrial Robot CR750/CR751 Series
/// - Kawasaki (NOTE: this example assumes radians values, but this robot uses degrees)
/// - Kuka (NOTE: this example assumes radians values, but this robot uses degrees)
fn convert_axis_angle_to_euler_zyx(rx: f64, ry: f64, rz: f64, angle: f64) -> (f64, f64, f64) {
	let axis_norm = sqrt(rx * rx + ry * ry + rz * rz);

	// Compute the quaternion
	let sin_angle = sin(0.5 * angle);
	let qw = cos(0.5 * angle);
	let qx = rx / axis_norm * sin_angle;
	let qy = ry / axis_norm * sin_angle;
	let qz = rz / axis_norm * sin_angle;

	let euler_x = atan2(2.0 * (qy * qz + qw * qx), 1.0 - 2.0 * (qx * qx + qy * qy));
	let euler_y = asin(clamp(2.0 * (qw * qy - qx * qz), -1.0, 1.0));
	let euler_z = atan2(2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qy * qy + qz * qz));

	return (euler_z, euler_y, euler_x);
}

/// Convert from Euler ZYX to AxisAngle.
///
/// All rotations are expected and returned as radians.
///
/// # Arguments
///
/// * `euler_rx` - The rotation around the X axis.
/// * `euler_ry` - The rotation around the Y axis.
/// * `euler_rz` - The rotation around the Z axis.
///
/// Known robots that use this format:
/// - Mitsubishi Industrial Robot CR750/CR751 Series
/// - Kawasaki (NOTE: this example assumes radians values, but this robot uses degrees)
/// - Kuka (NOTE: this example assumes radians values, but this robot uses degrees)
fn convert_euler_zyx_to_axis_angle(euler_rx: f64, euler_ry: f64, euler_rz: f64) -> (f64, f64, f64, f64) {
	let cx = cos(euler_rx / 2.0);
	let cy = cos(euler_ry / 2.0);
	let cz = cos(euler_rz / 2.0);
	let sx = sin(euler_rx / 2.0);
	let sy = sin(euler_ry / 2.0);
	let sz = sin(euler_rz / 2.0);

	// Compute the quaternion
	let qw =  (sz * sy * sx) + (cz * cy * cx);
	let qx = -(sz * sy * cx) + (sx * cz * cy);
	let qy =  (sz * sx * cy) + (sy * cz * cx);
	let qz =  (sz * cy * cx) - (sy * sx * cz);

	// Compute axis angle
	let rx = qx / sqrt(qx * qx + qy * qy + qz * qz);
	let ry = qy / sqrt(qx * qx + qy * qy + qz * qz);
	let rz = qz / sqrt(qx * qx + qy * qy + qz * qz);
	let angle = 2.0 * atan2(sqrt(qx * qx + qy * qy + qz * qz), qw);

	return (rx, ry, rz, angle);
}

/// Convert from AxisAngle to Euler YZX.
///
/// All rotations are expected and returned as radians.
///
/// # Arguments
///
/// * `rx` - The X component of the axis angle vector.
/// * `ry` - The Y component of the axis angle vector.
/// * `rz` - The Z component of the axis angle vector.
/// * `angle` - The rotation around the axis angle vector to apply in radians.
///
/// Known robots that use this format:
fn convert_axis_angle_to_euler_yzx(rx: f64, ry: f64, rz: f64, angle: f64) -> (f64, f64, f64) {
	let axis_norm = sqrt(rx * rx + ry * ry + rz * rz);

	// Compute the quaternion
	let sin_angle = sin(0.5 * angle);
	let qw = cos(0.5 * angle);
	let qx = rx / axis_norm * sin_angle;
	let qy = ry / axis_norm * sin_angle;
	let qz = rz / axis_norm * sin_angle;

	let euler_x = atan2(2.0 * (qw * qx - qy * qz), 1.0 - 2.0 * (qx * qx + qz * qz));
	let euler_y = atan2(2.0 * (qw * qy - qx * qz), 1.0 - 2.0 * (qy * qy + qz * qz));
	let euler_z = asin(clamp(2.0 * (qx * qy + qw * qz), -1.0, 1.0));

	return (euler_y, euler_z, euler_x);
}

/// Convert from Euler YZX to AxisAngle.
///
/// All rotations are expected and returned as radians.
///
/// # Arguments
///
/// * `euler_rx` - The rotation around the X axis.
/// * `euler_ry` - The rotation around the Y axis.
/// * `euler_rz` - The rotation around the Z axis.
///
/// Known robots that use this format:
fn convert_euler_yzx_to_axis_angle(euler_rx: f64, euler_ry: f64, euler_rz: f64) -> (f64, f64, f64, f64) {
	let cx = cos(euler_rx / 2.0);
	let cy = cos(euler_ry / 2.0);
	let cz = cos(euler_rz / 2.0);
	let sx = sin(euler_rx / 2.0);
	let sy = sin(euler_ry / 2.0);
	let sz = sin(euler_rz / 2.0);

	// Compute the quaternion
	let qw = -(sy * sz * sx) + (cy * cz * cx);
	let qx =  (sy * sz * cx) + (sx * cy * cz);
	let qy =  (sy * cz * cx) + (sz * sx * cy);
	let qz = -(sy * sx * cz) + (sz * cy * cx);

	// Compute axis angle
	let rx = qx / sqrt(qx * qx + qy * qy + qz * qz);
	let ry = qy / sqrt(qx * qx + qy * qy + qz * qz);
	let rz = qz / sqrt(qx * qx + qy * qy + qz * qz);
	let angle = 2.0 * atan2(sqrt(qx * qx + qy * qy + qz * qz), qw);

	return (rx, ry, rz, angle);
}


/// Convert from AxisAngle to Euler XZY.
///
/// All rotations are expected and returned as radians.
///
/// # Arguments
///
/// * `rx` - The X component of the axis angle vector.
/// * `ry` - The Y component of the axis angle vector.
/// * `rz` - The Z component of the axis angle vector.
/// * `angle` - The rotation around the axis angle vector to apply in radians.
///
/// Known robots that use this format:
fn convert_axis_angle_to_euler_xzy(rx: f64, ry: f64, rz: f64, angle: f64) -> (f64, f64, f64) {
	let axis_norm = sqrt(rx * rx + ry * ry + rz * rz);

	// Compute the quaternion
	let sin_angle = sin(0.5 * angle);
	let qw = cos(0.5 * angle);
	let qx = rx / axis_norm * sin_angle;
	let qy = ry / axis_norm * sin_angle;
	let qz = rz / axis_norm * sin_angle;

	let euler_x = atan2(2.0 * (qy * qz + qw * qx), 1.0 - 2.0 * (qx * qx + qz * qz));
	let euler_y = atan2(2.0 * (qx * qz + qw * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
	let euler_z = asin(clamp(2.0 * (qw * qz - qx * qy), -1.0, 1.0));

	return (euler_x, euler_z, euler_y);
}

/// Convert from Euler XZY to AxisAngle.
///
/// All rotations are expected and returned as radians.
///
/// # Arguments
///
/// * `euler_rx` - The rotation around the X axis.
/// * `euler_ry` - The rotation around the Y axis.
/// * `euler_rz` - The rotation around the Z axis.
///
/// Known robots that use this format:
fn convert_euler_xzy_to_axis_angle(euler_rx: f64, euler_ry: f64, euler_rz: f64) -> (f64, f64, f64, f64) {
	let cx = cos(euler_rx / 2.0);
	let cy = cos(euler_ry / 2.0);
	let cz = cos(euler_rz / 2.0);
	let sx = sin(euler_rx / 2.0);
	let sy = sin(euler_ry / 2.0);
	let sz = sin(euler_rz / 2.0);

	// Compute the quaternion
	let qw =  (sx * sz * sy) + (cx * cz * cy);
	let qx =  (sx * cz * cy) - (sz * sy * cx);
	let qy = -(sx * sz * cy) + (sy * cx * cz);
	let qz =  (sx * sy * cz) + (sz * cx * cy);

	// Compute axis angle
	let rx = qx / sqrt(qx * qx + qy * qy + qz * qz);
	let ry = qy / sqrt(qx * qx + qy * qy + qz * qz);
	let rz = qz / sqrt(qx * qx + qy * qy + qz * qz);
	let angle = 2.0 * atan2(sqrt(qx * qx + qy * qy + qz * qz), qw);

	return (rx, ry, rz, angle);
}


fn main() {
	let rx = 1.0;
	let ry = 0.0;
	let rz = 0.0;
	let angle = 3.14159265358979323846;

	println!("AxisAngle (rad): (rx: {}, ry: {}, rz: {}, angle: {})", rx, ry, rz, angle);
	println!("---");

	let (euler_rx, euler_ry, euler_rz) = convert_axis_angle_to_euler_xyz(rx, ry, rz, angle);
	let order = "xyz";
	println!("Euler (rad) {}: (rx: {}, ry: {}, rz: {})", order.to_uppercase(), euler_rx, euler_ry, euler_rz);
	println!("Euler (deg) {}: (rx: {}, ry: {}, rz: {})", order.to_uppercase(), euler_rx.to_degrees(), euler_ry.to_degrees(), euler_rz.to_degrees());
	println!();

	let (converted_rx,converted_ry,converted_rz, converted_angle) = convert_euler_xyz_to_axis_angle(euler_rx, euler_ry, euler_rz);
	println!("AxisAngle (rad): (rx: {}, ry: {}, rz: {}, angle: {})", converted_rx, converted_ry, converted_rz, converted_angle);
	println!("---");

	let (euler_ry, euler_rx, euler_rz) = convert_axis_angle_to_euler_yxz(rx, ry, rz, angle);
	let order = "yxz";
	println!("Euler (rad) {}: (ry: {}, rx: {}, rz: {})", order.to_uppercase(), euler_ry, euler_rx, euler_rz);
	println!("Euler (deg) {}: (ry: {}, rx: {}, rz: {})", order.to_uppercase(), euler_ry.to_degrees(), euler_rx.to_degrees(), euler_rz.to_degrees());
	println!();

	let (converted_rx,converted_ry,converted_rz, converted_angle) = convert_euler_yxz_to_axis_angle(euler_rx, euler_ry, euler_rz);
	println!("AxisAngle (rad): (rx: {}, ry: {}, rz: {}, angle: {})", converted_rx, converted_ry, converted_rz, converted_angle);
	println!("---");

	let (euler_rz, euler_rx, euler_ry) = convert_axis_angle_to_euler_zxy(rx, ry, rz, angle);
	let order = "zxy";
	println!("Euler (rad) {}: (rz: {}, rx: {}, ry: {})", order.to_uppercase(), euler_rz, euler_rx, euler_ry);
	println!("Euler (deg) {}: (rz: {}, rx: {}, ry: {})", order.to_uppercase(), euler_rz.to_degrees(), euler_rx.to_degrees(), euler_ry.to_degrees());
	println!();

	let (converted_rx,converted_ry,converted_rz, converted_angle) = convert_euler_zxy_to_axis_angle(euler_rx, euler_ry, euler_rz);
	println!("AxisAngle (rad): (rx: {}, ry: {}, rz: {}, angle: {})", converted_rx, converted_ry, converted_rz, converted_angle);
	println!("---");

	let (euler_rz, euler_ry, euler_rx) = convert_axis_angle_to_euler_zyx(rx, ry, rz, angle);
	let order = "zyx";
	println!("Euler (rad) {}: (rz: {}, ry: {}, rx: {})", order.to_uppercase(), euler_rz, euler_ry, euler_rx);
	println!("Euler (deg) {}: (rz: {}, ry: {}, rx: {})", order.to_uppercase(), euler_rz.to_degrees(), euler_ry.to_degrees(), euler_rx.to_degrees());
	println!();

	let (converted_rx,converted_ry,converted_rz, converted_angle) = convert_euler_zyx_to_axis_angle(euler_rx, euler_ry, euler_rz);
	println!("AxisAngle (rad): (rx: {}, ry: {}, rz: {}, angle: {})", converted_rx, converted_ry, converted_rz, converted_angle);
	println!("---");

	let (euler_ry, euler_rz, euler_rx) = convert_axis_angle_to_euler_yzx(rx, ry, rz, angle);
	let order = "yzx";
	println!("Euler (rad) {}: (ry: {}, rz: {}, rx: {})", order.to_uppercase(), euler_ry, euler_rz, euler_rx);
	println!("Euler (deg) {}: (ry: {}, rz: {}, rx: {})", order.to_uppercase(), euler_ry.to_degrees(), euler_rz.to_degrees(), euler_rx.to_degrees());
	println!();

	let (converted_rx,converted_ry,converted_rz, converted_angle) = convert_euler_yzx_to_axis_angle(euler_rz, euler_rx, euler_ry);
	println!("AxisAngle (rad): (rx: {}, ry: {}, rz: {}, angle: {})", converted_rx, converted_ry, converted_rz, converted_angle);
	println!("---");

	let (euler_rx, euler_rz, euler_ry) = convert_axis_angle_to_euler_xzy(rx, ry, rz, angle);
	let order = "xzy";
	println!("Euler (rad) {}: (rx: {}, rz: {}, ry: {})", order.to_uppercase(), euler_rx, euler_rz, euler_ry);
	println!("Euler (deg) {}: (rx: {}, rz: {}, ry: {})", order.to_uppercase(), euler_rx.to_degrees(), euler_rz.to_degrees(), euler_ry.to_degrees());
	println!();

	let (converted_rx,converted_ry,converted_rz, converted_angle) = convert_euler_xzy_to_axis_angle(euler_rz, euler_ry, euler_rx);
	println!("AxisAngle (rad): (rx: {}, ry: {}, rz: {}, angle: {})", converted_rx, converted_ry, converted_rz, converted_angle);
	println!("---");


	let euler_rx: f64 = 15.0;
	let euler_ry: f64 = 100.0;
	let euler_rz: f64 = 67.0;

	let (rx,ry,rz,angle) = convert_euler_xyz_to_axis_angle(euler_rx.to_radians(), euler_ry.to_radians(), euler_rz.to_radians());
	let order = "xyz";
	println!("Euler (rad) {}: (rx: {}, ry: {}, rz: {})", order.to_uppercase(), euler_rx, euler_ry, euler_rz);
	println!("AxisAngle (rad): (rx: {}, ry: {}, rz: {}, angle: {})", rx, ry, rz, angle);
	println!("---");

	let (rx,ry,rz,angle) = convert_euler_yxz_to_axis_angle(euler_rx.to_radians(), euler_ry.to_radians(), euler_rz.to_radians());
	let order = "yxz";
	println!("Euler (rad) {}: (ry: {}, rx: {}, rz: {})", order.to_uppercase(), euler_ry, euler_rx, euler_rz);
	println!("AxisAngle (rad): (rx: {}, ry: {}, rz: {}, angle: {})", rx, ry, rz, angle);
	println!("---");

	let (rx,ry,rz,angle) = convert_euler_zxy_to_axis_angle(euler_rx.to_radians(), euler_ry.to_radians(), euler_rz.to_radians());
	let order = "zxy";
	println!("Euler (rad) {}: (rz: {}, rx: {}, ry: {})", order.to_uppercase(), euler_rz, euler_rx, euler_ry);
	println!("AxisAngle (rad): (rx: {}, ry: {}, rz: {}, angle: {})", rx, ry, rz, angle);
	println!("---");

	let (rx,ry,rz,angle) = convert_euler_zyx_to_axis_angle(euler_rx.to_radians(), euler_ry.to_radians(), euler_rz.to_radians());
	let order = "zyx";
	println!("Euler (rad) {}: (rz: {}, ry: {}, rx: {})", order.to_uppercase(), euler_rz, euler_ry, euler_rx);
	println!("AxisAngle (rad): (rx: {}, ry: {}, rz: {}, angle: {})", rx, ry, rz, angle);
	println!("---");

	let (rx,ry,rz,angle) = convert_euler_yzx_to_axis_angle(euler_rx.to_radians(), euler_ry.to_radians(), euler_rz.to_radians());
	let order = "yzx";
	println!("Euler (rad) {}: (ry: {}, rz: {}, rx: {})", order.to_uppercase(), euler_ry, euler_rz, euler_rx);
	println!("AxisAngle (rad): (rx: {}, ry: {}, rz: {}, angle: {})", rx, ry, rz, angle);
	println!("---");

	let (rx,ry,rz,angle) = convert_euler_xzy_to_axis_angle(euler_rx.to_radians(), euler_ry.to_radians(), euler_rz.to_radians());
	let order = "xzy";
	println!("Euler (rad) {}: (rx: {}, rz: {}, ry: {})", order.to_uppercase(), euler_rx, euler_rz, euler_ry);
	println!("AxisAngle (rad): (rx: {}, ry: {}, rz: {}, angle: {})", rx, ry, rz, angle);
}
