#[test]
fn test_convert_axis_angle_to_euler() {
	use approx::assert_relative_eq;

	use crate::{
		convert_axis_angle_to_euler_xyz,
		convert_axis_angle_to_euler_yxz,
		convert_axis_angle_to_euler_zxy,
		convert_axis_angle_to_euler_zyx,
		convert_axis_angle_to_euler_yzx,
		convert_axis_angle_to_euler_xzy
	};

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

	let (euler_rx, euler_ry, euler_rz) = convert_axis_angle_to_euler_yxz(rx, ry, rz, angle);
	assert_relative_eq!(euler_rx, 0.0775457, epsilon = 1e-6);
	assert_relative_eq!(euler_ry, 0.2041556, epsilon = 1e-6);
	assert_relative_eq!(euler_rz, 0.1639563, epsilon = 1e-6);

	let (euler_rx, euler_ry, euler_rz) = convert_axis_angle_to_euler_zxy(rx, ry, rz, angle);
	assert_relative_eq!(euler_rx, 0.1081443, epsilon = 1e-6);
	assert_relative_eq!(euler_ry, 0.1898811, epsilon = 1e-6);
	assert_relative_eq!(euler_rz, 0.1457008, epsilon = 1e-6);

	let (euler_rx, euler_ry, euler_rz) = convert_axis_angle_to_euler_zyx(rx, ry, rz, angle);
	assert_relative_eq!(euler_rx, 0.1101078, epsilon = 1e-6);
	assert_relative_eq!(euler_ry, 0.1887585, epsilon = 1e-6);
	assert_relative_eq!(euler_rz, 0.1664424, epsilon = 1e-6);

	let (euler_rx, euler_ry, euler_rz) = convert_axis_angle_to_euler_yzx(rx, ry, rz, angle);
	assert_relative_eq!(euler_rx, 0.0785954, epsilon = 1e-6);
	assert_relative_eq!(euler_ry, 0.1913399, epsilon = 1e-6);
	assert_relative_eq!(euler_rz, 0.1634591, epsilon = 1e-6);

	let (euler_rx, euler_ry, euler_rz) = convert_axis_angle_to_euler_xzy(rx, ry, rz, angle);
	assert_relative_eq!(euler_rx, 0.1092933, epsilon = 1e-6);
	assert_relative_eq!(euler_ry, 0.2057181, epsilon = 1e-6);
	assert_relative_eq!(euler_rz, 0.1448436, epsilon = 1e-6);
}
