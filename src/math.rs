/// Wrapper for f64.atan to make the code more comparable for different programming environments.
pub fn atan(x: f64) -> f64 {
	return x.atan();
}

/// Wrapper for f64.asin to make the code more comparable for different programming environments.
pub fn asin(x: f64) -> f64 {
	return x.asin();
}

/// Wrapper for f64.cos to make the code more comparable for different programming environments.
pub fn cos(x: f64) -> f64 {
	return x.cos();
}

/// Wrapper for f64.sin to make the code more comparable for different programming environments.
pub fn sin(x: f64) -> f64 {
	return x.sin();
}

/// Wrapper for f64.sqrt to make the code more comparable for different programming environments.
pub fn sqrt(x: f64) -> f64 {
	return x.sqrt();
}

/// Computes the four quadrant arctangent of y / x in radians.
///
/// This atan2 implementation exists in case the target programming environment does not provide it.
///
/// Note that the output is returned in radians.
/// Some adjustments need to be made to use degrees.
pub fn atan2(y: f64, x: f64) -> f64 {
	let pi = 3.14159265358979323846;

	if x > 0.0 {
		return atan(y / x);
	} else if x < 0.0 && y >= 0.0 {
		return atan(y / x) + pi;
	} else if x < 0.0 && y < 0.0 {
		return atan(y / x) - pi
	} else if x == 0.0 && y > 0.0 {
		return pi / 2.0;
	} else if x == 0.0 && y < 0.0 {
		return -pi / 2.0;
	} else {
		// Generate a runtime error.
		panic!("Requested atan of 0 / 0.")
	}
}
