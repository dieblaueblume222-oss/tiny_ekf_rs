# tiny_ekf_rs

**A safe, `nalgebra`-powered Extended Kalman Filter — Rust translation of
[Simon D. Levy's TinyEKF](https://github.com/simondlevy/TinyEKF).**

[![Rust](https://img.shields.io/badge/rust-1.95%2B-orange.svg)](https://www.rust-lang.org)
[![License MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

---

## 1. Overview

TinyEKF is a classic lightweight C implementation of the Extended Kalman Filter,
designed for microcontrollers. `tiny_ekf_rs` reimagines it in safe, modern Rust:

| C (TinyEKF) | Rust (tiny_ekf_rs) |
| --- | --- |
| Raw `double[]` flat arrays | `nalgebra::SVector` / `SMatrix` — dimension-checked at compile time |
| Hand-rolled `mat_mult`/`mat_inv` | `nalgebra` — optimised, audited, no `unsafe` |
| `#define EKF_N 6` | `const N: usize` generic — pick any size |
| Cholesky failure silently ignored | `Result<()>` — errors propagated explicitly |
| Function-pointer callbacks | `SensorModel` trait — the compiler verifies all 4 methods exist |
| Stack scratch buffers | Zero scratch — temporaries are stack-allocated by nalgebra |

---

## 2. The EKF Algorithm

The filter performs two alternating steps:

### Predict

```
x  ←  f(x)                  state propagation
P  ←  F · P · Fᵀ + Q        covariance propagation
```

where **F** = ∂f/∂x is the Jacobian of the motion model evaluated at the
predicted state.

### Update

```
y  =  z − h(x)              innovation
S  =  H · P · Hᵀ + R        innovation covariance
K  =  P · Hᵀ · S⁻¹          Kalman gain
x  =  x + K · y             state correction
P  =  (I − K · H) · P       covariance correction
```

where **H** = ∂h/∂x is the Jacobian of the sensor model.

---

## 3. Why Rust? Memory Safety & API Advantages

### 3.1 No buffer overruns

The C version uses `#define EKF_N 6` and flat arrays. If a user accidentally
sets `ekf.n = 7`, every matrix operation silently reads garbage. In Rust,
`EKF<const N: usize, const M: usize>` enforces dimensions at the type level:
the compiler **rejects** any size mismatch before the code runs.

### 3.2 Explicit error handling

When the innovation covariance **S** becomes singular (e.g. unobservable
system), the C code returns silently and leaves the state corrupted. The Rust
`update()` method returns `Result<()>`, forcing the caller to handle the error.

### 3.3 Trait-based model contract

C uses four nullable function pointers (`f`, `F_jac`, `h`, `H_jac`) — forgetting
to set one causes a segfault. Rust's `SensorModel` trait requires all four
methods to be implemented; the compiler verifies this at every call site.

### 3.4 No `unsafe`

The entire library is safe Rust. All matrix inverses, transposes, and
multiplications go through `nalgebra`, which is thoroughly audited and
used across the robotics ecosystem.

---

## 4. Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
tiny_ekf_rs = { path = "path/to/tiny_ekf_rs" }
nalgebra = "0.33"
```

Or install the nalgebra family from crates.io once published.

---

## 5. Usage — UAV Flight Controller Example

```rust
use tiny_ekf_rs::{EKF, SensorModel};
use nalgebra::{SVector, SMatrix};

// ── 4D state: [px, py, vx, vy] ──
// ── 2D measurement: GPS [px, py] ──

struct ConstantVelocity { dt: f64 }

impl SensorModel<4, 2> for ConstantVelocity {
    fn f(&self, x: &SVector<f64, 4>) -> SVector<f64, 4> {
        SVector::from_vec(vec![
            x[0] + self.dt * x[2],
            x[1] + self.dt * x[3],
            x[2],
            x[3],
        ])
    }

    fn F_jacobian(&self, _x: &SVector<f64, 4>) -> SMatrix<f64, 4, 4> {
        let dt = self.dt;
        SMatrix::<f64, 4, 4>::from_fn(|r, c| match (r, c) {
            (0, 0) | (1, 1) | (2, 2) | (3, 3) => 1.0,
            (0, 2) | (1, 3) => dt,
            _ => 0.0,
        })
    }

    fn h(&self, x: &SVector<f64, 4>) -> SVector<f64, 2> {
        SVector::from_vec(vec![x[0], x[1]])
    }

    fn H_jacobian(&self, _x: &SVector<f64, 4>) -> SMatrix<f64, 2, 4> {
        SMatrix::<f64, 2, 4>::from_fn(|r, c| match (r, c) {
            (0, 0) | (1, 1) => 1.0,
            _ => 0.0,
        })
    }
}

fn main() -> anyhow::Result<()> {
    let model = ConstantVelocity { dt: 0.02 }; // 50 Hz IMU

    let mut ekf = EKF::<4, 2>::new(
        SVector::from_vec(vec![0.0, 0.0, 0.0, 0.0]),       // x₀
        SMatrix::from_diagonal(&SVector::from_vec(vec![     // P₀
            100.0, 100.0, 100.0, 100.0,
        ])),
        SMatrix::from_diagonal(&SVector::from_vec(vec![     // Q
            0.01, 0.01, 0.5, 0.5,
        ])),
        SMatrix::from_diagonal(&SVector::from_vec(vec![     // R (GPS)
            5.0_f64.powi(2), 5.0_f64.powi(2),
        ])),
    );

    // Main loop — called at each GPS update
    loop {
        let gps_reading = read_gps()?; // your sensor driver
        let z = SVector::from_vec(vec![gps_reading.lat, gps_reading.lon]);
        ekf.step(&model, &z)?;

        // Access filtered state
        let (px, py, vx, vy) = (ekf.x[0], ekf.x[1], ekf.x[2], ekf.x[3]);
        send_to_flight_controller(px, py, vx, vy);
    }
}
```

---

## 6. Running Tests

```bash
cargo test
```

The test suite includes:

| Test | Description |
| --- | --- |
| `test_uav_2d_tracking` | 100-step 2D constant-velocity simulation with noisy GPS. Verifies position error < 5 m and velocity convergence < 1 m/s. |
| `test_singular_update_returns_error` | Ensures a degenerate measurement Jacobian produces `Err(_)`, not silent corruption. |

---

## 7. Project Structure

```
tiny_ekf_rs/
├── Cargo.toml          # crate manifest (nalgebra + anyhow)
├── src/
│   └── lib.rs          # EKF struct, SensorModel trait, predict/update, tests
└── README.md           # this file
```

---

## 8. Original C Project Reference

The [original TinyEKF](https://github.com/simondlevy/TinyEKF) by Simon D. Levy
is a header-only C/C++ library targeting Arduino, STM32, and Teensy. It uses
static memory allocation (no `malloc`) and hand-rolled Cholesky decomposition.
Our Rust port preserves the mathematical core while adding the safety guarantees
described above.

---

## License

MIT — see the original TinyEKF repository for attribution.
