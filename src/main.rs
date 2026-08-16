mod guidance;
mod vec3;

use std::fs::File;
use std::io::Write;

use guidance::{augmented_proportional_navigation, EngagementState};
use vec3::Vec3;

struct SimConfig {
    dt: f64,
    max_time: f64,
    nav_gain: f64,
    impact_dist: f64,
    target_accel: f64,
    weave_freq: f64,
    boost_accel: f64,
    boost_time: f64,
    max_accel: f64,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            dt: 1e-4,
            max_time: 60.0,
            nav_gain: 4.5,
            impact_dist: 5.0,
            target_accel: 3.0,
            weave_freq: 0.5,
            boost_accel: 100.0,
            boost_time: 4.0,
            max_accel: 196.2,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct Kinematics {
    pos: Vec3,
    vel: Vec3,
}

#[derive(Debug, Clone, Copy)]
struct State {
    missile: Kinematics,
    target: Kinematics,
}

impl State {
    fn deriv(&self, cfg: &SimConfig, t: f64) -> State {
        let target_acc =
            Vec3::new(0.0, cfg.target_accel * (cfg.weave_freq * t).sin(), 0.0);

        let engagement = EngagementState {
            missile_pos: self.missile.pos,
            missile_vel: self.missile.vel,
            target_pos: self.target.pos,
            target_vel: self.target.vel,
            target_acc,
        };

        let mut missile_acc = augmented_proportional_navigation(&engagement, cfg.nav_gain);
        if t < cfg.boost_time {
            missile_acc += self.missile.vel.normalized() * cfg.boost_accel;
        }
        if missile_acc.norm() > cfg.max_accel {
            missile_acc = missile_acc.normalized() * cfg.max_accel;
        }

        State {
            missile: Kinematics {
                pos: self.missile.vel,
                vel: missile_acc,
            },
            target: Kinematics {
                pos: self.target.vel,
                vel: target_acc,
            },
        }
    }

    fn rk4_step(&self, cfg: &SimConfig, t: f64) -> State {
        let half_dt = cfg.dt / 2.0;

        let k1 = self.deriv(cfg, t);
        let k2 = self.scale_add(&k1, half_dt).deriv(cfg, t + half_dt);
        let k3 = self.scale_add(&k2, half_dt).deriv(cfg, t + half_dt);
        let k4 = self.scale_add(&k3, cfg.dt).deriv(cfg, t + cfg.dt);

        let mut rk = k1 + k2 * 2.0 + k3 * 2.0 + k4;
        rk = rk * (cfg.dt / 6.0);
        *self + rk
    }

    fn scale_add(&self, k: &State, scalar: f64) -> State {
        *self + *k * scalar
    }

    fn miss_distance(&self) -> f64 {
        (self.target.pos - self.missile.pos).norm()
    }
}

impl std::ops::Add for State {
    type Output = State;

    fn add(self, other: State) -> State {
        State {
            missile: Kinematics {
                pos: self.missile.pos + other.missile.pos,
                vel: self.missile.vel + other.missile.vel,
            },
            target: Kinematics {
                pos: self.target.pos + other.target.pos,
                vel: self.target.vel + other.target.vel,
            },
        }
    }
}

impl std::ops::Mul<f64> for State {
    type Output = State;

    fn mul(self, scalar: f64) -> State {
        State {
            missile: Kinematics {
                pos: self.missile.pos * scalar,
                vel: self.missile.vel * scalar,
            },
            target: Kinematics {
                pos: self.target.pos * scalar,
                vel: self.target.vel * scalar,
            },
        }
    }
}

fn main() {
    let cfg = SimConfig::default();
    let mut state = State {
        missile: Kinematics {
            pos: Vec3::new(0.0, 0.0, 5000.0),
            vel: Vec3::new(268.24, 0.0, -9.08),
        },
        target: Kinematics {
            pos: Vec3::new(5000.0, 0.0, 0.0),
            vel: Vec3::new(32.4, 8.9, 0.0),
        },
    };
    state.missile.vel = (state.target.pos - state.missile.pos).normalized() * 100.0;

    let mut traj: Vec<(f64, State)> = Vec::new();               //vis
    traj.push((0.0, state));                                    //vis

    let mut t = 0.0;
    let mut min_range = f64::MAX;
    let mut min_t = 0.0;
    let mut min_state = state;
    let mut seen_close = false;
    while t < cfg.max_time {
        let r = state.miss_distance();
        if r < min_range {
            min_range = r;
            min_t = t;
            min_state = state;
        }
        if seen_close && r > min_range + 0.5 {
            break;
        }
        if r < cfg.impact_dist {
            seen_close = true;
        }
        state = state.rk4_step(&cfg, t);
        t += cfg.dt;
        traj.push((t, state));                                  //vis
    }

    if seen_close {
        println!("Intercept at t={}s", min_t);
        println!("Closest approach: {}m", min_range);
        println!(
            "Hit position: missile ({:.2}, {:.2}, {:.2})m  target ({:.2}, {:.2}, {:.2})m",
            min_state.missile.pos.x,
            min_state.missile.pos.y,
            min_state.missile.pos.z,
            min_state.target.pos.x,
            min_state.target.pos.y,
            min_state.target.pos.z
        );
        if min_range <= 0.54 {
            println!("[+] DIRECT HIT, less than a car seat miss distance")
        }
        write_trajectory(&traj);                            //vis
        return;
    }

    println!("No intercept within {}s", cfg.max_time);
    println!("Closest approach: {}m", min_range);
    write_trajectory(&traj);                                    //vis
    std::process::exit(1);
}


//For testing/visualization only                //vis
fn write_trajectory(traj: &[(f64, State)]) {
    let mut f = File::create("trajectory.csv").expect("failed to create trajectory.csv");
    writeln!(f, "t,mx,my,mz,tx,ty,tz").expect("failed to write header");
    for (t, s) in traj {
        writeln!(
            f,
            "{},{},{},{},{},{},{}",
            t,
            s.missile.pos.x,
            s.missile.pos.y,
            s.missile.pos.z,
            s.target.pos.x,
            s.target.pos.y,
            s.target.pos.z
        )
        .expect("failed to write trajectory");
    }
}
