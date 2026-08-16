use crate::vec3::Vec3;

pub struct EngagementState {
    pub missile_pos: Vec3,
    pub missile_vel: Vec3,
    pub target_pos: Vec3,
    pub target_vel: Vec3,
    pub target_acc: Vec3,
}

pub fn proportional_navigation(state: &EngagementState, nav_gain: f64) -> Vec3 {
    let los = state.target_pos - state.missile_pos;
    let rel_vel = state.target_vel - state.missile_vel;
    let range = los.norm();
    let missile_speed = state.missile_vel.norm();
    if missile_speed < 1e-6 {
        return Vec3::new(0.0, 0.0, 0.0);
    }

    let los_rate = los.cross(rel_vel) / (range * range);
    let unit_missile = state.missile_vel / missile_speed;
    let unit_los = los / range;
    //let closing_speed = -rel_vel.dot(unit_los);
    let closing_speed = (-rel_vel.dot(unit_los)).max(0.0);
 
    //let closing_speed = rel_vel.norm();
    los_rate.cross(unit_missile) * (nav_gain * closing_speed)
}

pub fn augmented_proportional_navigation(state: &EngagementState, nav_gain: f64) -> Vec3 {
    let los = state.target_pos - state.missile_pos;
    let rel_vel = state.target_vel - state.missile_vel;
    let range = los.norm();
    let missile_speed = state.missile_vel.norm();
    if missile_speed < 1e-6 || range < 1e-9 {
        return Vec3::new(0.0, 0.0, 0.0);
    }

    let los_rate = los.cross(rel_vel) / (range * range);
    let unit_missile = state.missile_vel / missile_speed;
    let unit_los = los / range;
    let closing_speed = (-rel_vel.dot(unit_los)).max(0.0);

    let mut acc = los_rate.cross(unit_missile) * (nav_gain * closing_speed);
    let at_perp = state.target_acc - unit_los * state.target_acc.dot(unit_los);
    acc += at_perp * (nav_gain / 2.0);
    acc
}
