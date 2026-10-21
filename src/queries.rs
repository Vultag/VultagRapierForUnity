
use rapier2d::prelude::*;

use crate::world::RapierWorld;


impl RapierWorld {
    pub fn get_closest_body(
        &self,
        source_handle: u64,
        x: f32,
        y: f32,
        radius: f32,
        memberships: u32,
        filter: u32
    ) -> Option<RigidBodyHandle> {

        let mut center = Vec2::new(x, y);

        let source_body_handle = if source_handle != 0 {
            let index = source_handle as u32;
            let generation = (source_handle >> 32) as u32;

            let handle =
                RigidBodyHandle::from_raw_parts(index, generation);

            let source_body = self.bodies.get(handle)?;

            center = source_body.translation();

            Some(handle)
        } else {
            None
        };

        let shape = Ball::new(radius);

    let query_groups = InteractionGroups {
                memberships: Group::from_bits_truncate(memberships),
                filter: Group::from_bits_truncate(filter),
                test_mode: InteractionTestMode::And,
            };

        let query_filter = QueryFilter::default()
            .groups(query_groups);

        let query_pipeline = self.broad_phase.as_query_pipeline(
            self.narrow_phase.query_dispatcher(),
            &self.bodies,
            &self.colliders,
            query_filter,
        );

        let shape_pos = Pose::translation(center.x, center.y);

        let mut closest_body = None;
        let mut closest_distance = radius;

        for (_, collider) in query_pipeline.intersect_shape(
            shape_pos,
            &shape,
        ) {
            // get the body from collider
            let Some(body_handle) = collider.parent() else {
                continue;
            };

            // look for the source body to ignore
            if Some(body_handle) == source_body_handle {
                continue;
            }

            let projection = collider.shape().project_point(
                collider.position(),
                center,
                true,
            );

            let distance = center.distance(projection.point);

            if distance < closest_distance {
                closest_distance = distance;
                closest_body = Some(body_handle);
            }
        }

        closest_body
    }
}


#[unsafe(no_mangle)]
pub extern "C" fn world_get_closest_body(
    world: *mut RapierWorld,
    source_handle: u64,
    x: f32,
    y: f32,
    radius: f32,
    memberships: u32,
    filter: u32
) -> u64 {

    let world = unsafe {&*world };

    let Some(handle) = world.get_closest_body(
        source_handle,
        x,
        y,
        radius,
        memberships,
        filter
    ) else {
        return 0;
    };

    let (index, generation) = handle.into_raw_parts();

    (index as u64) | ((generation as u64) << 32)
}