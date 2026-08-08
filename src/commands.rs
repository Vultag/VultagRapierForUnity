use crate::world::RapierWorld;
use rapier2d::prelude::*;

const COLLIDER_FLAG: u64 = 1 << 63;

#[repr(C)]
pub struct TransformUpdate {
    pub entity_id: u64,
    pub position: [f32; 2],
    pub rotation: f32,
}
#[repr(C)]
pub struct ColTrigEventFFI {
    pub entity_a: u64,
    pub entity_b: u64,
    //pub event_type: , // 0 = Enter, 1 = Exit
}#[repr(C)]
pub struct BodyStateFFI {
    pub x: f32,
    pub y: f32,
    pub rotation: f32,          // radians
    pub velocity_x: f32,
    pub velocity_y: f32,
    pub angular_velocity: f32,  // rad/s
}
#[repr(C)]
pub struct UpdatesCoutsFFI {
    pub trans_count: usize,
    pub collision_event_count: usize,
    pub trigger_event_count: usize,
}



#[unsafe(no_mangle)]
pub extern "C" fn world_create() -> *mut RapierWorld {
    let world = RapierWorld::new();
    Box::into_raw(Box::new(world))
}
#[unsafe(no_mangle)]
pub extern "C" fn world_destroy(world: *mut RapierWorld) {
     debug_assert!(
        !world.is_null(),
        "world_destroy called with null pointer"
    );
    if world.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(world));
    }
}
#[unsafe(no_mangle)]
pub extern "C" fn world_set_layer_filters(
    world_ptr: *mut RapierWorld,
    filters: *const u32,
) {
    let world = unsafe { &mut *world_ptr };
    let filters = unsafe { std::slice::from_raw_parts(filters, 32) };

    for layer in 0..32 {
        world.layer_groups[layer] = InteractionGroups {
            memberships: Group::from_bits_truncate(1 << layer),
            filter: Group::from_bits_truncate(filters[layer]),
            test_mode: InteractionTestMode::And,
        };
    }
}
#[unsafe(no_mangle)]
pub extern "C" fn ignore_collision(
    world: *mut RapierWorld,
    body1_handle: u64,
    body2_handle: u64,
    ignore: bool,
) {
    let world = unsafe { &mut *world };

    let body1 = RigidBodyHandle::from_raw_parts(
        body1_handle as u32,
        (body1_handle >> 32) as u32,
    );

    let body2 = RigidBodyHandle::from_raw_parts(
        body2_handle as u32,
        (body2_handle >> 32) as u32,
    );

    let collider1 = *world.bodies[body1].colliders().first().unwrap();
    let collider2 = *world.bodies[body2].colliders().first().unwrap();

    let pair = {
        let (i1, g1) = collider1.into_raw_parts();
        let (i2, g2) = collider2.into_raw_parts();

        if (i1, g1) < (i2, g2) {
            (collider1, collider2)
        } else {
            (collider2, collider1)
        }
    };

    if ignore {
        world.ignore_pairs.ignored.insert(pair);
        world.colliders.get_mut(collider1).unwrap().set_active_hooks(ActiveHooks::FILTER_CONTACT_PAIRS);
        world.colliders.get_mut(collider2).unwrap().set_active_hooks(ActiveHooks::FILTER_CONTACT_PAIRS);
    } else {
        world.ignore_pairs.ignored.remove(&pair);
        world.colliders.get_mut(collider1).unwrap().set_active_hooks(ActiveHooks::empty());
        world.colliders.get_mut(collider2).unwrap().set_active_hooks(ActiveHooks::empty());
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn body_create(
    worldptr: *mut RapierWorld,
    typ: u8,
    mass: f32,
    x: f32,
    y: f32,
    rot: f32, // radians
    linear_damping: f32,
    angular_damping: f32,
    freeze_rotation: bool
) -> u64 {
    let mut builder = match typ {
        0 => RigidBodyBuilder::dynamic(),
        1 => RigidBodyBuilder::kinematic_position_based(),
        2 => RigidBodyBuilder::fixed(),
        _ => RigidBodyBuilder::dynamic(),
    };

    builder = builder
        .additional_mass(mass)
        .translation(Vec2::new(x, y))
        .rotation(rot);

    if freeze_rotation {
        builder = builder.lock_rotations();
    }

    let body = builder
    .linear_damping(linear_damping)
    .angular_damping(angular_damping)
    .build();

    let world = unsafe { &mut *worldptr };
    let handle = world.bodies.insert(body);

    let (index, generation) = handle.into_raw_parts();
    ((generation as u64) << 32) | (index as u64) 
}
#[unsafe(no_mangle)]
pub extern "C" fn body_destroy(
    worldptr: *mut RapierWorld,
    handle: u64
)
{

    let index = handle as u32;
    let generation = (handle >> 32) as u32;

    let body_handle =
        rapier2d::prelude::RigidBodyHandle::from_raw_parts(index, generation);

    let world = unsafe { &mut *worldptr };

    world.bodies.remove(
            body_handle,
            &mut world.islands,
            &mut world.colliders,
            &mut world.impulse_joints,
            &mut world.multibody_joints,
            true, // remove attached colliders
    );
}


#[unsafe(no_mangle)]
pub extern "C" fn body_add_circle_collider(
    worldptr: *mut RapierWorld,
    handle: u64,
    radius: f32,
    ox: f32,
    oy: f32,
    friction: f32,
    layer: u8,
    register_event: bool
) {
    let world = unsafe { &mut *worldptr };

    let index = handle as u32;
    let generation = (handle >> 32) as u32;
    let group = world.layer_groups[layer as usize];

    let body_handle =
        rapier2d::prelude::RigidBodyHandle::from_raw_parts(index, generation);

    let collider = ColliderBuilder::ball(radius)
        .translation(Vec2::new(ox, oy))
        .friction(friction)
        .collision_groups(group)
        .active_events(
        if register_event {ActiveEvents::COLLISION_EVENTS} else {ActiveEvents::empty()})
        .build();

    world.colliders.insert_with_parent(
        collider,
        body_handle,
        &mut world.bodies,
    );
}
#[unsafe(no_mangle)]
pub extern "C" fn add_standalone_circle_collider(
    worldptr: *mut RapierWorld,
    radius: f32,
    ox: f32,
    oy: f32,
    layer: u8,
    //collide_event:bool
    is_trigger: bool
) -> u64 
{
    let world = unsafe { &mut *worldptr };
    let group = world.layer_groups[layer as usize];

    let collider = ColliderBuilder::ball(radius)
        .translation(Vec2::new(ox, oy))
        .collision_groups(group)
        .sensor(is_trigger) // trigger
        .active_events(ActiveEvents::COLLISION_EVENTS)
        .build();

    let collider_handle = world.colliders.insert(collider);

    let (index, generation) = collider_handle.into_raw_parts();
    // to have unique handle for bodies and colliders
    COLLIDER_FLAG | ((generation as u64) << 32) | (index as u64)  
}
#[unsafe(no_mangle)]
pub extern "C" fn body_add_box_collider(
    world_ptr: *mut RapierWorld,
    handle: u64,
    hx: f32,
    hy: f32,
    ox: f32,
    oy: f32,
    friction: f32,
    layer: u8,
    collide_event: bool 
) {
    let world = unsafe { &mut *world_ptr };

    let index = handle as u32;
    let generation = (handle >> 32) as u32;
    let group = world.layer_groups[layer as usize];

    let body_handle =
        rapier2d::prelude::RigidBodyHandle::from_raw_parts(index, generation);

    let collider = ColliderBuilder::cuboid(hx, hy)
        .translation(Vec2::new(ox, oy))
        .friction(friction)
        .collision_groups(group)
        .active_events(
        if collide_event {ActiveEvents::COLLISION_EVENTS} else {ActiveEvents::empty()})
        .build();

    world.colliders.insert_with_parent(
        collider,
        body_handle,
        &mut world.bodies,
    );
}
#[unsafe(no_mangle)]
pub extern  "C" fn add_standalone_box_collider(
    world_ptr: *mut RapierWorld,
    hx: f32,
    hy: f32,
    ox: f32,
    oy: f32,
    layer: u8,
    //collide_event: bool,
    is_trigger: bool,
)-> u64
{
    let world = unsafe { &mut *world_ptr };
    let group = world.layer_groups[layer as usize];
    
    let collider = ColliderBuilder::cuboid(hx, hy)
    .translation(Vec2::new(ox, oy))
    .collision_groups(group)
    .sensor(is_trigger) // trigger
    .active_events(ActiveEvents::COLLISION_EVENTS)
    .build();

    let collider_handle = world.colliders.insert(collider);

    let (index, generation) = collider_handle.into_raw_parts();
    // to have unique handle for bodies and colliders
    COLLIDER_FLAG | ((generation as u64) << 32) | (index as u64)  
}
#[unsafe(no_mangle)]
pub extern "C" fn body_add_polyline_collider(
    world_ptr: *mut RapierWorld,
    handle: u64,
    vertices_ptr: *const Vec2,
    count: usize,
    friction: f32
) {

    let world = unsafe { &mut *world_ptr };

    let index = handle as u32;
    let generation = (handle >> 32) as u32;

    let body_handle =
        rapier2d::prelude::RigidBodyHandle::from_raw_parts(index, generation);

    let vertices = unsafe {
        std::slice::from_raw_parts(vertices_ptr, count).to_vec()
    };
    let mut indices = Vec::with_capacity(vertices.len());
            for i in 0..vertices.len() {
                indices.push([i as u32, ((i + 1) % vertices.len()) as u32]);
            }

    let collider = ColliderBuilder::polyline(vertices, Some(indices)).friction(friction).build();
   
    world.colliders.insert_with_parent(
        collider,
        body_handle,
        &mut world.bodies,
    );
}

#[unsafe(no_mangle)]
pub extern "C" fn joint_add_revolute(
    world: *mut RapierWorld,
    body1: u64,
    body2: u64,
    enable_collision:bool,
    connected_to_world:bool,
    ax1: f32,
    ay1: f32,
    ax2: f32,
    ay2: f32,

    use_limits: bool,
    min_angle: f32,
    max_angle: f32,

    motor_enabled: bool,
    target_angle: f32,
    stiffness: f32,
    damping: f32,
    velocity_target: f32,
    velocity_gain: f32,
    max_torque: f32,
    
) -> u64 {
    let world = unsafe { &mut *world };

    let h1 = RigidBodyHandle::from_raw_parts(body1 as u32, (body1 >> 32) as u32);
    let h2 = if connected_to_world {
            world.world_body
        } else {
           RigidBodyHandle::from_raw_parts(body2 as u32, (body2 >> 32) as u32)
        };

    let mut joint = RevoluteJointBuilder::new()
        .local_anchor1(Vec2::new(ax1, ay1))
        .local_anchor2(Vec2::new(ax2, ay2))
        .contacts_enabled(enable_collision);


    if use_limits {
        joint = joint.limits([min_angle, max_angle]);
    }

    if motor_enabled {
        joint = joint.motor_position(target_angle, stiffness, damping);
        joint = joint.motor_velocity(velocity_target, velocity_gain);
        joint = joint.motor_max_force(max_torque);
    }

    let handle = world.impulse_joints.insert(h1, h2, joint, true);

    let (index, generation) = handle.into_raw_parts();
    ((generation as u64) << 32) | index as u64
}
#[unsafe(no_mangle)]
pub extern "C" fn joint_add_prismatic(
    world: *mut RapierWorld,

    body1: u64,
    body2: u64,
    connected_to_world: bool,
    enable_collision: bool,

    ax1: f32,
    ay1: f32,
    ax2: f32,
    ay2: f32,

    axis_x: f32,
    axis_y: f32,

    use_limits: bool,
    min_limit: f32,
    max_limit: f32,

    motor_enabled: bool,
    target_position: f32,
    stiffness: f32,
    damping: f32,
    max_force: f32,
) -> u64 {
    let world = unsafe { &mut *world };

    let h1 = RigidBodyHandle::from_raw_parts(
        body1 as u32,
        (body1 >> 32) as u32,
    );

    let h2 = if connected_to_world {
        world.world_body
    } else {
        RigidBodyHandle::from_raw_parts(
            body2 as u32,
            (body2 >> 32) as u32,
        )
    };

    let axis = Vec2::new(axis_x, axis_y).normalize();

    let mut joint = PrismaticJointBuilder::new(axis)
        .local_anchor1(Vec2::new(ax1, ay1))
        .local_anchor2(Vec2::new(ax2, ay2))
        .contacts_enabled(enable_collision);

    if use_limits {
        joint = joint.limits([min_limit, max_limit]);
    }

    if motor_enabled {
        joint = joint.motor_position(
            target_position,
            stiffness,
            damping,
        );

        joint = joint.motor_max_force(max_force);
    }

    let handle = world.impulse_joints.insert(
        h1,
        h2,
        joint,
        true,
    );

    let (index, generation) = handle.into_raw_parts();
    ((generation as u64) << 32) | index as u64
}

#[unsafe(no_mangle)]
pub extern "C" fn joint_set_prismatic_motor(
    world_ptr: *mut RapierWorld,
    handle: u64,
    target_position: f32,
    stiffness: f32,
    damping: f32,
    max_force: f32,
) {
    let world = unsafe { &mut *world_ptr };

    let joint_handle =
        ImpulseJointHandle::from_raw_parts(
            handle as u32,
            (handle >> 32) as u32,
        );

    let prismatic = world.impulse_joints.get_mut(joint_handle, true).unwrap().data.as_prismatic_mut().unwrap();

    prismatic.set_motor_position(
        target_position,
        stiffness,
        damping,
    );

    prismatic.set_motor_max_force(max_force);
}
#[unsafe(no_mangle)]
pub extern "C" fn body_set_position(
    world_ptr: *mut RapierWorld,
    handle: u64,
    x: f32,
    y: f32
) {
    let world = unsafe { &mut *world_ptr };

    let body_handle = RigidBodyHandle::from_raw_parts(
        handle as u32,
        (handle >> 32) as u32,
    );

    if let Some(body) = world.bodies.get_mut(body_handle) {
      let mut pos = *body.position();
        pos.translation.x = x;
        pos.translation.y = y;

        body.set_position(pos, true);
    }
}
#[unsafe(no_mangle)]
pub extern "C" fn body_set_rotation(
    world_ptr: *mut RapierWorld,
    handle: u64,
    angle: f32,
) {
    let world = unsafe { &mut *world_ptr };

    let body_handle = RigidBodyHandle::from_raw_parts(
        handle as u32,
        (handle >> 32) as u32,
    );

    if let Some(body) = world.bodies.get_mut(body_handle) {
        body.set_rotation(
            rapier2d::math::Rot2::new(angle),
            true,
        );
    }
}
#[unsafe(no_mangle)]
pub extern "C" fn body_set_state(
    world: *mut RapierWorld,
    handle: u64,
    body_state: BodyStateFFI 
) {
    unsafe {
        (*world).body_set_state(handle, body_state);
    }
}
#[unsafe(no_mangle)]
pub extern "C" fn body_add_force(
    world: *mut RapierWorld,
    handle: u64,
    linear_x: f32,
    linear_y: f32,
    angular: f32,
) {
    unsafe {
        (*world).body_add_force(handle, linear_x, linear_y, angular);
    }
}


#[unsafe(no_mangle)]
pub extern "C" fn body_get_state(
    world_ptr: *mut RapierWorld,
    handle: u64
)-> BodyStateFFI{
    let world = unsafe { &mut *world_ptr };

   let body_handle = RigidBodyHandle::from_raw_parts(
        handle as u32,
        (handle >> 32) as u32,
    );

    let body = world.bodies.get(body_handle).unwrap();

    BodyStateFFI {
            x: body.translation().x,
            y: body.translation().y,
            rotation: body.rotation().angle(),
            velocity_x: body.linvel().x,
            velocity_y: body.linvel().y,
            angular_velocity: body.angvel(),
        }
}

#[unsafe(no_mangle)]
pub extern "C" fn world_step(
    worldptr: *mut RapierWorld,
    dt: f32,
    tranform_output: *mut TransformUpdate,
    collision_event_output: *mut ColTrigEventFFI,
    trigger_event_output: *mut ColTrigEventFFI,
    trans_capacity: usize,
    collision_event_capacity: usize,
    trigger_event_capacity: usize,
)-> UpdatesCoutsFFI
{

    unsafe {
        
        let world = &mut *worldptr ;
        world.step(dt);
        let tranform_updates = std::slice::from_raw_parts_mut(tranform_output, trans_capacity);
        let collision_events = std::slice::from_raw_parts_mut(collision_event_output, collision_event_capacity);
        let trigger_events = std::slice::from_raw_parts_mut(trigger_event_output, trigger_event_capacity);
    
        let mut trans_count = 0;
        let mut collision_event_count = 0;
        let mut trigger_event_count = 0;

        for rb_handle in world.islands.active_bodies() {

            if trans_count >= trans_capacity {
                break;
            }

            let body = &world.bodies[rb_handle];

            let (index, generation) = rb_handle.into_raw_parts();
            let handle = ((generation as u64) << 32) | (index as u64);

            tranform_updates[trans_count] = TransformUpdate {
                entity_id: handle,
                position: [
                    body.translation().x,
                    body.translation().y,
                ],
                rotation: body.rotation().angle(),
            };

            trans_count += 1;
        }

        while let Ok(event) = world.collision_recv.try_recv() {
            
            match event {
                CollisionEvent::Started(c1, c2, _) => {
                        let collider1 = world.colliders.get(c1).unwrap();
                        let collider2 = world.colliders.get(c2).unwrap();

                        if collider1.is_sensor(){

                            if trigger_event_count >= trigger_event_capacity {
                                break;
                            }
                            let (c1_index, c1_generation) = c1.into_raw_parts();
                            let body2 = collider2.parent().unwrap();
                            let (body2_index, body2_generation) = body2.into_raw_parts();

                            trigger_events[trigger_event_count] = ColTrigEventFFI {
                                entity_a: COLLIDER_FLAG | ((c1_generation as u64) << 32) | (c1_index as u64),
                                entity_b: ((body2_generation as u64) << 32) | (body2_index as u64),
                            };
                            trigger_event_count += 1;
                        }
                        else if collider2.is_sensor()
                        {
                            if trigger_event_count >= trigger_event_capacity {
                                break;
                            }
                            let (c2_index, c2_generation) = c2.into_raw_parts();
                            let body1 = collider1.parent().unwrap();
                            let (body1_index, body1_generation) = body1.into_raw_parts();

                            trigger_events[trigger_event_count] = ColTrigEventFFI {
                                entity_a: COLLIDER_FLAG | ((c2_generation as u64) << 32) | (c2_index as u64),
                                entity_b: ((body1_generation as u64) << 32) | (body1_index as u64),
                            };
                            trigger_event_count += 1;
                        }
                        else{
                            if collision_event_count >= collision_event_capacity {
                                break;
                            }
                            let body1 = collider1.parent().unwrap();
                            let body2 = collider2.parent().unwrap();

                            let (body1_index, body1_generation) = body1.into_raw_parts();
                            let (body2_index, body2_generation) = body2.into_raw_parts();

                            collision_events[collision_event_count] = ColTrigEventFFI {
                                entity_a: ((body1_generation as u64) << 32) | (body1_index as u64),
                                entity_b: ((body2_generation as u64) << 32) | (body2_index as u64),
                            };
                            collision_event_count+=1;
                        }

                        
                }
                CollisionEvent::Stopped(_, _, _) => {
                    // Collision Exit
                }
            }
        }

        UpdatesCoutsFFI{trans_count: trans_count, collision_event_count,trigger_event_count}
    }
}

