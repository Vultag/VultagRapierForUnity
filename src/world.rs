use std::sync::mpsc::{channel, Receiver};
use rapier2d::prelude::*;
use std::collections::HashSet;
use serde::{Serialize, Deserialize};
use crate::commands::BodyStateFFI;
use crate::snapshots::SerializedWorldSnapshot;

#[derive(Serialize, Deserialize, Clone)]
pub struct IgnorePairs {
    pub ignored: HashSet<(ColliderHandle, ColliderHandle)>,
}
impl PhysicsHooks for IgnorePairs {
    fn filter_contact_pair(
        &self,
        context: &PairFilterContext,
    ) -> Option<SolverFlags> {
        let a = context.collider1;
        let b = context.collider2;

        if self.ignored.contains(&(a, b)) || self.ignored.contains(&(b, a)) {
            return None; // Ignore this pair
        }
        Some(SolverFlags::COMPUTE_IMPULSES)
    }
}


#[repr(C)]
pub struct BodyHandle {
    pub index: u32,
    pub generation: u32,
}

pub struct RapierWorld {
    pub pipeline: PhysicsPipeline,
    pub gravity:  Vec2,
    pub integration_parameters: IntegrationParameters,

    pub islands: IslandManager,
    pub broad_phase: BroadPhaseBvh,
    pub narrow_phase: NarrowPhase,

    pub world_body: RigidBodyHandle,
    pub bodies: RigidBodySet,
    pub colliders: ColliderSet,
    pub impulse_joints: ImpulseJointSet,
    pub multibody_joints: MultibodyJointSet,
    pub ccd_solver: CCDSolver,

    pub layer_filters: [u32; 32],

    pub collision_recv: Receiver<CollisionEvent>,
    pub contact_recv: Receiver<ContactForceEvent>,
    pub ignore_pairs: IgnorePairs,
    pub event_handler: ChannelEventCollector,
    pub snapshot: SerializedWorldSnapshot
}

impl RapierWorld {
    pub fn new() -> Self {
            let (collision_send, collision_recv) = channel::<CollisionEvent>();
            let (contact_send, contact_recv) = channel::<ContactForceEvent>();
      
            let mut bodies = RigidBodySet::new();
            let world_body = bodies.insert(
                RigidBodyBuilder::fixed().build()
            );
            let ignore_pairs = IgnorePairs {
                ignored: HashSet::new(),
            };

        Self {
            pipeline: PhysicsPipeline::new(),
            gravity: Vec2::new(0.0, -9.81),
            integration_parameters: IntegrationParameters::default(),

            islands: IslandManager::new(),
            broad_phase: BroadPhaseBvh::new(),
            narrow_phase: NarrowPhase::new(),

            world_body,
            bodies,
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),

            layer_filters: [0; 32],

            collision_recv,
            contact_recv,
            ignore_pairs,
            event_handler: ChannelEventCollector::new(
                collision_send,
                contact_send,
            ),
            snapshot: SerializedWorldSnapshot::new(),
            }
    }
    pub fn step(&mut self, dt: f32) {
        self.integration_parameters.dt = dt;

        self.pipeline.step(
            self.gravity,
            &self.integration_parameters,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            &mut self.ignore_pairs,
            &mut self.event_handler,
        );
    }
    pub fn body_add_force(  
    &mut self,
    handle: u64,
    linear_x: f32,
    linear_y: f32,
    angular: f32)
    {
        let index = handle as u32;
        let generation = (handle >> 32) as u32;

        let body_handle = rapier2d::prelude::RigidBodyHandle::from_raw_parts(index, generation);

        let body = &mut self.bodies[body_handle];
        body.apply_impulse(Vec2::new(linear_x, linear_y), true);
        body.apply_torque_impulse(angular, true);
    }
     pub fn body_set_state(  
    &mut self,
    handle: u64,
    body_state: BodyStateFFI 
    )
    {
        let index = handle as u32;
        let generation = (handle >> 32) as u32;

        let body_handle = rapier2d::prelude::RigidBodyHandle::from_raw_parts(index, generation);

        let body = &mut self.bodies[body_handle];
    
        body.set_translation(Vec2::new(body_state.x, body_state.y), true);
        body.set_rotation(Rot2::new(body_state.rotation), true);
        body.set_linvel(Vec2::new(body_state.velocity_x,body_state.velocity_y), true);
        body.set_angvel(body_state.angular_velocity, true);
        body.reset_forces(false);
        body.reset_torques(false);
    }
    


}
