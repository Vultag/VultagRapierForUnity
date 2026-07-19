use rapier2d::prelude::*;
use serde::{Serialize, Deserialize};
use crate::world::RapierWorld;
use crate::world::IgnorePairs;
use std::sync::mpsc::channel;


#[derive(Serialize, Deserialize)]
pub struct WorldSnapshot {
    pub gravity: Vec2,
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

    pub layer_groups: [InteractionGroups; 32],
    pub ignore_pairs: IgnorePairs
}
pub struct SerializedWorldSnapshot {
    pub data: Option<Vec<u8>>,
}

impl SerializedWorldSnapshot {
    pub fn new() -> Self {
        Self {
            data: None,
        }
    }
}


impl RapierWorld {
    fn save_snapshot(&self) -> Vec<u8> {
        let snapshot = WorldSnapshot {
            gravity: self.gravity,
            integration_parameters: self.integration_parameters,

            islands: self.islands.clone(),
            broad_phase: self.broad_phase.clone(),
            narrow_phase: self.narrow_phase.clone(),

            world_body: self.world_body,
            bodies: self.bodies.clone(),
            colliders: self.colliders.clone(),
            impulse_joints: self.impulse_joints.clone(),
            multibody_joints: self.multibody_joints.clone(),
            ccd_solver: self.ccd_solver.clone(),

            layer_groups: self.layer_groups,
            ignore_pairs: self.ignore_pairs.clone(),
        };

        bincode::serialize(&snapshot).unwrap()
    }
    fn restore(&mut self, snapshot: WorldSnapshot) {
        self.pipeline = PhysicsPipeline::new();

        self.gravity = snapshot.gravity;
        self.integration_parameters = snapshot.integration_parameters;

        self.islands = snapshot.islands;
        self.broad_phase = snapshot.broad_phase;
        self.narrow_phase = snapshot.narrow_phase;

        self.world_body = snapshot.world_body;

        self.bodies = snapshot.bodies;
        self.colliders = snapshot.colliders;

        self.impulse_joints = snapshot.impulse_joints;
        self.multibody_joints = snapshot.multibody_joints;

        self.ccd_solver = snapshot.ccd_solver;

        self.layer_groups = snapshot.layer_groups;

        self.ignore_pairs = snapshot.ignore_pairs;

        // Recreate runtime-only event system
        let (collision_send, collision_recv) = channel();
        let (contact_send, contact_recv) = channel();

        self.collision_recv = collision_recv;
        self.contact_recv = contact_recv;

        self.event_handler = ChannelEventCollector::new(
            collision_send,
            contact_send,
        );
    }
    
    fn store_snapshot(&mut self) {
        self.snapshot.data = Some(self.save_snapshot());
    }

    fn restore_snapshot(&mut self) {
     if let Some(data) = self.snapshot.data.take() {
            let snapshot: WorldSnapshot =
                bincode::deserialize(&data).unwrap();

            self.restore(snapshot);

            self.snapshot.data = Some(data);
        }
    }

}
#[unsafe(no_mangle)]
pub extern "C" fn world_store_snapshot(
    world_ptr: *mut RapierWorld
) {
    let world = unsafe { &mut *world_ptr };

    world.store_snapshot();
}
#[unsafe(no_mangle)]
pub extern "C" fn world_restore_snapshot(
    world_ptr: *mut RapierWorld
) {
    let world = unsafe { &mut *world_ptr };

    world.restore_snapshot();
}