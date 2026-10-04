//! The canonical order of one step's events, which does not depend on which thread reported
//! what first.
//!
//! Every comparison is total: after the key it compares the whole payload bit for bit, so only
//! identical events compare equal, and identical events are interchangeable.

use std::cmp::Ordering;

use super::{
    ActivationEvent, ContactEvent, ContactManifold, ContactSettings, SoftBodyContactSettings,
    SoftBodyContacts, SoftBodyValidateResult, SoftBodyValidation, SubShapeIdPair,
};
use crate::{RVec3, Real, Vec3};

pub(super) fn sort_contacts(events: &mut [ContactEvent]) {
    events.sort_by(compare_contacts);
}

/// Stable by body: Jolt reports one body's changes in causal order under its mutex.
pub(super) fn sort_activations(events: &mut [ActivationEvent]) {
    events.sort_by_key(|event| event.body().to_raw());
}

pub(super) fn sort_soft_body_validations(events: &mut [SoftBodyValidation]) {
    events.sort_by(|a, b| {
        (a.soft_body.to_raw(), a.other.to_raw())
            .cmp(&(b.soft_body.to_raw(), b.other.to_raw()))
            .then_with(|| result_rank(a.result).cmp(&result_rank(b.result)))
            .then_with(|| soft_settings_bits(&a.settings).cmp(&soft_settings_bits(&b.settings)))
    });
}

pub(super) fn sort_soft_body_contacts(events: &mut [SoftBodyContacts]) {
    events.sort_by(|a, b| {
        a.soft_body
            .to_raw()
            .cmp(&b.soft_body.to_raw())
            .then_with(|| compare_soft_payload(a, b))
    });
}

fn pair_key(pair: &SubShapeIdPair) -> [u32; 4] {
    [
        pair.body1.to_raw(),
        pair.sub_shape1.to_raw(),
        pair.body2.to_raw(),
        pair.sub_shape2.to_raw(),
    ]
}

fn kind_rank(event: &ContactEvent) -> u8 {
    match event {
        ContactEvent::Added { .. } => 0,
        ContactEvent::Persisted { .. } => 1,
        ContactEvent::Removed(_) => 2,
    }
}

fn compare_contacts(a: &ContactEvent, b: &ContactEvent) -> Ordering {
    pair_key(&a.pair())
        .cmp(&pair_key(&b.pair()))
        .then_with(|| kind_rank(a).cmp(&kind_rank(b)))
        .then_with(|| match (a, b) {
            (
                ContactEvent::Added {
                    manifold: m1,
                    settings: s1,
                },
                ContactEvent::Added {
                    manifold: m2,
                    settings: s2,
                },
            )
            | (
                ContactEvent::Persisted {
                    manifold: m1,
                    settings: s1,
                },
                ContactEvent::Persisted {
                    manifold: m2,
                    settings: s2,
                },
            ) => settings_bits(s1)
                .cmp(&settings_bits(s2))
                .then_with(|| s1.rule_bits().cmp(&s2.rule_bits()))
                .then_with(|| compare_manifolds(m1, m2)),
            _ => Ordering::Equal,
        })
}

fn vec3_bits(v: Vec3) -> [u32; 3] {
    [v.x, v.y, v.z].map(f32::to_bits)
}

/// The bits of a position, in either precision.
fn rvec3_bits(v: RVec3) -> [impl Ord; 3] {
    [v.x, v.y, v.z].map(Real::to_bits)
}

fn settings_bits(s: &ContactSettings) -> [u32; 13] {
    let [lx, ly, lz] = vec3_bits(s.relative_linear_surface_velocity());
    let [ax, ay, az] = vec3_bits(s.relative_angular_surface_velocity());
    [
        s.combined_friction().to_bits(),
        s.combined_restitution().to_bits(),
        s.inv_mass_scale1().to_bits(),
        s.inv_inertia_scale1().to_bits(),
        s.inv_mass_scale2().to_bits(),
        s.inv_inertia_scale2().to_bits(),
        u32::from(s.is_sensor()),
        lx,
        ly,
        lz,
        ax,
        ay,
        az,
    ]
}

fn compare_manifolds(a: &ContactManifold, b: &ContactManifold) -> Ordering {
    a.materials
        .cmp(&b.materials)
        .then_with(|| vec3_bits(a.normal).cmp(&vec3_bits(b.normal)))
        .then_with(|| {
            a.penetration_depth
                .to_bits()
                .cmp(&b.penetration_depth.to_bits())
        })
        .then_with(|| a.points.len().cmp(&b.points.len()))
        .then_with(|| point_bits(a).cmp(point_bits(b)))
}

fn point_bits(m: &ContactManifold) -> impl Iterator<Item = impl Ord> + '_ {
    m.points
        .iter()
        .map(|p| (rvec3_bits(p.on1), rvec3_bits(p.on2)))
}

fn result_rank(result: SoftBodyValidateResult) -> u8 {
    match result {
        SoftBodyValidateResult::AcceptContact => 0,
        SoftBodyValidateResult::RejectContact => 1,
    }
}

fn soft_settings_bits(s: &SoftBodyContactSettings) -> [u32; 4] {
    [
        s.inv_mass_scale1().to_bits(),
        s.inv_mass_scale2().to_bits(),
        s.inv_inertia_scale2().to_bits(),
        u32::from(s.is_sensor()),
    ]
}

fn compare_soft_payload(a: &SoftBodyContacts, b: &SoftBodyContacts) -> Ordering {
    vertex_bits(a)
        .cmp(vertex_bits(b))
        .then_with(|| sensor_ids(a).cmp(sensor_ids(b)))
}

fn vertex_bits(c: &SoftBodyContacts) -> impl Iterator<Item = impl Ord> + '_ {
    c.vertices.iter().map(|v| {
        (
            v.vertex,
            v.body.to_raw(),
            rvec3_bits(v.position),
            vec3_bits(v.normal),
        )
    })
}

fn sensor_ids(c: &SoftBodyContacts) -> impl Iterator<Item = u32> + '_ {
    c.sensors.iter().map(|s| s.to_raw())
}

#[cfg(test)]
mod tests {
    use joltphysics_sys::{JPH_ContactSettings, JPH_SoftBodyContactSettings};

    use super::*;
    use crate::listener::contact::ContactFacts;
    use crate::listener::ContactPoint;
    use crate::world::WorldTag;
    use crate::{BodyId, SubShapeId};

    fn tag() -> WorldTag {
        WorldTag::next()
    }

    fn pair(world: WorldTag, body1: u32, body2: u32) -> SubShapeIdPair {
        SubShapeIdPair {
            body1: BodyId::new(body1, world),
            sub_shape1: SubShapeId::new(u32::MAX),
            body2: BodyId::new(body2, world),
            sub_shape2: SubShapeId::new(7),
        }
    }

    fn settings() -> ContactSettings {
        settings_with(|_| {})
    }

    fn settings_with(change: impl FnOnce(&mut JPH_ContactSettings)) -> ContactSettings {
        let mut settings = JPH_ContactSettings {
            combinedFriction: 0.5,
            combinedRestitution: 0.0,
            invMassScale1: 1.0,
            invInertiaScale1: 1.0,
            invMassScale2: 1.0,
            invInertiaScale2: 1.0,
            isSensor: 0,
            relativeLinearSurfaceVelocity: Vec3::ZERO.to_jph(),
            relativeAngularSurfaceVelocity: Vec3::ZERO.to_jph(),
        };
        change(&mut settings);
        let facts = ContactFacts {
            sensor_body: false,
            lever_arm: 0.0,
        };
        ContactSettings::new(&settings, facts)
    }

    fn manifold(pair: SubShapeIdPair, depth: f32) -> ContactManifold {
        ContactManifold {
            pair,
            normal: Vec3::new(0.0, 1.0, 0.0),
            penetration_depth: depth,
            points: vec![ContactPoint {
                on1: RVec3::new(0.0, 0.0, 0.0),
                on2: RVec3::new(0.0, -0.01, 0.0),
            }],
            materials: [None, Some(3)],
        }
    }

    /// A small deterministic shuffle (a linear congruential generator), so the test needs no
    /// random crate.
    fn shuffled<T: Clone>(items: &[T], seed: u64) -> Vec<T> {
        let mut items = items.to_vec();
        let mut state = seed;
        for i in (1..items.len()).rev() {
            state = state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            items.swap(i, (state >> 33) as usize % (i + 1));
        }
        items
    }

    fn sample_contacts() -> Vec<ContactEvent> {
        let world = tag();
        let added = |body1, body2, depth| ContactEvent::Added {
            manifold: manifold(pair(world, body1, body2), depth),
            settings: settings(),
        };
        let persisted = |body1, body2, depth| ContactEvent::Persisted {
            manifold: manifold(pair(world, body1, body2), depth),
            settings: settings(),
        };
        vec![
            added(1, 2, 0.01),
            persisted(1, 2, 0.01),
            persisted(1, 2, 0.02),
            ContactEvent::Removed(pair(world, 1, 3)),
            added(0, 5, 0.0),
            added(0, 5, -0.0),
            persisted(1, 2, 0.01),
            ContactEvent::Removed(pair(world, 1, 2)),
        ]
    }

    #[test]
    fn shuffled_contacts_sort_identically_and_keep_duplicates() {
        let events = sample_contacts();
        let mut reference = events.clone();
        sort_contacts(&mut reference);
        for seed in 0..50 {
            let mut sorted = shuffled(&events, seed);
            sort_contacts(&mut sorted);
            assert_eq!(sorted, reference, "seed {seed}");
        }
        assert_eq!(
            reference.len(),
            events.len(),
            "duplicates keep their multiplicity"
        );
        let duplicates = reference
            .windows(2)
            .filter(|w| compare_contacts(&w[0], &w[1]) == Ordering::Equal)
            .count();
        assert_eq!(duplicates, 1);
    }

    #[test]
    fn contacts_order_by_pair_then_kind() {
        let mut events = sample_contacts();
        sort_contacts(&mut events);
        let keys: Vec<([u32; 4], u8)> = events
            .iter()
            .map(|e| (pair_key(&e.pair()), kind_rank(e)))
            .collect();
        assert!(keys.windows(2).all(|w| w[0] <= w[1]), "{keys:?}");
    }

    #[test]
    fn every_payload_field_separates_contacts() {
        let world = tag();
        let base = manifold(pair(world, 1, 2), 0.0);
        let event = |manifold: ContactManifold, settings: ContactSettings| ContactEvent::Added {
            manifold,
            settings,
        };
        let reference = event(base.clone(), settings());
        let mut variants = Vec::new();
        let mut m = base.clone();
        m.penetration_depth = -0.0;
        variants.push(event(m, settings()));
        let mut m = base.clone();
        m.normal.x = -0.0;
        variants.push(event(m, settings()));
        let mut m = base.clone();
        m.materials = [Some(1), Some(3)];
        variants.push(event(m, settings()));
        let mut m = base.clone();
        m.points[0].on2.z = -0.0;
        variants.push(event(m, settings()));
        let mut m = base.clone();
        m.points.push(m.points[0]);
        variants.push(event(m, settings()));
        variants.push(event(
            base.clone(),
            settings_with(|s| s.combinedFriction = 0.25),
        ));
        variants.push(event(base.clone(), settings_with(|s| s.isSensor = 1)));
        variants.push(event(
            base.clone(),
            settings_with(|s| s.relativeAngularSurfaceVelocity.y = -0.0),
        ));
        let mut m = base.clone();
        m.points[0].on1.x = -0.0;
        variants.push(event(m, settings()));
        let changes: [fn(&mut JPH_ContactSettings); 6] = [
            |s| s.combinedRestitution = 0.5,
            |s| s.invMassScale1 = 0.5,
            |s| s.invInertiaScale1 = 0.5,
            |s| s.invMassScale2 = 0.5,
            |s| s.invInertiaScale2 = 0.5,
            |s| s.relativeLinearSurfaceVelocity.x = -0.0,
        ];
        for change in changes {
            variants.push(event(base.clone(), settings_with(change)));
        }
        for variant in variants {
            assert_ne!(
                compare_contacts(&reference, &variant),
                Ordering::Equal,
                "{variant:?}"
            );
            assert_eq!(
                compare_contacts(&reference, &variant),
                compare_contacts(&variant, &reference).reverse()
            );
        }
    }

    #[test]
    fn nan_payloads_separate_contacts() {
        let world = tag();
        let quiet = f32::from_bits(0x7fc0_0000);
        let other = f32::from_bits(0x7fc0_0001);
        let with = |depth: f32, z: crate::Real| {
            let mut m = manifold(pair(world, 1, 2), depth);
            m.points[0].on1.z = z;
            ContactEvent::Added {
                manifold: m,
                settings: settings(),
            }
        };
        let real_nan =
            |bits_offset| crate::Real::from_bits(crate::Real::NAN.to_bits() + bits_offset);
        assert_ne!(
            compare_contacts(&with(quiet, 0.0), &with(other, 0.0)),
            Ordering::Equal
        );
        assert_ne!(
            compare_contacts(&with(0.0, real_nan(0)), &with(0.0, real_nan(1))),
            Ordering::Equal
        );
    }

    #[test]
    fn activations_sort_stably_by_body() {
        let world = tag();
        let a = |raw| ActivationEvent::Activated(BodyId::new(raw, world));
        let d = |raw| ActivationEvent::Deactivated(BodyId::new(raw, world));
        let mut events = vec![a(3), a(1), d(3), d(1), a(3)];
        sort_activations(&mut events);
        assert_eq!(events, vec![a(1), d(1), a(3), d(3), a(3)]);
    }

    #[test]
    fn soft_body_events_sort_by_soft_body_and_payload() {
        let world = tag();
        let soft_settings = |sensor| {
            SoftBodyContactSettings::from_jph(&JPH_SoftBodyContactSettings {
                invMassScale1: 1.0,
                invMassScale2: 1.0,
                invInertiaScale2: 1.0,
                isSensor: sensor,
            })
        };
        let validation = |soft, other, sensor| SoftBodyValidation {
            soft_body: BodyId::new(soft, world),
            other: BodyId::new(other, world),
            settings: soft_settings(sensor),
            result: SoftBodyValidateResult::AcceptContact,
        };
        let validations = vec![
            validation(4, 1, true),
            validation(2, 9, false),
            validation(4, 1, false),
            validation(2, 1, false),
        ];
        let mut reference = validations.clone();
        sort_soft_body_validations(&mut reference);
        assert_eq!(
            reference,
            vec![
                validation(2, 1, false),
                validation(2, 9, false),
                validation(4, 1, false),
                validation(4, 1, true),
            ]
        );
        let contacts = |soft, position_z: crate::Real| SoftBodyContacts {
            soft_body: BodyId::new(soft, world),
            vertices: vec![crate::listener::SoftBodyVertexContact {
                vertex: 0,
                body: BodyId::new(1, world),
                position: RVec3::new(0.0, 0.0, position_z),
                normal: Vec3::new(0.0, -1.0, 0.0),
            }],
            sensors: Vec::new(),
        };
        let mut sorted = vec![contacts(5, 0.0), contacts(3, 1.0), contacts(5, -0.0)];
        sort_soft_body_contacts(&mut sorted);
        assert_eq!(sorted[0].soft_body.to_raw(), 3);
        assert_eq!(
            sorted[1].vertices[0].position.z.to_bits(),
            RVec3::ZERO.z.to_bits()
        );
    }
}
