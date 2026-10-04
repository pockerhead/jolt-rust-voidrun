//! The configurations the study compares: CharacterVirtual's creation and update settings, the
//! call pattern around the update and the game's own passes.

use oxijolt::*;

use super::frame::UpPolicy;
use super::scenes::Scene;
use crate::common::math::{rvec3, scale, sub, vec3, V3};
use crate::common::walker::{
    capsule, controller_filter, from_y_to, Layers, Walker, CENTRE_UP, HALF_HEIGHT, STEP_HEIGHT,
};

/// Height of the body origin (lower sphere centre) above the character position along up, for
/// padding `p`: Jolt puts the shape at position + shape offset + padding along up.
pub fn origin_above_position(p: f32) -> f32 {
    p + CENTRE_UP - HALF_HEIGHT
}

/// The character position for body origin `origin`.
pub fn position_for(origin: V3, up: V3, p: f32) -> RVec3 {
    rvec3(sub(origin, scale(up, f64::from(origin_above_position(p)))))
}

/// CharacterVirtual's creation settings as plain numbers.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SettingsSpec {
    pub max_slope_deg: f32,
    pub padding: f32,
    pub predictive_contact_distance: f32,
    pub collision_tolerance: f32,
    pub enhanced_internal_edge_removal: bool,
    pub back_face_collision: bool,
    pub recovery_speed: f32,
    pub inner_body: bool,
    pub max_num_hits: u32,
    pub hit_reduction_cos_max_angle: f32,
    /// The game's supporting plane through the lower sphere centre (spec D.1); Jolt's default
    /// (every contact supports) otherwise.
    pub supporting_plane: bool,
}

impl SettingsSpec {
    /// The game's controller (spec D.1), as `walker::controller_settings`.
    pub const SPEC_D1: Self = Self {
        max_slope_deg: 45.0,
        padding: 0.02,
        predictive_contact_distance: 0.1,
        collision_tolerance: 1.0e-3,
        enhanced_internal_edge_removal: true,
        back_face_collision: true,
        recovery_speed: 1.0,
        inner_body: false,
        max_num_hits: 256,
        hit_reduction_cos_max_angle: 0.999,
        supporting_plane: true,
    };

    /// Jolt's defaults with the game's capsule, slope limit and padding.
    pub const JOLT_DEFAULTS: Self = Self {
        enhanced_internal_edge_removal: false,
        supporting_plane: false,
        ..Self::SPEC_D1
    };

    /// The creation settings for `shape` with up `up`.
    pub fn character_settings<'a>(
        &self,
        shape: &'a Shape,
        inner: Option<InnerBody<'a>>,
        up: V3,
    ) -> CharacterSettings<'a> {
        let mut settings = CharacterSettings::new(shape)
            .shape_offset(Vec3::new(0.0, CENTRE_UP, 0.0))
            .character_padding(self.padding)
            .max_slope_angle(self.max_slope_deg.to_radians())
            .enhanced_internal_edge_removal(self.enhanced_internal_edge_removal);
        if self.supporting_plane {
            settings = settings.supporting_volume(
                Vec3::new(0.0, 1.0, 0.0),
                -origin_above_position(self.padding),
            );
        }
        settings
            .predictive_contact_distance(self.predictive_contact_distance)
            .collision_tolerance(self.collision_tolerance)
            .back_face_collision(self.back_face_collision)
            .penetration_recovery_speed(self.recovery_speed)
            .max_num_hits(self.max_num_hits)
            .hit_reduction_cos_max_angle(self.hit_reduction_cos_max_angle)
            .inner_body(inner)
            .up(vec3(up))
    }
}

/// Jolt's `cos(75 degrees)` as `ExtendedUpdateSettings::default` computes it.
pub fn jolt_cos_forward_contact() -> f32 {
    (75.0 * (std::f32::consts::PI / 180.0_f32)).cos()
}

/// ExtendedUpdate's settings as lengths along up.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExtendedSpec {
    /// Stick-to-floor step down, metres along -up; 0 turns it off.
    pub stick: f32,
    /// Walk-stairs step up, metres along up; 0 turns it off.
    pub stairs_up: f32,
    pub min_step_forward: f32,
    pub step_forward_test: f32,
    pub cos_forward_contact: f32,
    /// Walk-stairs step down extra, metres along -up.
    pub step_down_extra: f32,
}

impl ExtendedSpec {
    /// Both features off; the walk-stairs distances at Jolt's defaults.
    pub fn off() -> Self {
        Self {
            stick: 0.0,
            stairs_up: 0.0,
            min_step_forward: 0.02,
            step_forward_test: 0.15,
            cos_forward_contact: jolt_cos_forward_contact(),
            step_down_extra: 0.0,
        }
    }

    /// Jolt's `ExtendedUpdateSettings` defaults.
    pub fn jolt() -> Self {
        Self {
            stick: 0.5,
            stairs_up: 0.4,
            ..Self::off()
        }
    }

    /// Spec D.1: stick 0.3, step up 0.45, step forward 0.15, forward test 0.5.
    pub fn spec_d1() -> Self {
        Self {
            stick: 0.3,
            stairs_up: 0.45,
            min_step_forward: 0.15,
            step_forward_test: 0.5,
            ..Self::off()
        }
    }

    /// The update settings for this tick's `up`, with stick to floor only when `stick` is set.
    pub fn update_settings(&self, up: V3, stick: bool) -> ExtendedUpdateSettings {
        let along = |metres: f32| {
            if metres == 0.0 {
                Vec3::ZERO
            } else {
                vec3(scale(up, f64::from(metres)))
            }
        };
        let stick_vector = if stick && self.stick > 0.0 {
            vec3(scale(up, -f64::from(self.stick)))
        } else {
            Vec3::ZERO
        };
        let extra = if self.step_down_extra == 0.0 {
            Vec3::ZERO
        } else {
            vec3(scale(up, -f64::from(self.step_down_extra)))
        };
        ExtendedUpdateSettings::default()
            .stick_to_floor_step_down(stick_vector)
            .walk_stairs_step_up(along(self.stairs_up))
            .walk_stairs_min_step_forward(self.min_step_forward)
            .walk_stairs_step_forward_test(self.step_forward_test)
            .walk_stairs_cos_angle_forward_contact(self.cos_forward_contact)
            .walk_stairs_step_down_extra(extra)
    }
}

/// When the stick-to-floor vector is passed to the update.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StickGate {
    /// Always; Jolt applies its own precondition (supported before, not after, not rising).
    Jolt,
    /// Only after a grounded tick and when not rising (the reference near step).
    Caller,
}

/// When the caller refreshes the character's contacts before the move.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refresh {
    Never,
    /// After a maintenance pass moved the character.
    WhenMoved,
    /// Every tick.
    EveryTick,
}

/// When the caller runs an update without velocity before the move, so that Jolt's own
/// penetration recovery is not counted as motion.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Still {
    Never,
    Always,
    /// When a current contact penetrates by more than 1 mm.
    WhenPenetrating,
    /// When the depenetration pass found an overlap deeper than radius + padding.
    WhenDeep,
}

/// The game's own passes (spec C.1, D.2) a configuration runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Passes {
    /// Q3: the underground recovery.
    pub underground: bool,
    /// Q6: the depenetration push before the move.
    pub q6: bool,
    /// The game's autostep.
    pub autostep: bool,
    /// Q5: the floor snap after the move.
    pub q5: bool,
    /// Q4: the terrain support normal for the steep-as-wall rule.
    pub q4: bool,
}

/// One configuration of the study.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Config {
    pub name: &'static str,
    pub settings: SettingsSpec,
    pub extended: ExtendedSpec,
    pub stick_gate: StickGate,
    pub refresh: Refresh,
    pub still: Still,
    /// The recovery speed during the move, restored to the creation value after it.
    pub moving_recovery: Option<f32>,
    pub passes: Passes,
}

impl Config {
    fn built_in(name: &'static str, settings: SettingsSpec, extended: ExtendedSpec) -> Self {
        Self {
            name,
            settings,
            extended,
            stick_gate: StickGate::Jolt,
            refresh: Refresh::Never,
            still: Still::Never,
            moving_recovery: None,
            passes: Passes::default(),
        }
    }

    /// CharacterVirtual with the game's settings, without stick to floor or walk stairs.
    pub fn bare() -> Self {
        Self::built_in("bare", SettingsSpec::SPEC_D1, ExtendedSpec::off())
    }

    /// Jolt's defaults for everything but the capsule, the slope limit and the padding.
    pub fn jolt_defaults() -> Self {
        Self::built_in(
            "jolt-defaults",
            SettingsSpec::JOLT_DEFAULTS,
            ExtendedSpec::jolt(),
        )
    }

    /// Spec D.1 on the built-ins alone.
    pub fn spec_d1() -> Self {
        Self::built_in("spec-d1", SettingsSpec::SPEC_D1, ExtendedSpec::spec_d1())
    }

    /// Spec D.1 with a contact refresh before every move.
    pub fn spec_d1_refresh() -> Self {
        Self {
            name: "spec-d1+refresh",
            refresh: Refresh::EveryTick,
            ..Self::spec_d1()
        }
    }

    /// The repository's reference near step (`tests/common/walker.rs`).
    pub fn walker() -> Self {
        Self {
            name: "walker",
            extended: ExtendedSpec {
                stick: 0.3,
                ..ExtendedSpec::off()
            },
            stick_gate: StickGate::Caller,
            refresh: Refresh::WhenMoved,
            passes: Passes {
                underground: true,
                autostep: true,
                ..Passes::default()
            },
            ..Self::bare()
        }
    }

    /// Every pass the game added, in spec D.2's order.
    pub fn spec_d2() -> Self {
        Self {
            name: "spec-d2",
            refresh: Refresh::EveryTick,
            still: Still::WhenDeep,
            passes: Passes {
                underground: true,
                q6: true,
                autostep: true,
                q5: true,
                q4: true,
            },
            ..Self::bare()
        }
    }

    /// `spec-d2` with Jolt's stick to floor (gated by the caller) instead of the Q5 snap.
    pub fn d2_stick() -> Self {
        let d2 = Self::spec_d2();
        Self {
            name: "d2-stick",
            extended: ExtendedSpec {
                stick: 0.3,
                ..d2.extended
            },
            stick_gate: StickGate::Caller,
            passes: Passes {
                q5: false,
                ..d2.passes
            },
            ..d2
        }
    }

    /// `d2-stick` refreshing only after a maintenance pass moved the character.
    pub fn d2_stick_norefresh() -> Self {
        Self {
            name: "d2-stick-norefresh",
            refresh: Refresh::WhenMoved,
            ..Self::d2_stick()
        }
    }

    /// `spec-d2` without the Q4 support cast: Jolt's ground state decides steepness.
    pub fn d2_noq4() -> Self {
        let d2 = Self::spec_d2();
        Self {
            name: "d2-noq4",
            passes: Passes {
                q4: false,
                ..d2.passes
            },
            ..d2
        }
    }

    /// `spec-d2` with Jolt's recovery in a still update instead of the Q6 push, and none in the
    /// move.
    pub fn d2_still() -> Self {
        let d2 = Self::spec_d2();
        Self {
            name: "d2-still",
            still: Still::WhenPenetrating,
            moving_recovery: Some(0.0),
            passes: Passes {
                q6: false,
                ..d2.passes
            },
            ..d2
        }
    }

    /// `spec-d2` with Jolt's recovery inside the move instead of the Q6 push.
    pub fn d2_inmove() -> Self {
        let d2 = Self::spec_d2();
        Self {
            name: "d2-inmove",
            still: Still::Never,
            passes: Passes {
                q6: false,
                ..d2.passes
            },
            ..d2
        }
    }

    /// `spec-d2` with Jolt's walk stairs instead of the autostep.
    pub fn d2_stairs() -> Self {
        let d2 = Self::spec_d2();
        Self {
            name: "d2-stairs",
            extended: ExtendedSpec {
                stairs_up: D2_STAIRS_STEP_UP,
                min_step_forward: 0.15,
                step_forward_test: D2_STAIRS_FORWARD_TEST,
                ..d2.extended
            },
            passes: Passes {
                autostep: false,
                ..d2.passes
            },
            ..d2
        }
    }

    /// `spec-d2` without the underground recovery.
    pub fn d2_noground() -> Self {
        let d2 = Self::spec_d2();
        Self {
            name: "d2-noground",
            passes: Passes {
                underground: false,
                ..d2.passes
            },
            ..d2
        }
    }

    /// `spec-d1` with a 50 degree slope limit: the engine control of law 1.
    pub fn max_slope_50() -> Self {
        Self {
            name: "max-slope-50",
            settings: SettingsSpec {
                max_slope_deg: 50.0,
                ..SettingsSpec::SPEC_D1
            },
            ..Self::spec_d1()
        }
    }

    /// The recommended configuration: the row with the most held law columns and, among those,
    /// the lowest median cost per move on the planet walk (see the study doc). It is `d2-noq4`.
    pub fn recommended() -> Self {
        Self {
            name: "recommended",
            ..Self::d2_noq4()
        }
    }

    /// The rows the law tests pin, in table order.
    pub fn pinned() -> Vec<Self> {
        vec![
            Self::bare(),
            Self::jolt_defaults(),
            Self::spec_d1(),
            Self::spec_d1_refresh(),
            Self::walker(),
            Self::spec_d2(),
            Self::d2_stick(),
            Self::d2_stick_norefresh(),
            Self::d2_noq4(),
            Self::d2_still(),
            Self::d2_inmove(),
            Self::d2_stairs(),
            Self::d2_noground(),
            Self::max_slope_50(),
            Self::recommended(),
        ]
    }

    /// The pinned row called `name`.
    pub fn named(name: &str) -> Self {
        Self::pinned()
            .into_iter()
            .chain(Self::survey_rows())
            .find(|config| config.name == name)
            .unwrap_or_else(|| panic!("no study row {name}"))
    }

    /// The survey's extra rows, not pinned.
    pub fn survey_rows() -> Vec<Self> {
        let d1 = Self::spec_d1();
        let refresh = Self::spec_d1_refresh();
        let mut rows = Vec::new();
        for (name, up, test) in [
            ("d1-up0.30-test0.15", 0.30, 0.15),
            ("d1-up0.30-test0.5", 0.30, 0.5),
            ("d1-up0.33-test0.15", 0.33, 0.15),
            ("d1-up0.33-test0.5", 0.33, 0.5),
            ("d1-up0.36-test0.15", 0.36, 0.15),
            ("d1-up0.36-test0.5", 0.36, 0.5),
            ("d1-up0.40-test0.15", 0.40, 0.15),
            ("d1-up0.40-test0.5", 0.40, 0.5),
        ] {
            rows.push(Self {
                name,
                extended: ExtendedSpec {
                    stairs_up: up,
                    step_forward_test: test,
                    ..d1.extended
                },
                ..d1
            });
        }
        for (name, tolerance) in [("d1r-tolerance0.01", 0.01), ("d1r-tolerance0.02", 0.02)] {
            rows.push(Self {
                name,
                settings: SettingsSpec {
                    collision_tolerance: tolerance,
                    ..d1.settings
                },
                ..refresh
            });
        }
        rows.push(Self {
            name: "d1r-stick0.5",
            extended: ExtendedSpec {
                stick: 0.5,
                ..d1.extended
            },
            ..refresh
        });
        for (name, distance) in [
            ("d1r-predictive0.025", 0.025),
            ("d1r-predictive0.05", 0.05),
            ("d1r-predictive0.2", 0.2),
        ] {
            rows.push(Self {
                name,
                settings: SettingsSpec {
                    predictive_contact_distance: distance,
                    ..d1.settings
                },
                ..refresh
            });
        }
        for (name, padding) in [("d1-padding0.01", 0.01), ("d1-padding0.04", 0.04)] {
            rows.push(Self {
                name,
                settings: SettingsSpec {
                    padding,
                    ..d1.settings
                },
                ..d1
            });
        }
        rows.push(Self {
            name: "d1-no-edge-removal",
            settings: SettingsSpec {
                enhanced_internal_edge_removal: false,
                ..d1.settings
            },
            ..d1
        });
        rows.push(Self {
            name: "d1-ignore-back-faces",
            settings: SettingsSpec {
                back_face_collision: false,
                ..d1.settings
            },
            ..d1
        });
        rows.push(Self {
            name: "d1-still-always",
            still: Still::Always,
            ..d1
        });
        rows.push(Self {
            name: "d1-still-penetrating",
            still: Still::WhenPenetrating,
            ..d1
        });
        rows.push(Self {
            name: "d1-moving-recovery0",
            moving_recovery: Some(0.0),
            ..d1
        });
        rows.push(Self {
            name: "d1-recovery0.5",
            settings: SettingsSpec {
                recovery_speed: 0.5,
                ..d1.settings
            },
            ..d1
        });
        rows.push(Self {
            name: "d1-inner-body",
            settings: SettingsSpec {
                inner_body: true,
                ..d1.settings
            },
            ..d1
        });
        rows
    }

    /// The configuration in words, for the study table.
    pub fn describe(&self) -> String {
        let mut parts = Vec::new();
        let s = &self.settings;
        let base = if s.supporting_plane && s.enhanced_internal_edge_removal {
            "D.1 settings"
        } else if !s.supporting_plane && !s.enhanced_internal_edge_removal {
            "Jolt default settings"
        } else {
            "mixed settings"
        };
        parts.push(base.to_owned());
        if s.max_slope_deg != 45.0 {
            parts.push(format!("max slope {}", s.max_slope_deg));
        }
        let e = &self.extended;
        if e.stick > 0.0 {
            let gate = match self.stick_gate {
                StickGate::Jolt => "",
                StickGate::Caller => " (caller-gated)",
            };
            parts.push(format!("stick {}{gate}", e.stick));
        }
        if e.stairs_up > 0.0 {
            parts.push(format!(
                "stairs {} fwd {} test {}",
                e.stairs_up, e.min_step_forward, e.step_forward_test
            ));
        }
        match self.refresh {
            Refresh::Never => {}
            Refresh::WhenMoved => parts.push("refresh when moved".to_owned()),
            Refresh::EveryTick => parts.push("refresh every tick".to_owned()),
        }
        match self.still {
            Still::Never => {}
            Still::Always => parts.push("still update always".to_owned()),
            Still::WhenPenetrating => parts.push("still update when penetrating".to_owned()),
            Still::WhenDeep => parts.push("still update when deep".to_owned()),
        }
        if let Some(speed) = self.moving_recovery {
            parts.push(format!("recovery {speed} in the move"));
        }
        let p = &self.passes;
        for (on, name) in [
            (p.underground, "Q3 underground"),
            (p.q6, "Q6 push"),
            (p.autostep, "autostep"),
            (p.q5, "Q5 snap"),
            (p.q4, "Q4 support"),
        ] {
            if on {
                parts.push(name.to_owned());
            }
        }
        parts.join(", ")
    }

    /// Creates this configuration's character with body origin `origin` in `scene`, refreshes
    /// its contacts once (a new character knows no ground), puts the scene's actor capsule
    /// there and returns its handle. The startup ground state is the character's own, not
    /// assumed.
    pub fn create_character(&self, scene: &mut Scene, origin: V3) -> Walker {
        let walker = self.create_character_with_actor(
            &mut scene.world,
            scene.layers,
            scene.up,
            scene.actor,
            origin,
        );
        scene.place_actor(origin, from_y_to(scene.up.up_at(origin)));
        walker
    }

    /// As [`create_character`](Self::create_character) in `world`, for the actor capsule
    /// `actor`, which is not moved.
    pub fn create_character_with_actor(
        &self,
        world: &mut PhysicsWorld,
        layers: Layers,
        up_policy: UpPolicy,
        actor: BodyId,
        origin: V3,
    ) -> Walker {
        let up = up_policy.up_at(origin);
        let shape = capsule();
        let inner = self.settings.inner_body.then_some(InnerBody {
            shape: &shape,
            object_layer: layers.actor,
        });
        let settings = self.settings.character_settings(&shape, inner, up);
        let id = world
            .create_character(
                &settings,
                position_for(origin, up, self.settings.padding),
                from_y_to(up),
            )
            .unwrap();
        let walker = Walker {
            id,
            actor,
            layers,
            step_height: STEP_HEIGHT,
            items_in_filter: false,
        };
        let filter_layers = [layers.terrain, layers.chunk, layers.actor];
        world
            .refresh_character_contacts(id, &controller_filter(&walker, &filter_layers))
            .unwrap();
        walker
    }
}

/// The step-up of `d2-stairs`, chosen from the survey's walk-stairs rows (see the study doc).
pub const D2_STAIRS_STEP_UP: f32 = 0.33;
/// The step forward test of `d2-stairs`.
pub const D2_STAIRS_FORWARD_TEST: f32 = 0.5;
