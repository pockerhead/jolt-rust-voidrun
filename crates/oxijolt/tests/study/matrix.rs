//! The configuration x law matrix: the cases of each column, their outcomes under each start
//! perturbation, and the cell rule.
//!
//! A case is stable when all five perturbations give the same pass or fail. A cell is held when
//! every case passes under every perturbation, broken when at least one stable case fails (the
//! first in case-id order is its witness), and variable otherwise.

use super::config::Config;
use super::expected::Cell;
use super::laws::{self, evaluate, Case, Column, Outcome, Scenes, PERTURBATIONS};
use super::scenes::Split;

/// The cases of `column`, sorted by id.
pub fn cases(scenes: &mut Scenes, column: Column) -> Vec<Case> {
    let mut cases = match column {
        Column::Slope => laws::slope::cases(scenes, false),
        Column::Floor => laws::floor::cases(scenes),
        Column::StepSharp => laws::step::cases(scenes, false),
        Column::StepRounded => laws::step::cases(scenes, true),
        Column::Hops => laws::hops::cases(scenes),
        Column::PushVelocity => laws::pushout::velocity_cases(scenes),
        Column::PushRecovery => laws::pushout::recovery_cases(scenes),
        Column::Seam => laws::seam::cases(scenes, Split::Pair),
        Column::Radial => laws::radial::cases(scenes),
    };
    cases.sort_by(|a, b| a.id.cmp(&b.id));
    cases
}

/// The report-only cases: exactly 45 degrees, the 0.5 m ledge and the raised seam control.
pub fn report_cases(scenes: &mut Scenes) -> Vec<Case> {
    let mut cases = laws::slope::cases(scenes, true);
    cases.extend(
        laws::floor::cases(scenes)
            .into_iter()
            .filter(|case| case.report_only),
    );
    cases.extend(laws::seam::cases(scenes, Split::Raised));
    cases
}

/// One case's outcomes, one per perturbation played.
#[derive(Clone, Debug)]
pub struct CaseOutcomes {
    pub id: String,
    pub outcomes: Vec<Outcome>,
}

impl CaseOutcomes {
    pub fn stable(&self) -> bool {
        let first = self.outcomes[0].is_pass();
        self.outcomes.iter().all(|o| o.is_pass() == first)
    }

    pub fn invalid(&self) -> Option<&Outcome> {
        self.outcomes
            .iter()
            .find(|o| matches!(o, Outcome::Invalid { .. }))
    }
}

/// Plays every case of `cases` under `config` with the first `perturbations` start offsets.
pub fn outcomes(
    scenes: &mut Scenes,
    config: &Config,
    cases: &[Case],
    perturbations: usize,
) -> Vec<CaseOutcomes> {
    cases
        .iter()
        .filter(|case| !case.report_only)
        .map(|case| CaseOutcomes {
            id: case.id.clone(),
            outcomes: PERTURBATIONS[..perturbations]
                .iter()
                .map(|&offset| evaluate(scenes, config, case, offset))
                .collect(),
        })
        .collect()
}

/// The measured cell of a column from its case outcomes (all played with the same
/// perturbations), with the witness id kept as an owned string.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Measured {
    Held,
    Broken(String),
    Variable,
}

pub fn cell(results: &[CaseOutcomes]) -> Measured {
    if results
        .iter()
        .all(|r| r.outcomes.iter().all(Outcome::is_pass))
    {
        return Measured::Held;
    }
    match results
        .iter()
        .find(|r| r.stable() && !r.outcomes[0].is_pass())
    {
        Some(witness) => Measured::Broken(witness.id.clone()),
        None => Measured::Variable,
    }
}

/// Whether `measured` agrees with the pinned `cell` under the law tests' rule for one
/// perturbation: held means every case passes; broken means the witness fails; variable
/// accepts any pass or fail.
pub fn agrees(results: &[CaseOutcomes], cell: Cell) -> Result<(), String> {
    if let Some(bad) = results.iter().find_map(|r| r.invalid().map(|o| (r, o))) {
        return Err(format!("{}: {:?}", bad.0.id, bad.1));
    }
    match cell {
        Cell::Held => match results
            .iter()
            .find(|r| !r.outcomes.iter().all(Outcome::is_pass))
        {
            None => Ok(()),
            Some(failed) => Err(format!("held, but {}: {:?}", failed.id, failed.outcomes)),
        },
        Cell::Broken(witness) => match results.iter().find(|r| r.id == witness) {
            Some(r) if r.outcomes.iter().all(|o| !o.is_pass()) => Ok(()),
            Some(r) => Err(format!("broken by {witness}, but it gave {:?}", r.outcomes)),
            None => Err(format!("broken by {witness}, which is not a case")),
        },
        Cell::Variable => Ok(()),
    }
}

/// A one-line summary of a column's results: counts and the first failure.
pub fn summary(results: &[CaseOutcomes]) -> String {
    let total: usize = results.iter().map(|r| r.outcomes.len()).sum();
    let passed: usize = results
        .iter()
        .map(|r| r.outcomes.iter().filter(|o| o.is_pass()).count())
        .sum();
    let unstable = results.iter().filter(|r| !r.stable()).count();
    let first = results
        .iter()
        .find(|r| !r.outcomes.iter().all(Outcome::is_pass))
        .map(|r| {
            let failure = r.outcomes.iter().find(|o| !o.is_pass()).unwrap();
            format!("; first failure {} {:?}", r.id, failure)
        })
        .unwrap_or_default();
    format!("{passed}/{total} pass, {unstable} unstable cases{first}")
}
