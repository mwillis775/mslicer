//! Support generation test harness.
//!
//! Generates supports for a model, slices the model with its supports and
//! checks the result for islands (regions that would print in mid-air). Any
//! remaining islands are places the supports failed to hold up.
//!
//! Usage: supports [--style <standard|tree|lattice|mini|needle>] [--orient] [--gap-fill]
//! <model.stl> [output.stl]

use std::{
    env,
    fs::File,
    io::{BufReader, BufWriter},
    time::Instant,
};

use anyhow::{Context, Result};
use common::{
    progress::Progress,
    serde::{ReaderDeserializer, WriterSerializer},
    slice::SliceConfig,
    units::{Milimeter, Milimeters},
};
use mesh_format::Format;
use nalgebra::{Vector2, Vector3};
use slicer::{
    geometry::bvh::Bvh,
    mesh::Mesh,
    post_process::island_detection::detect_islands,
    slicer::{Slicer, SlicerModel},
};
use tools::supports::{
    bracing::brace,
    collider::Collider,
    generate::generate,
    mesh::SupportMesh,
    orient::orient,
    settings::{SupportSettings, SupportStyle},
};

/// Resolution used when checking the sliced result for islands.
const CHECK_PIXEL: f32 = 0.05;

fn main() -> Result<()> {
    let mut args = env::args().skip(1).peekable();
    let mut style = SupportStyle::Standard;
    let (mut auto_orient, mut gap_fill) = (false, false);
    while let Some(flag) = args.next_if(|x| x.starts_with("--")) {
        match flag.as_str() {
            "--style" => {
                let name = args.next().context("No style supplied")?;
                style = (SupportStyle::ALL.into_iter())
                    .find(|x| format!("{x:?}").eq_ignore_ascii_case(&name))
                    .context("Unknown style")?;
            }
            "--orient" => auto_orient = true,
            "--gap-fill" => gap_fill = true,
            _ => anyhow::bail!("Unknown flag {flag}"),
        }
    }
    let path = args.next().context("No input path supplied")?;
    let output = args.next();

    let file = File::open(&path)?;
    let des = ReaderDeserializer::new(BufReader::new(file));
    let raw = mesh_format::load_mesh(des, Format::Stl, &Progress::new())?;
    let mut mesh = Mesh::new_boxed(raw.verts, raw.faces);

    let mut settings = SupportSettings::for_style(style);
    if let Ok(lift) = env::var("LIFT") {
        settings.placement.lift = lift.parse()?;
    }
    settings.gap_fill.enabled = gap_fill;
    println!("Style: {}, gap fill: {gap_fill}", style.name());

    let start = Instant::now();
    let bvh = Bvh::build(mesh.inner(), Progress::new());
    let bvh_time = start.elapsed();

    if auto_orient {
        let start = Instant::now();
        let orientation = orient(
            &mesh,
            &bvh,
            &settings,
            Vector3::repeat(f32::MAX),
            Progress::new(),
        );
        mesh.set_rotation(orientation.rotation);
        println!(
            "Oriented to {:.1?}° (cost {:.1}) in {:?}",
            orientation.rotation.map(f32::to_degrees),
            orientation.cost,
            start.elapsed()
        );
    }

    let (min, _) = mesh.bounds();
    mesh.set_position(mesh.position() + Vector3::z() * (settings.placement.lift - min.z));
    let (min, max) = mesh.bounds();
    println!(
        "Loaded {path}: {} faces, {:.1}×{:.1}×{:.1} mm",
        mesh.face_count(),
        max.x - min.x,
        max.y - min.y,
        max.z - min.z
    );

    let collider = Collider::new(&mesh, &bvh);

    let start = Instant::now();
    let generated = generate(&collider, &settings, 0.0, Progress::new());
    let generate_time = start.elapsed();

    let start = Instant::now();
    let braces = brace(&collider, &generated.supports, &settings, 0.0);
    let brace_time = start.elapsed();

    let start = Instant::now();
    let (support_mesh, _) = SupportMesh::build(&settings, 0.0, &generated.supports, &braces)
        .context("No supports generated")?;
    let mesh_time = start.elapsed();

    let leaning = (generated.supports.iter())
        .filter(|x| x.route.path.len() > 1)
        .count();
    let fills = (generated.supports.iter())
        .filter(|x| x.style == SupportStyle::Fill)
        .count();
    let on_model = (generated.supports.iter())
        .filter(|x| matches!(x.route.foot, tools::supports::Foot::Model(_)))
        .count();
    let columns = (generated.supports.iter())
        .map(|x| x.column_top().map(f32::to_bits))
        .collect::<std::collections::HashSet<_>>()
        .len();
    println!(
        "Generated {} supports ({leaning} leaning, {on_model} on model, {columns} columns, {fills} gap fill), {} braces, {} failed",
        generated.supports.len(),
        braces.len(),
        generated.failed.len()
    );
    let mut reasons = std::collections::HashMap::new();
    for (_, err) in generated.failed.iter() {
        *reasons.entry(format!("{err:?}")).or_insert(0) += 1;
    }
    println!("Failure reasons: {reasons:?}");
    let start = Instant::now();
    let candidates = tools::supports::placement::place(&collider, &settings, 0.0);
    println!(
        "Placement alone: {:?} ({} candidates)",
        start.elapsed(),
        candidates.len()
    );
    for (point, err) in generated.failed.iter().take(10) {
        println!(
            "  failed: ({:.2}, {:.2}, {:.2}) {err:?}",
            point.x, point.y, point.z
        );
    }
    println!(
        "Timing: bvh {bvh_time:?}, generate {generate_time:?}, brace {brace_time:?}, mesh {mesh_time:?} ({} faces)",
        support_mesh.face_count()
    );

    // Lists the supports near a point, for looking into islands.
    if let Ok(at) = env::var("DEBUG_AT") {
        let at = at
            .split(',')
            .map(|x| x.parse::<f32>())
            .collect::<Result<Vec<_>, _>>()?;
        let at = Vector2::new(at[0], at[1]);
        for x in generated.supports.iter() {
            let near = (x.route.path.iter().chain([&x.contact.point]))
                .any(|p| (p.xy() - at).magnitude() < 3.0);
            near.then(|| {
                println!(
                    "  {:?} {:?}\n    contact {:?}\n    path {:?}\n    radii {:?}\n    foot {:?}",
                    x.style, x.size, x.contact.point, x.route.path, x.route.radii, x.route.foot
                )
            });
        }
    }

    let before = island_check(&[&mesh], (min, max), false)?;
    let after = island_check(&[&mesh, &support_mesh], (min, max), true)?;
    println!("Islands without supports: {before}");
    println!("Islands with supports:    {after}");

    if let Some(output) = output {
        save_combined(&output, &[&mesh, &support_mesh])?;
        save_combined(&format!("{output}.model.stl"), &[&mesh])?;
        save_combined(&format!("{output}.supports.stl"), &[&support_mesh])?;
        let fills = (generated.supports.iter())
            .filter(|x| x.style == SupportStyle::Fill)
            .cloned()
            .collect::<Vec<_>>();
        if let Some((fill_mesh, _)) = SupportMesh::build(&settings, 0.0, &fills, &[]) {
            save_combined(&format!("{output}.fills.stl"), &[&fill_mesh])?;
        }
        println!("Wrote {output}");
    }

    Ok(())
}

struct IslandReport {
    layers: usize,
    area: f32,
    lowest: Option<f32>,
}

impl std::fmt::Display for IslandReport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} layers, {:.3} mm²", self.layers, self.area)?;
        if let Some(z) = self.lowest {
            write!(f, ", first at z={z:.2}")?;
        }
        Ok(())
    }
}

fn island_check(
    meshes: &[&Mesh],
    (min, max): (Vector3<f32>, Vector3<f32>),
    verbose: bool,
) -> Result<IslandReport> {
    let margin = 30.0;
    let size = (max.xy() - min.xy()).map(|x| x + margin * 2.0);
    let center = (max.xy() + min.xy()) / 2.0;
    let resolution = (size / CHECK_PIXEL).map(|x| x.ceil() as u32);

    let config = SliceConfig {
        platform_resolution: resolution,
        platform_size: Vector3::new(size.x, size.y, max.z + 1.0).map(Milimeters::new),
        ..Default::default()
    };

    let slice_height = config.slice_height.get::<Milimeter>();
    let mm_to_px = resolution.cast::<f32>().component_div(&size).push(1.0);
    let offset = (resolution.cast::<f32>() / 2.0 - center.component_mul(&mm_to_px.xy()))
        .push(-slice_height / 2.0);

    let models = (meshes.iter())
        .map(|mesh| {
            let mut mesh = (*mesh).clone();
            mesh.set_scale_unchecked(mesh.scale().component_mul(&mm_to_px));
            mesh.set_position_unchecked(mesh.position().component_mul(&mm_to_px) + offset);
            mesh.update_transformation_matrix();
            SlicerModel {
                mesh,
                exposure: 255,
            }
        })
        .collect();

    let slicer = Slicer::new(config, models);
    let layers = slicer.slice_raster();
    let islands = detect_islands(resolution, &layers, Progress::new(), false);

    let px_area = CHECK_PIXEL * CHECK_PIXEL;
    let mut report = IslandReport {
        layers: 0,
        area: 0.0,
        lowest: None,
    };
    for (i, runs) in islands.iter().enumerate() {
        let pixels = runs.iter().skip(1).step_by(2).sum::<u64>();
        if pixels > 0 && verbose {
            let first = runs[0];
            let (x, y) = (first % resolution.x as u64, first / resolution.x as u64);
            let pos = (Vector2::new(x as f32, y as f32) - resolution.cast::<f32>() / 2.0)
                .component_div(&mm_to_px.xy())
                + center;
            println!(
                "  island at ({:.2}, {:.2}, {:.2}): {:.3} mm²",
                pos.x,
                pos.y,
                (i + 2) as f32 * slice_height,
                pixels as f32 * px_area
            );
        }
        if pixels > 0 {
            report.layers += 1;
            report.area += pixels as f32 * px_area;
            // Islands are reported starting from the second layer.
            report.lowest.get_or_insert((i + 2) as f32 * slice_height);
        }
    }

    Ok(report)
}

fn save_combined(path: &str, meshes: &[&Mesh]) -> Result<()> {
    let mut verts = Vec::new();
    let mut faces = Vec::new();
    for mesh in meshes {
        let offset = verts.len() as u32;
        verts.extend(mesh.vertices().iter().map(|x| mesh.transform(x)));
        faces.extend(mesh.faces().iter().map(|f| f.map(|x| x + offset)));
    }

    let combined = mesh_format::Mesh {
        verts: verts.into_boxed_slice(),
        faces: faces.into_boxed_slice(),
    };
    let mut ser = WriterSerializer::new(BufWriter::new(File::create(path)?));
    mesh_format::save_mesh(&mut ser, Format::Stl, &Progress::new(), &combined);
    Ok(())
}
