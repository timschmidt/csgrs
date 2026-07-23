use std::str::FromStr;
use std::sync::{Arc, Mutex};

use eframe::glow::HasContext as _;
use egui::{Align2, CentralPanel, Color32, ComboBox, FontId, Sense, SidePanel, TopBottomPanel};
use hypergraphics::backend::{GpuColoredMesh, UnlitProgram};
use hypergraphics::{
    Color3, ExactCamera, ExactMesh, ExactVertex, Primitive, Projection64, RenderVertex64, Viewport,
    axes_mesh, grid_mesh,
};
use hypermesh::{
    BooleanOp, EmberConfig, InputMesh, MeshRef, OutputVertex, Point3, Real, Triangle, TriangleSoup,
    boolean_operation, triangulate_and_resolve_certified,
};
use web_time::{Duration, Instant};

pub struct MainApp {
    cube_a: InputMesh,
    cube_b: InputMesh,
    operation: DemoOperation,
    show_cube_a: bool,
    show_cube_b: bool,
    show_wireframe: bool,
    offset_quarters: i32,
    spin: f32,
    last_frame: Instant,
    result: Option<TriangleSoup>,
    render_scene: RenderScene,
    render_resources: Arc<Mutex<Option<RenderResources>>>,
    stats: DemoStats,
}

impl MainApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        cc.egui_ctx.style_mut(|style| {
            for font_id in style.text_styles.values_mut() {
                font_id.size += 1.0;
            }
        });

        let cube_a = cube_mesh(-1, 1);
        let cube_b = shifted_cube_mesh(1);
        let mut app = Self {
            cube_a,
            cube_b,
            operation: DemoOperation::Union,
            show_cube_a: true,
            show_cube_b: true,
            show_wireframe: true,
            offset_quarters: 4,
            spin: 0.0,
            last_frame: Instant::now(),
            result: None,
            render_scene: RenderScene::empty(),
            render_resources: Arc::new(Mutex::new(None)),
            stats: DemoStats::default(),
        };
        app.recompute();
        app
    }

    fn recompute(&mut self) {
        let started = Instant::now();
        self.cube_b = shifted_cube_mesh(self.offset_quarters);

        let refs = match self.operation {
            DemoOperation::Union => vec![self.cube_a.as_ref(), self.cube_b.as_ref()],
            DemoOperation::Intersection => vec![self.cube_a.as_ref(), self.cube_b.as_ref()],
            DemoOperation::CubeAMinusB => vec![self.cube_a.as_ref(), self.cube_b.as_ref()],
            DemoOperation::CubeBMinusA => vec![self.cube_b.as_ref(), self.cube_a.as_ref()],
            DemoOperation::SymmetricDifference => {
                vec![self.cube_a.as_ref(), self.cube_b.as_ref()]
            }
        };
        let op = match self.operation {
            DemoOperation::Union => BooleanOp::Union,
            DemoOperation::Intersection => BooleanOp::Intersection,
            DemoOperation::CubeAMinusB | DemoOperation::CubeBMinusA => BooleanOp::Difference,
            DemoOperation::SymmetricDifference => BooleanOp::SymmetricDifference,
        };

        let config = EmberConfig { max_depth: 8 };

        match run_boolean(&refs, op, config) {
            Ok(result) => {
                self.stats = DemoStats::ok(started.elapsed(), &self.cube_a, &self.cube_b, &result);
                self.result = Some(result);
            }
            Err(error) => {
                self.stats = DemoStats::failed(started.elapsed(), error.to_string());
                self.result = None;
            }
        }
        self.render_scene =
            RenderScene::from_demo(&self.cube_a, &self.cube_b, self.result.as_ref());
    }

    fn camera(&self) -> ExactCamera {
        let mut camera = ExactCamera::default();
        camera.yaw = Real::try_from(self.spin as f64).unwrap_or_else(|_| Real::zero());
        camera.pitch = Real::try_from(-0.65_f64).expect("finite camera pitch");
        camera.zoom = Real::from(6);
        camera.target = point(real_ratio(1, 2), Real::zero(), Real::zero());
        camera
    }
}

impl eframe::App for MainApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let now = Instant::now();
        let dt = now.duration_since(self.last_frame).as_secs_f32().min(0.05);
        self.last_frame = now;
        self.spin = (self.spin + dt * 0.35) % std::f32::consts::TAU;
        ctx.request_repaint();

        TopBottomPanel::top("top").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.heading("hypermesh exact CSG");
                ui.separator();
                ui.label("Exact booleans over hyperreal coordinates");
            });
        });

        SidePanel::right("controls")
            .default_width(292.0)
            .show(ctx, |ui| {
                ui.heading("Boolean");
                let mut changed = false;
                ComboBox::from_label("Operation")
                    .selected_text(self.operation.label())
                    .show_ui(ui, |ui| {
                        for operation in DemoOperation::ALL {
                            changed |= ui
                                .selectable_value(&mut self.operation, operation, operation.label())
                                .changed();
                        }
                    });
                changed |= ui
                    .add(
                        egui::Slider::new(&mut self.offset_quarters, -2..=8)
                            .text("Cube B x offset")
                            .custom_formatter(|value, _| format!("{:.2}", value / 4.0)),
                    )
                    .changed();
                if ui.button("Run exact boolean").clicked() {
                    changed = true;
                }
                if changed {
                    self.recompute();
                }

                ui.separator();
                ui.heading("Display");
                ui.checkbox(&mut self.show_cube_a, "Cube A");
                ui.checkbox(&mut self.show_cube_b, "Cube B");
                ui.checkbox(&mut self.show_wireframe, "Wireframe");

                ui.separator();
                ui.heading("Mesh");
                ui.label(format!("Cube A triangles: {}", self.stats.cube_a_triangles));
                ui.label(format!("Cube B triangles: {}", self.stats.cube_b_triangles));
                ui.label(format!("Result triangles: {}", self.stats.result_triangles));
                ui.label(format!("Result vertices: {}", self.stats.result_vertices));
                ui.label(format!(
                    "Solve: {:.2} ms",
                    self.stats.elapsed.as_secs_f64() * 1000.0
                ));
                if let Some(error) = &self.stats.error {
                    ui.separator();
                    ui.colored_label(Color32::from_rgb(255, 122, 122), error);
                }
            });

        CentralPanel::default().show(ctx, |ui| {
            let (rect, _) = ui.allocate_exact_size(ui.available_size(), Sense::hover());
            let painter = ui.painter_at(rect);
            painter.rect_filled(rect, 0.0, Color32::from_rgb(17, 20, 24));
            let render_frame = self.render_scene.render_frame().unwrap_or_else(|error| {
                log::warn!("hypergraphics export failed: {error}");
                RenderFrame::empty()
            });
            let projection = self
                .camera()
                .projection64(
                    Viewport::new(0.0, 0.0, f64::from(rect.width()), f64::from(rect.height()))
                        .expect("finite egui viewport"),
                )
                .ok();
            let show_cube_a = self.show_cube_a;
            let show_cube_b = self.show_cube_b;
            let show_wireframe = self.show_wireframe;
            let resources = Arc::clone(&self.render_resources);
            painter.add(egui::PaintCallback {
                rect,
                callback: Arc::new(eframe::egui_glow::CallbackFn::new(move |info, painter| {
                    let gl = painter.gl();
                    if let Err(error) = render_hypergraphics(
                        gl,
                        info,
                        &resources,
                        &render_frame,
                        projection.as_ref(),
                        show_cube_a,
                        show_cube_b,
                        show_wireframe,
                    ) {
                        log::warn!("hypergraphics render failed: {error}");
                    }
                })),
            });

            if self.result.is_none() {
                painter.text(
                    rect.center(),
                    Align2::CENTER_CENTER,
                    "No output for this operation",
                    FontId::proportional(18.0),
                    Color32::from_rgb(255, 178, 178),
                );
            }
        });
    }
}

#[derive(Clone, Debug)]
struct RenderScene {
    grid: ExactMesh,
    axes: ExactMesh,
    input_a_faces: ExactMesh,
    input_b_faces: ExactMesh,
    input_a_wire: ExactMesh,
    input_b_wire: ExactMesh,
    result_faces: ExactMesh,
    result_wire: ExactMesh,
}

impl RenderScene {
    fn empty() -> Self {
        let grid_color = Color3::new(0.18, 0.21, 0.25).expect("finite grid color");
        Self {
            grid: grid_mesh(10, real_ratio(1, 2), grid_color),
            axes: axes_mesh(Real::from(3), Real::zero()).expect("valid axes mesh"),
            input_a_faces: ExactMesh::empty(Primitive::Triangles),
            input_b_faces: ExactMesh::empty(Primitive::Triangles),
            input_a_wire: ExactMesh::empty(Primitive::Lines),
            input_b_wire: ExactMesh::empty(Primitive::Lines),
            result_faces: ExactMesh::empty(Primitive::Triangles),
            result_wire: ExactMesh::empty(Primitive::Lines),
        }
    }

    fn from_demo(cube_a: &InputMesh, cube_b: &InputMesh, result: Option<&TriangleSoup>) -> Self {
        let blue = Color3::new(0.31, 0.67, 1.0).expect("finite input color");
        let red = Color3::new(1.0, 0.47, 0.40).expect("finite input color");
        let green = Color3::new(0.41, 0.86, 0.60).expect("finite result color");
        let pale_green = Color3::new(0.82, 0.98, 0.87).expect("finite wire color");
        let mut scene = Self::empty();
        scene.input_a_faces = input_mesh_faces(cube_a, blue);
        scene.input_b_faces = input_mesh_faces(cube_b, red);
        scene.input_a_wire = input_mesh_wire(cube_a, blue);
        scene.input_b_wire = input_mesh_wire(cube_b, red);
        if let Some(result) = result {
            scene.result_faces = triangle_soup_faces(result, green);
            scene.result_wire = triangle_soup_wire(result, pale_green);
        }
        scene
    }

    fn render_frame(&self) -> hypergraphics::Result<RenderFrame> {
        Ok(RenderFrame {
            grid: self.grid.to_render_vertices64()?,
            axes: self.axes.to_render_vertices64()?,
            input_a_faces: self.input_a_faces.to_render_vertices64()?,
            input_b_faces: self.input_b_faces.to_render_vertices64()?,
            input_a_wire: self.input_a_wire.to_render_vertices64()?,
            input_b_wire: self.input_b_wire.to_render_vertices64()?,
            result_faces: self.result_faces.to_render_vertices64()?,
            result_wire: self.result_wire.to_render_vertices64()?,
        })
    }
}

#[derive(Clone, Debug, Default)]
struct RenderFrame {
    grid: Vec<RenderVertex64>,
    axes: Vec<RenderVertex64>,
    input_a_faces: Vec<RenderVertex64>,
    input_b_faces: Vec<RenderVertex64>,
    input_a_wire: Vec<RenderVertex64>,
    input_b_wire: Vec<RenderVertex64>,
    result_faces: Vec<RenderVertex64>,
    result_wire: Vec<RenderVertex64>,
}

impl RenderFrame {
    fn empty() -> Self {
        Self::default()
    }
}

struct RenderResources {
    program: UnlitProgram,
    grid: GpuColoredMesh,
    axes: GpuColoredMesh,
    input_a_faces: GpuColoredMesh,
    input_b_faces: GpuColoredMesh,
    input_a_wire: GpuColoredMesh,
    input_b_wire: GpuColoredMesh,
    result_faces: GpuColoredMesh,
    result_wire: GpuColoredMesh,
}

const RESULT_FACE_ALPHA: f32 = 0.92;

impl RenderResources {
    unsafe fn new(gl: &eframe::glow::Context) -> hypergraphics::Result<Self> {
        unsafe {
            Ok(Self {
                program: UnlitProgram::new(gl)?,
                grid: GpuColoredMesh::new(gl, Primitive::Lines)?,
                axes: GpuColoredMesh::new(gl, Primitive::Lines)?,
                input_a_faces: GpuColoredMesh::new(gl, Primitive::Triangles)?,
                input_b_faces: GpuColoredMesh::new(gl, Primitive::Triangles)?,
                input_a_wire: GpuColoredMesh::new(gl, Primitive::Lines)?,
                input_b_wire: GpuColoredMesh::new(gl, Primitive::Lines)?,
                result_faces: GpuColoredMesh::new(gl, Primitive::Triangles)?,
                result_wire: GpuColoredMesh::new(gl, Primitive::Lines)?,
            })
        }
    }

    unsafe fn upload(
        &mut self,
        gl: &eframe::glow::Context,
        frame: &RenderFrame,
    ) -> hypergraphics::Result<()> {
        unsafe {
            self.grid.upload_render_vertices64(gl, &frame.grid)?;
            self.axes.upload_render_vertices64(gl, &frame.axes)?;
            self.input_a_faces
                .upload_render_vertices64(gl, &frame.input_a_faces)?;
            self.input_b_faces
                .upload_render_vertices64(gl, &frame.input_b_faces)?;
            self.input_a_wire
                .upload_render_vertices64(gl, &frame.input_a_wire)?;
            self.input_b_wire
                .upload_render_vertices64(gl, &frame.input_b_wire)?;
            self.result_faces
                .upload_render_vertices64(gl, &frame.result_faces)?;
            self.result_wire
                .upload_render_vertices64(gl, &frame.result_wire)?;
        }
        Ok(())
    }

    unsafe fn draw(
        &self,
        gl: &eframe::glow::Context,
        projection: &hypergraphics::Projection64,
        show_cube_a: bool,
        show_cube_b: bool,
        show_wireframe: bool,
    ) -> hypergraphics::Result<()> {
        unsafe {
            self.program.bind(gl, projection)?;
            draw_mesh(gl, &self.program, &self.grid, 1.0)?;
            draw_mesh(gl, &self.program, &self.axes, 1.0)?;
            if show_cube_a || show_cube_b {
                gl.enable(eframe::glow::BLEND);
                gl.blend_func(eframe::glow::SRC_ALPHA, eframe::glow::ONE_MINUS_SRC_ALPHA);
            }
            if show_cube_a {
                draw_mesh(gl, &self.program, &self.input_a_faces, 0.24)?;
                draw_mesh(gl, &self.program, &self.input_a_wire, 1.0)?;
            }
            if show_cube_b {
                draw_mesh(gl, &self.program, &self.input_b_faces, 0.24)?;
                draw_mesh(gl, &self.program, &self.input_b_wire, 1.0)?;
            }
            draw_depth_prepassed_mesh(
                gl,
                &self.program,
                &self.result_faces,
                RESULT_FACE_ALPHA,
            )?;
            if show_wireframe {
                draw_mesh(gl, &self.program, &self.result_wire, 1.0)?;
            }
        }
        Ok(())
    }
}

fn render_hypergraphics(
    gl: &eframe::glow::Context,
    _info: egui::PaintCallbackInfo,
    resources: &Arc<Mutex<Option<RenderResources>>>,
    frame: &RenderFrame,
    projection: Option<&Projection64>,
    show_cube_a: bool,
    show_cube_b: bool,
    show_wireframe: bool,
) -> hypergraphics::Result<()> {
    let Some(projection) = projection else {
        return Ok(());
    };
    let mut guard = resources.lock().expect("render resources mutex poisoned");
    if guard.is_none() {
        *guard = Some(unsafe { RenderResources::new(gl)? });
    }
    let resources = guard.as_mut().expect("render resources should exist");
    unsafe {
        gl.enable(eframe::glow::DEPTH_TEST);
        gl.depth_func(eframe::glow::LEQUAL);
        gl.clear_color(17.0 / 255.0, 20.0 / 255.0, 24.0 / 255.0, 1.0);
        gl.clear(eframe::glow::COLOR_BUFFER_BIT | eframe::glow::DEPTH_BUFFER_BIT);
        gl.enable(eframe::glow::POLYGON_OFFSET_FILL);
        gl.polygon_offset(1.0, 1.0);
        resources.upload(gl, frame)?;
        resources.draw(gl, projection, show_cube_a, show_cube_b, show_wireframe)?;
        gl.disable(eframe::glow::POLYGON_OFFSET_FILL);
        gl.disable(eframe::glow::DEPTH_TEST);
        gl.disable(eframe::glow::BLEND);
    }
    Ok(())
}

unsafe fn draw_depth_prepassed_mesh(
    gl: &eframe::glow::Context,
    program: &UnlitProgram,
    mesh: &GpuColoredMesh,
    alpha: f32,
) -> hypergraphics::Result<()> {
    unsafe {
        // Blending translucent triangles in mesh order accumulates different
        // layers at different pixels. Populate depth first, then shade only
        // the nearest result surface exactly once.
        gl.color_mask(false, false, false, false);
        gl.depth_mask(true);
        gl.depth_func(eframe::glow::LEQUAL);
        mesh.draw(gl);

        gl.color_mask(true, true, true, true);
        gl.depth_mask(false);
        gl.depth_func(eframe::glow::EQUAL);
        let result = draw_mesh(gl, program, mesh, alpha);
        gl.depth_mask(true);
        gl.depth_func(eframe::glow::LEQUAL);
        result
    }
}

unsafe fn draw_mesh(
    gl: &eframe::glow::Context,
    program: &UnlitProgram,
    mesh: &GpuColoredMesh,
    alpha: f32,
) -> hypergraphics::Result<()> {
    unsafe {
        program.set_alpha(gl, alpha)?;
        mesh.draw(gl);
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum DemoOperation {
    Union,
    Intersection,
    CubeAMinusB,
    CubeBMinusA,
    SymmetricDifference,
}

impl DemoOperation {
    const ALL: [Self; 5] = [
        Self::Union,
        Self::Intersection,
        Self::CubeAMinusB,
        Self::CubeBMinusA,
        Self::SymmetricDifference,
    ];

    const fn label(self) -> &'static str {
        match self {
            Self::Union => "Union",
            Self::Intersection => "Intersection",
            Self::CubeAMinusB => "Cube A - Cube B",
            Self::CubeBMinusA => "Cube B - Cube A",
            Self::SymmetricDifference => "Symmetric difference",
        }
    }
}

#[derive(Clone, Debug)]
struct DemoStats {
    elapsed: Duration,
    cube_a_triangles: usize,
    cube_b_triangles: usize,
    result_triangles: usize,
    result_vertices: usize,
    error: Option<String>,
}

impl Default for DemoStats {
    fn default() -> Self {
        Self {
            elapsed: Duration::ZERO,
            cube_a_triangles: 0,
            cube_b_triangles: 0,
            result_triangles: 0,
            result_vertices: 0,
            error: None,
        }
    }
}

impl DemoStats {
    fn ok(
        elapsed: Duration,
        cube_a: &InputMesh,
        cube_b: &InputMesh,
        result: &TriangleSoup,
    ) -> Self {
        Self {
            elapsed,
            cube_a_triangles: cube_a.triangles.len(),
            cube_b_triangles: cube_b.triangles.len(),
            result_triangles: result.triangles.len(),
            result_vertices: result.vertices.len(),
            error: None,
        }
    }

    fn failed(elapsed: Duration, error: String) -> Self {
        Self {
            elapsed,
            error: Some(error),
            ..Self::default()
        }
    }
}

fn run_boolean(
    meshes: &[MeshRef<'_>],
    op: BooleanOp,
    config: EmberConfig,
) -> hypermesh::HypermeshResult<TriangleSoup> {
    let result = boolean_operation(meshes, op, config)?;
    triangulate_and_resolve_certified(&result)
}

fn cube_mesh(min: i32, max: i32) -> InputMesh {
    InputMesh::new(
        vec![
            p(min, min, min),
            p(max, min, min),
            p(max, max, min),
            p(min, max, min),
            p(min, min, max),
            p(max, min, max),
            p(max, max, max),
            p(min, max, max),
        ],
        cube_triangles(),
    )
}

fn shifted_cube_mesh(offset_quarters: i32) -> InputMesh {
    let offset = real_ratio(offset_quarters, 4);
    let min = &offset - &Real::one();
    let max = &offset + &Real::one();
    InputMesh::new(
        vec![
            point(min.clone(), r(-1), r(-1)),
            point(max.clone(), r(-1), r(-1)),
            point(max.clone(), r(1), r(-1)),
            point(min.clone(), r(1), r(-1)),
            point(min.clone(), r(-1), r(1)),
            point(max.clone(), r(-1), r(1)),
            point(max.clone(), r(1), r(1)),
            point(min, r(1), r(1)),
        ],
        cube_triangles(),
    )
}

fn cube_triangles() -> Vec<Triangle> {
    vec![
        Triangle::new(4, 5, 6),
        Triangle::new(4, 6, 7),
        Triangle::new(0, 3, 2),
        Triangle::new(0, 2, 1),
        Triangle::new(1, 2, 6),
        Triangle::new(1, 6, 5),
        Triangle::new(0, 4, 7),
        Triangle::new(0, 7, 3),
        Triangle::new(3, 7, 6),
        Triangle::new(3, 6, 2),
        Triangle::new(0, 1, 5),
        Triangle::new(0, 5, 4),
    ]
}

fn input_mesh_faces(mesh: &InputMesh, color: Color3) -> ExactMesh {
    let mut out = ExactMesh::empty(Primitive::Triangles);
    for triangle in &mesh.triangles {
        let [Some(a), Some(b), Some(c)] = triangle
            .indices()
            .map(|index| mesh.positions.get(index).cloned())
        else {
            continue;
        };
        let shaded = flat_shaded_color(color, [&a, &b, &c]);
        for point in [a, b, c] {
            out.push(ExactVertex::new(point, shaded));
        }
    }
    out
}

fn input_mesh_wire(mesh: &InputMesh, color: Color3) -> ExactMesh {
    let mut out = ExactMesh::empty(Primitive::Lines);
    for triangle in &mesh.triangles {
        push_wire_triangle(
            &mut out,
            triangle
                .indices()
                .map(|index| mesh.positions.get(index).cloned()),
            color,
        );
    }
    out
}

fn triangle_soup_faces(soup: &TriangleSoup, color: Color3) -> ExactMesh {
    let mut out = ExactMesh::empty(Primitive::Triangles);
    for triangle in &soup.triangles {
        let [Some(a), Some(b), Some(c)] =
            triangle.map(|index| soup.vertices.get(index).map(output_vertex_point))
        else {
            continue;
        };
        let shaded = flat_shaded_color(color, [&a, &b, &c]);
        for point in [a, b, c] {
            out.push(ExactVertex::new(point, shaded));
        }
    }
    out
}

fn flat_shaded_color(base: Color3, [a, b, c]: [&Point3; 3]) -> Color3 {
    const AMBIENT: f64 = 0.28;
    const DIFFUSE: f64 = 0.72;
    // Unit vector from the surface toward a fixed world-space key light.
    const LIGHT: [f64; 3] = [
        0.365_148_371_670_110_7,
        -0.182_574_185_835_055_36,
        0.912_870_929_175_276_9,
    ];

    let Some(a) = a.to_f64_array_lossy() else {
        return base;
    };
    let Some(b) = b.to_f64_array_lossy() else {
        return base;
    };
    let Some(c) = c.to_f64_array_lossy() else {
        return base;
    };
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let normal = [
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    ];
    let length =
        (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
    if !length.is_finite() || length <= f64::EPSILON {
        return base;
    }
    let diffuse = ((normal[0] * LIGHT[0] + normal[1] * LIGHT[1] + normal[2] * LIGHT[2]) / length)
        .max(0.0);
    let intensity = (AMBIENT + DIFFUSE * diffuse) as f32;
    Color3::new(
        base.r * intensity,
        base.g * intensity,
        base.b * intensity,
    )
    .unwrap_or(base)
}

fn triangle_soup_wire(soup: &TriangleSoup, color: Color3) -> ExactMesh {
    let mut out = ExactMesh::empty(Primitive::Lines);
    for triangle in &soup.triangles {
        push_wire_triangle(
            &mut out,
            triangle.map(|index| soup.vertices.get(index).map(output_vertex_point)),
            color,
        );
    }
    out
}

fn push_wire_triangle(out: &mut ExactMesh, vertices: [Option<Point3>; 3], color: Color3) {
    let [Some(a), Some(b), Some(c)] = vertices else {
        return;
    };
    out.push(ExactVertex::new(a.clone(), color));
    out.push(ExactVertex::new(b.clone(), color));
    out.push(ExactVertex::new(b.clone(), color));
    out.push(ExactVertex::new(c.clone(), color));
    out.push(ExactVertex::new(c, color));
    out.push(ExactVertex::new(a, color));
}

fn output_vertex_point(vertex: &OutputVertex) -> Point3 {
    point(vertex.x.clone(), vertex.y.clone(), vertex.z.clone())
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    point(r(x), r(y), r(z))
}

fn point(x: Real, y: Real, z: Real) -> Point3 {
    Point3::new(x, y, z)
}

fn r(value: i32) -> Real {
    value.into()
}

fn real_ratio(numerator: i32, denominator: i32) -> Real {
    if numerator % denominator == 0 {
        return r(numerator / denominator);
    }
    Real::from_str(&format!("{numerator}/{denominator}")).expect("literal rational should parse")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn demo_primitives_are_exact_cubes() {
        let cube = cube_mesh(-1, 1);
        let shifted = shifted_cube_mesh(4);

        assert_eq!(cube.positions.len(), 8);
        assert_eq!(cube.triangles.len(), 12);
        assert_eq!(shifted.triangles.len(), 12);
        assert_eq!(shifted.positions[0].x, r(0));
        assert_eq!(shifted.positions[1].x, r(2));
    }

    #[test]
    fn demo_boolean_operations_materialize() {
        let cube_a = cube_mesh(-1, 1);
        let cube_b = shifted_cube_mesh(4);
        let config = EmberConfig { max_depth: 8 };

        for operation in DemoOperation::ALL {
            let refs = match operation {
                DemoOperation::CubeBMinusA => vec![cube_b.as_ref(), cube_a.as_ref()],
                _ => vec![cube_a.as_ref(), cube_b.as_ref()],
            };
            let op = match operation {
                DemoOperation::Union => BooleanOp::Union,
                DemoOperation::Intersection => BooleanOp::Intersection,
                DemoOperation::CubeAMinusB | DemoOperation::CubeBMinusA => BooleanOp::Difference,
                DemoOperation::SymmetricDifference => BooleanOp::SymmetricDifference,
            };
            let result = run_boolean(&refs, op, config);
            assert!(
                result.is_ok(),
                "{} failed: {:?}",
                operation.label(),
                result.err()
            );
        }
    }

    #[test]
    fn flat_shading_is_consistent_across_coplanar_triangles() {
        let base = Color3::new(0.41, 0.86, 0.60).unwrap();
        let a = p(0, 0, 0);
        let b = p(2, 0, 0);
        let c = p(2, 2, 0);
        let d = p(0, 2, 0);

        assert_eq!(
            flat_shaded_color(base, [&a, &b, &c]),
            flat_shaded_color(base, [&a, &c, &d])
        );
    }

    #[test]
    fn flat_shading_distinguishes_surface_orientations() {
        let base = Color3::new(0.41, 0.86, 0.60).unwrap();
        let origin = p(0, 0, 0);
        let x = p(1, 0, 0);
        let y = p(0, 1, 0);
        let z = p(0, 0, 1);

        assert_ne!(
            flat_shaded_color(base, [&origin, &x, &y]),
            flat_shaded_color(base, [&origin, &y, &z])
        );
    }

    #[test]
    fn default_union_contains_only_exterior_boundary_triangles() {
        let cube_a = cube_mesh(-1, 1);
        let cube_b = shifted_cube_mesh(4);
        let result = run_boolean(
            &[cube_a.as_ref(), cube_b.as_ref()],
            BooleanOp::Union,
            EmberConfig { max_depth: 8 },
        )
        .unwrap();

        for triangle in &result.triangles {
            let vertices = triangle.map(|index| &result.vertices[index]);
            let on_boundary = [
                vertices.iter().all(|vertex| vertex.x == r(-1)),
                vertices.iter().all(|vertex| vertex.x == r(2)),
                vertices.iter().all(|vertex| vertex.y == r(-1)),
                vertices.iter().all(|vertex| vertex.y == r(1)),
                vertices.iter().all(|vertex| vertex.z == r(-1)),
                vertices.iter().all(|vertex| vertex.z == r(1)),
            ];
            assert!(
                on_boundary.into_iter().any(|is_boundary| is_boundary),
                "union emitted an internal triangle: {vertices:?}"
            );
        }
    }

    #[test]
    fn default_union_shades_each_exterior_plane_consistently() {
        let cube_a = cube_mesh(-1, 1);
        let cube_b = shifted_cube_mesh(4);
        let result = run_boolean(
            &[cube_a.as_ref(), cube_b.as_ref()],
            BooleanOp::Union,
            EmberConfig { max_depth: 8 },
        )
        .unwrap();
        let green = Color3::new(0.41, 0.86, 0.60).unwrap();
        let faces = triangle_soup_faces(&result, green);
        let mut plane_colors = [None; 6];

        for (triangle, rendered) in result
            .triangles
            .iter()
            .zip(faces.vertices().chunks_exact(3))
        {
            assert!(rendered.iter().all(|vertex| vertex.color == rendered[0].color));
            let vertices = triangle.map(|index| &result.vertices[index]);
            let plane = [
                vertices.iter().all(|vertex| vertex.x == r(-1)),
                vertices.iter().all(|vertex| vertex.x == r(2)),
                vertices.iter().all(|vertex| vertex.y == r(-1)),
                vertices.iter().all(|vertex| vertex.y == r(1)),
                vertices.iter().all(|vertex| vertex.z == r(-1)),
                vertices.iter().all(|vertex| vertex.z == r(1)),
            ]
            .into_iter()
            .position(|is_boundary| is_boundary)
            .expect("every union triangle should be on one exterior plane");
            match plane_colors[plane] {
                Some(color) => assert_eq!(rendered[0].color, color),
                None => plane_colors[plane] = Some(rendered[0].color),
            }
        }
        assert!(plane_colors.into_iter().all(|color| color.is_some()));
    }

    #[test]
    fn result_depth_prepass_preserves_alpha_without_accumulating_hidden_faces() {
        fn blend(source: f32, destination: f32) -> f32 {
            source * RESULT_FACE_ALPHA + destination * (1.0 - RESULT_FACE_ALPHA)
        }
        fn depth_prepass(draw_order: [(f32, f32); 2], background: f32) -> f32 {
            let (_, visible_color) = draw_order
                .into_iter()
                .min_by(|left, right| left.0.total_cmp(&right.0))
                .unwrap();
            blend(visible_color, background)
        }

        let background = 0.1;
        let hidden = (0.8, 0.28);
        let visible = (0.2, 0.94);
        let mesh_order_blend = blend(visible.1, blend(hidden.1, background));
        let depth_prepass_blend = depth_prepass([hidden, visible], background);

        assert_eq!(RESULT_FACE_ALPHA, 0.92);
        assert_ne!(mesh_order_blend, depth_prepass_blend);
        assert_eq!(
            depth_prepass_blend,
            depth_prepass([visible, hidden], background),
            "nearest-surface color should be independent of triangle order"
        );
    }
}
