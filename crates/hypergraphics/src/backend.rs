//! CopperForge-style glow backend for exact meshes.
//!
//! The backend receives f64 render exports from hyperreal-owned scene data and
//! narrows them to OpenGL f32 attributes only at upload/uniform time.

use glow::{Context, HasContext as _};

use crate::camera::Projection64;
use crate::error::{Error, Result};
use crate::geometry::{ExactMesh, Primitive, RenderVertex64};

const VS_UNLIT: &str = r#"#version 330
uniform mat4 u_mvp;
layout(location=0) in vec3 a_pos;
layout(location=1) in vec3 a_col;
out vec3 v_col;
void main() { v_col = a_col; gl_Position = u_mvp * vec4(a_pos, 1.0); }
"#;

const FS_UNLIT: &str = r#"#version 330
uniform float u_alpha;
in vec3 v_col;
out vec4 o_col;
void main() { o_col = vec4(v_col, u_alpha); }
"#;

/// GPU-side colored mesh with CopperForge's `xyz rgb` stride.
pub struct GpuColoredMesh {
    vao: glow::VertexArray,
    vbo: glow::Buffer,
    vertex_count: i32,
    primitive: Primitive,
}

impl GpuColoredMesh {
    /// Create an empty GPU mesh.
    ///
    /// # Safety
    ///
    /// `gl` must be a current, valid OpenGL context.
    pub unsafe fn new(gl: &Context, primitive: Primitive) -> Result<Self> {
        unsafe {
            let vao = gl.create_vertex_array().map_err(Error::Backend)?;
            let vbo = gl.create_buffer().map_err(Error::Backend)?;
            gl.bind_vertex_array(Some(vao));
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo));
            gl.enable_vertex_attrib_array(0);
            gl.vertex_attrib_pointer_f32(0, 3, glow::FLOAT, false, 24, 0);
            gl.enable_vertex_attrib_array(1);
            gl.vertex_attrib_pointer_f32(1, 3, glow::FLOAT, false, 24, 12);
            gl.bind_vertex_array(None);
            Ok(Self {
                vao,
                vbo,
                vertex_count: 0,
                primitive,
            })
        }
    }

    /// Upload already exported f64 render vertices, narrowing to f32 at the GL edge.
    ///
    /// # Safety
    ///
    /// `gl` must be the context that owns this mesh's objects.
    pub unsafe fn upload_render_vertices64(
        &mut self,
        gl: &Context,
        vertices: &[RenderVertex64],
    ) -> Result<()> {
        let mut packed = Vec::with_capacity(vertices.len() * 6);
        for vertex in vertices {
            for value in vertex.position {
                packed.push(f64_to_f32(value, "vertex position")?);
            }
            packed.extend_from_slice(&vertex.color);
        }
        unsafe {
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(self.vbo));
            gl.buffer_data_u8_slice(
                glow::ARRAY_BUFFER,
                bytemuck::cast_slice(&packed),
                glow::STATIC_DRAW,
            );
        }
        self.vertex_count = vertices.len() as i32;
        Ok(())
    }

    /// Export and upload a hyperreal-owned mesh.
    ///
    /// # Safety
    ///
    /// `gl` must be the context that owns this mesh's objects.
    pub unsafe fn upload_exact_mesh(&mut self, gl: &Context, mesh: &ExactMesh) -> Result<()> {
        self.primitive = mesh.primitive();
        unsafe { self.upload_render_vertices64(gl, &mesh.to_render_vertices64()?) }
    }

    /// Draw the mesh.
    ///
    /// # Safety
    ///
    /// A compatible shader program must be bound and `gl` must own this mesh.
    pub unsafe fn draw(&self, gl: &Context) {
        if self.vertex_count == 0 {
            return;
        }
        unsafe {
            gl.bind_vertex_array(Some(self.vao));
            gl.draw_arrays(self.primitive.to_glow(), 0, self.vertex_count);
        }
    }
}

// The egui_glow paint callback closure must be Send + Sync. glow handles are
// used on the UI thread in the extracted CopperForge pattern.
unsafe impl Send for GpuColoredMesh {}
unsafe impl Sync for GpuColoredMesh {}

/// Unlit shader program compatible with [`GpuColoredMesh`].
pub struct UnlitProgram {
    prog: glow::Program,
    u_mvp: glow::UniformLocation,
    u_alpha: glow::UniformLocation,
}

impl UnlitProgram {
    /// Compile the unlit shader program.
    ///
    /// # Safety
    ///
    /// `gl` must be a current, valid OpenGL context.
    pub unsafe fn new(gl: &Context) -> Result<Self> {
        unsafe {
            let prog = compile(gl, VS_UNLIT, FS_UNLIT)?;
            let u_mvp = gl
                .get_uniform_location(prog, "u_mvp")
                .ok_or_else(|| Error::Backend("unlit shader missing u_mvp".to_string()))?;
            let u_alpha = gl
                .get_uniform_location(prog, "u_alpha")
                .ok_or_else(|| Error::Backend("unlit shader missing u_alpha".to_string()))?;
            Ok(Self {
                prog,
                u_mvp,
                u_alpha,
            })
        }
    }

    /// Bind the program and upload the f64 projection matrix after f32 narrowing.
    ///
    /// # Safety
    ///
    /// `gl` must own this program.
    pub unsafe fn bind(&self, gl: &Context, projection: &Projection64) -> Result<()> {
        let matrix = projection.to_f32_array()?;
        unsafe {
            gl.use_program(Some(self.prog));
            gl.uniform_matrix_4_f32_slice(Some(&self.u_mvp), false, &matrix);
            gl.uniform_1_f32(Some(&self.u_alpha), 1.0);
        }
        Ok(())
    }

    /// Override fragment alpha until the next bind.
    ///
    /// # Safety
    ///
    /// `gl` must own this program and the program should be current.
    pub unsafe fn set_alpha(&self, gl: &Context, alpha: f32) -> Result<()> {
        if !alpha.is_finite() {
            return Err(Error::NonFinitePrimitive { value: "alpha" });
        }
        unsafe {
            gl.uniform_1_f32(Some(&self.u_alpha), alpha);
        }
        Ok(())
    }
}

unsafe impl Send for UnlitProgram {}
unsafe impl Sync for UnlitProgram {}

impl Primitive {
    fn to_glow(self) -> u32 {
        match self {
            Self::Lines => glow::LINES,
            Self::Triangles => glow::TRIANGLES,
        }
    }
}

unsafe fn compile(gl: &Context, vs_src: &str, fs_src: &str) -> Result<glow::Program> {
    unsafe {
        let make = |kind: u32, src: &str| -> Result<glow::Shader> {
            let shader = gl.create_shader(kind).map_err(Error::Backend)?;
            gl.shader_source(shader, src);
            gl.compile_shader(shader);
            if !gl.get_shader_compile_status(shader) {
                let log = gl.get_shader_info_log(shader);
                gl.delete_shader(shader);
                return Err(Error::Backend(format!("shader compile error: {log}")));
            }
            Ok(shader)
        };
        let vs = make(glow::VERTEX_SHADER, vs_src)?;
        let fs = make(glow::FRAGMENT_SHADER, fs_src)?;
        let prog = gl.create_program().map_err(Error::Backend)?;
        gl.attach_shader(prog, vs);
        gl.attach_shader(prog, fs);
        gl.link_program(prog);
        gl.delete_shader(vs);
        gl.delete_shader(fs);
        if !gl.get_program_link_status(prog) {
            let log = gl.get_program_info_log(prog);
            gl.delete_program(prog);
            return Err(Error::Backend(format!("shader link error: {log}")));
        }
        Ok(prog)
    }
}

fn f64_to_f32(value: f64, name: &'static str) -> Result<f32> {
    if !value.is_finite() {
        return Err(Error::NonFiniteProjection { value: name });
    }
    let narrowed = value as f32;
    if !narrowed.is_finite() {
        return Err(Error::F32Overflow { value: name });
    }
    Ok(narrowed)
}
