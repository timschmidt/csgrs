//! Geometry-only VRML 2.0 (`.wrl`) mesh import.
//!
//! The importer targets the polygonal subset used by KiCad package models:
//! transform/group/shape scene graphs and `IndexedFaceSet` geometry. Rendering
//! attributes are intentionally ignored. Unsupported geometry fails explicitly
//! instead of producing an incomplete mesh.

use crate::io::IoError;
use crate::mesh::Mesh;
use crate::mesh::polygon::triangulate_indexed_positions_into;
use crate::triangulated::IndexedTriangleMesh3D;
use hashbrown::HashMap;
use hyperlattice::{Matrix4, Point3, Real, Vector3};
use std::fmt::Debug;

/// Geometry evidence produced while flattening a VRML scene.
#[derive(Clone, Debug)]
pub struct VrmlMeshImport<M: Clone + Debug + Send + Sync> {
    /// Flattened exact-aware triangle mesh.
    pub mesh: Mesh<M>,
    /// Number of `Shape` nodes visited.
    pub shape_count: usize,
    /// Number of `IndexedFaceSet` nodes consumed.
    pub indexed_face_set_count: usize,
    /// Number of line or point geometry nodes intentionally omitted.
    pub ignored_non_mesh_geometry_count: usize,
    /// Number of degenerate polygons that could not form a surface.
    pub ignored_degenerate_polygon_count: usize,
    /// Number of emitted triangles omitted after exact degeneracy checks.
    pub ignored_degenerate_triangle_count: usize,
}

/// Imports polygonal geometry from a VRML 2.0 scene.
///
/// Decimal coordinates and transforms are parsed into [`Real`] before scene
/// flattening. `DEF`/`USE` reuse is supported for geometry and scene nodes.
/// Appearance, material, normal, color, and texture data are accepted but do
/// not cross this geometry-only boundary.
pub fn from_vrml<M: Clone + Debug + Send + Sync>(
    bytes: &[u8],
    metadata: M,
) -> Result<VrmlMeshImport<M>, IoError> {
    let text = std::str::from_utf8(bytes)
        .map_err(|error| malformed(format!("document is not UTF-8: {error}")))?;
    let header = text
        .lines()
        .next()
        .map(|line| line.trim_start_matches('\u{feff}').trim())
        .unwrap_or_default();
    if !header.eq_ignore_ascii_case("#VRML V2.0 utf8") {
        return Err(IoError::Unsupported {
            format: "VRML",
            detail: "only VRML V2.0 utf8 documents are supported".into(),
        });
    }

    let tokens = lex(text)?;
    let mut parser = Parser::new(tokens);
    let roots = parser.parse_document()?;
    let mut flattened = Flattened::default();
    let identity = Matrix4::identity();
    for root in &roots {
        flatten_node(root, &identity, &mut flattened)?;
    }
    if flattened.triangles.is_empty() {
        return Err(IoError::Geometry {
            format: "VRML",
            detail: "scene contains no triangulatable IndexedFaceSet geometry".into(),
        });
    }

    let mut normals = Vec::with_capacity(flattened.triangles.len());
    let mut faces = Vec::with_capacity(flattened.triangles.len());
    let triangles = std::mem::take(&mut flattened.triangles);
    for [a, b, c] in triangles {
        let ab = &flattened.positions[b] - &flattened.positions[a];
        let ac = &flattened.positions[c] - &flattened.positions[a];
        let Ok(normal) = ab.unit_cross_checked(&ac) else {
            flattened.ignored_degenerate_triangle_count = flattened
                .ignored_degenerate_triangle_count
                .checked_add(1)
                .ok_or(IoError::SizeOverflow {
                    format: "VRML",
                    limit: "ignored degenerate triangle count",
                })?;
            continue;
        };
        let normal_index = normals.len();
        normals.push(normal);
        faces.push([(a, normal_index), (b, normal_index), (c, normal_index)]);
    }
    if faces.is_empty() {
        return Err(IoError::Geometry {
            format: "VRML",
            detail: "scene contains no certifiably nondegenerate triangles".into(),
        });
    }
    let mesh = Mesh::from_indexed_triangles(
        IndexedTriangleMesh3D {
            positions: flattened.positions,
            normals,
            faces,
        },
        metadata,
    )
    .map_err(|error| IoError::Geometry {
        format: "VRML",
        detail: error.to_string(),
    })?;

    Ok(VrmlMeshImport {
        mesh,
        shape_count: flattened.shape_count,
        indexed_face_set_count: flattened.indexed_face_set_count,
        ignored_non_mesh_geometry_count: flattened.ignored_non_mesh_geometry_count,
        ignored_degenerate_polygon_count: flattened.ignored_degenerate_polygon_count,
        ignored_degenerate_triangle_count: flattened.ignored_degenerate_triangle_count,
    })
}

#[derive(Clone, Debug, PartialEq)]
enum Token {
    Word(String),
    String(String),
    LeftBrace,
    RightBrace,
    LeftBracket,
    RightBracket,
}

fn lex(text: &str) -> Result<Vec<Token>, IoError> {
    let mut tokens = Vec::new();
    let mut characters = text.chars().peekable();
    while let Some(character) = characters.next() {
        match character {
            character if character.is_whitespace() || character == ',' => {},
            '#' => {
                for character in characters.by_ref() {
                    if character == '\n' {
                        break;
                    }
                }
            },
            '{' => tokens.push(Token::LeftBrace),
            '}' => tokens.push(Token::RightBrace),
            '[' => tokens.push(Token::LeftBracket),
            ']' => tokens.push(Token::RightBracket),
            '"' => {
                let mut value = String::new();
                let mut terminated = false;
                while let Some(character) = characters.next() {
                    match character {
                        '"' => {
                            terminated = true;
                            break;
                        },
                        '\\' => {
                            let escaped = characters
                                .next()
                                .ok_or_else(|| malformed("unterminated string escape"))?;
                            value.push(escaped);
                        },
                        character => value.push(character),
                    }
                }
                if !terminated {
                    return Err(malformed("unterminated string literal"));
                }
                tokens.push(Token::String(value));
            },
            _ => {
                let mut word = String::from(character);
                while let Some(&next) = characters.peek() {
                    if next.is_whitespace()
                        || matches!(next, ',' | '#' | '"' | '{' | '}' | '[' | ']')
                    {
                        break;
                    }
                    word.push(next);
                    characters.next();
                }
                tokens.push(Token::Word(word));
            },
        }
    }
    Ok(tokens)
}

#[derive(Clone, Debug)]
enum Node {
    Transform(Box<TransformNode>),
    Group(Vec<Node>),
    Shape(Option<Box<Node>>),
    IndexedFaceSet {
        positions: Vec<Point3>,
        polygons: Vec<Vec<usize>>,
        ccw: bool,
    },
    Coordinate(Vec<Point3>),
    NonMeshGeometry,
    Ignored,
}

#[derive(Clone, Debug)]
struct TransformNode {
    center: [Real; 3],
    rotation: Rotation,
    scale: [Real; 3],
    scale_orientation: Rotation,
    translation: [Real; 3],
    children: Vec<Node>,
}

#[derive(Clone, Debug)]
struct Rotation {
    axis: [Real; 3],
    angle: Real,
}

impl Default for Rotation {
    fn default() -> Self {
        Self {
            axis: [Real::zero(), Real::zero(), Real::one()],
            angle: Real::zero(),
        }
    }
}

struct Parser {
    tokens: Vec<Token>,
    cursor: usize,
    definitions: HashMap<String, Node>,
}

impl Parser {
    fn new(tokens: Vec<Token>) -> Self {
        Self {
            tokens,
            cursor: 0,
            definitions: HashMap::new(),
        }
    }

    fn parse_document(&mut self) -> Result<Vec<Node>, IoError> {
        let mut roots = Vec::new();
        while self.peek().is_some() {
            roots.push(self.parse_node()?);
        }
        Ok(roots)
    }

    fn parse_node(&mut self) -> Result<Node, IoError> {
        if self.consume_word("DEF") {
            let name = self.word("DEF name")?;
            let node = self.parse_node()?;
            self.definitions.insert(name, node.clone());
            return Ok(node);
        }
        if self.consume_word("USE") {
            let name = self.word("USE name")?;
            return self
                .definitions
                .get(&name)
                .cloned()
                .ok_or_else(|| malformed(format!("USE references undefined node {name:?}")));
        }

        let kind = self.word("node type")?;
        self.expect(Token::LeftBrace, "opening node brace")?;
        match kind.as_str() {
            "Transform" => self.parse_transform(),
            "Group" => self.parse_group(),
            "Shape" => self.parse_shape(),
            "IndexedFaceSet" => self.parse_indexed_face_set(),
            "Coordinate" => self.parse_coordinate(),
            "Appearance" | "Material" | "Normal" | "Color" | "TextureCoordinate"
            | "ImageTexture" | "MovieTexture" | "PixelTexture" | "TextureTransform"
            | "WorldInfo" | "NavigationInfo" | "Background" | "Viewpoint" | "Fog"
            | "DirectionalLight" | "PointLight" | "SpotLight" => {
                self.skip_braced_body()?;
                Ok(Node::Ignored)
            },
            "IndexedLineSet" | "PointSet" => {
                self.skip_braced_body()?;
                Ok(Node::NonMeshGeometry)
            },
            "Box" | "Cone" | "Cylinder" | "ElevationGrid" | "Extrusion" | "Sphere"
            | "Text" => Err(IoError::Unsupported {
                format: "VRML",
                detail: format!("geometry node {kind} is not polygonal IndexedFaceSet data"),
            }),
            _ => Err(IoError::Unsupported {
                format: "VRML",
                detail: format!("scene node {kind:?} is not supported"),
            }),
        }
    }

    fn parse_transform(&mut self) -> Result<Node, IoError> {
        let mut center = zero3();
        let mut rotation = Rotation::default();
        let mut scale = one3();
        let mut scale_orientation = Rotation::default();
        let mut translation = zero3();
        let mut children = Vec::new();
        while !self.consume(&Token::RightBrace) {
            let field = self.word("Transform field")?;
            match field.as_str() {
                "center" => center = self.real3()?,
                "rotation" => rotation = self.rotation()?,
                "scale" => scale = self.real3()?,
                "scaleOrientation" => scale_orientation = self.rotation()?,
                "translation" => translation = self.real3()?,
                "children" => children = self.node_list()?,
                "bboxCenter" | "bboxSize" => {
                    self.real3()?;
                },
                _ => return Err(malformed(format!("unknown Transform field {field:?}"))),
            }
        }
        Ok(Node::Transform(Box::new(TransformNode {
            center,
            rotation,
            scale,
            scale_orientation,
            translation,
            children,
        })))
    }

    fn parse_group(&mut self) -> Result<Node, IoError> {
        let mut children = Vec::new();
        while !self.consume(&Token::RightBrace) {
            let field = self.word("Group field")?;
            match field.as_str() {
                "children" => children = self.node_list()?,
                "bboxCenter" | "bboxSize" => {
                    self.real3()?;
                },
                _ => return Err(malformed(format!("unknown Group field {field:?}"))),
            }
        }
        Ok(Node::Group(children))
    }

    fn parse_shape(&mut self) -> Result<Node, IoError> {
        let mut geometry = None;
        while !self.consume(&Token::RightBrace) {
            let field = self.word("Shape field")?;
            match field.as_str() {
                "appearance" => self.skip_node_value()?,
                "geometry" if self.consume_word("NULL") => {},
                "geometry" => geometry = Some(Box::new(self.parse_node()?)),
                _ => return Err(malformed(format!("unknown Shape field {field:?}"))),
            }
        }
        Ok(Node::Shape(geometry))
    }

    fn parse_indexed_face_set(&mut self) -> Result<Node, IoError> {
        let mut positions = None;
        let mut coordinate_indices = None;
        let mut ccw = true;
        while !self.consume(&Token::RightBrace) {
            let field = self.word("IndexedFaceSet field")?;
            match field.as_str() {
                "coord" if self.consume_word("NULL") => {},
                "coord" => match self.parse_node()? {
                    Node::Coordinate(points) => positions = Some(points),
                    _ => {
                        return Err(malformed(
                            "IndexedFaceSet coord is not a Coordinate node",
                        ));
                    },
                },
                "coordIndex" => coordinate_indices = Some(self.integer_list()?),
                "ccw" => ccw = self.boolean()?,
                "color" | "normal" | "texCoord" => self.skip_node_value()?,
                "colorIndex" | "normalIndex" | "texCoordIndex" => {
                    self.integer_list()?;
                },
                "colorPerVertex" | "convex" | "normalPerVertex" | "solid" => {
                    self.boolean()?;
                },
                "creaseAngle" => {
                    self.real()?;
                },
                _ => {
                    return Err(malformed(format!("unknown IndexedFaceSet field {field:?}")));
                },
            }
        }
        let positions =
            positions.ok_or_else(|| malformed("IndexedFaceSet has no Coordinate node"))?;
        let indices =
            coordinate_indices.ok_or_else(|| malformed("IndexedFaceSet has no coordIndex"))?;
        let polygons = split_coordinate_indices(indices)?;
        Ok(Node::IndexedFaceSet {
            positions,
            polygons,
            ccw,
        })
    }

    fn parse_coordinate(&mut self) -> Result<Node, IoError> {
        let mut points = None;
        while !self.consume(&Token::RightBrace) {
            let field = self.word("Coordinate field")?;
            match field.as_str() {
                "point" => {
                    self.expect(Token::LeftBracket, "Coordinate point list")?;
                    let mut coordinates = Vec::new();
                    while !self.consume(&Token::RightBracket) {
                        coordinates.push(self.real()?);
                    }
                    if coordinates.len() % 3 != 0 {
                        return Err(malformed(
                            "Coordinate point list length is not divisible by three",
                        ));
                    }
                    points = Some(
                        coordinates
                            .chunks_exact(3)
                            .map(|point| {
                                Point3::new(
                                    point[0].clone(),
                                    point[1].clone(),
                                    point[2].clone(),
                                )
                            })
                            .collect(),
                    );
                },
                _ => return Err(malformed(format!("unknown Coordinate field {field:?}"))),
            }
        }
        Ok(Node::Coordinate(points.ok_or_else(|| {
            malformed("Coordinate node has no point field")
        })?))
    }

    fn node_list(&mut self) -> Result<Vec<Node>, IoError> {
        if self.consume_word("NULL") {
            return Ok(Vec::new());
        }
        if !self.consume(&Token::LeftBracket) {
            return Ok(vec![self.parse_node()?]);
        }
        let mut nodes = Vec::new();
        while !self.consume(&Token::RightBracket) {
            nodes.push(self.parse_node()?);
        }
        Ok(nodes)
    }

    fn skip_node_value(&mut self) -> Result<(), IoError> {
        if self.consume_word("NULL") {
            return Ok(());
        }
        if self.consume_word("USE") {
            self.word("USE name")?;
            return Ok(());
        }
        if self.consume_word("DEF") {
            self.word("DEF name")?;
        }
        self.word("node type")?;
        self.expect(Token::LeftBrace, "opening node brace")?;
        self.skip_braced_body()
    }

    fn skip_braced_body(&mut self) -> Result<(), IoError> {
        let mut depth = 1usize;
        while depth > 0 {
            match self
                .next()
                .ok_or_else(|| malformed("unterminated node body"))?
            {
                Token::LeftBrace => depth += 1,
                Token::RightBrace => depth -= 1,
                _ => {},
            }
        }
        Ok(())
    }

    fn integer_list(&mut self) -> Result<Vec<i64>, IoError> {
        self.expect(Token::LeftBracket, "integer list")?;
        let mut values = Vec::new();
        while !self.consume(&Token::RightBracket) {
            let text = self.word("integer")?;
            values.push(
                text.parse().map_err(|error| {
                    malformed(format!("invalid integer {text:?}: {error}"))
                })?,
            );
        }
        Ok(values)
    }

    fn boolean(&mut self) -> Result<bool, IoError> {
        let text = self.word("boolean")?;
        match text.as_str() {
            "TRUE" | "true" => Ok(true),
            "FALSE" | "false" => Ok(false),
            _ => Err(malformed(format!("invalid boolean {text:?}"))),
        }
    }

    fn rotation(&mut self) -> Result<Rotation, IoError> {
        let [x, y, z] = self.real3()?;
        Ok(Rotation {
            axis: [x, y, z],
            angle: self.real()?,
        })
    }

    fn real3(&mut self) -> Result<[Real; 3], IoError> {
        Ok([self.real()?, self.real()?, self.real()?])
    }

    fn real(&mut self) -> Result<Real, IoError> {
        let text = self.word("number")?;
        parse_real(&text)
            .map_err(|error| malformed(format!("invalid number {text:?}: {error}")))
    }

    fn word(&mut self, context: &str) -> Result<String, IoError> {
        match self.next() {
            Some(Token::Word(word)) => Ok(word),
            Some(Token::String(value)) => Err(malformed(format!(
                "expected {context}, found string {value:?}"
            ))),
            Some(token) => Err(malformed(format!("expected {context}, found {token:?}"))),
            None => Err(malformed(format!("expected {context}, found end of input"))),
        }
    }

    fn expect(&mut self, expected: Token, context: &str) -> Result<(), IoError> {
        match self.next() {
            Some(token) if token == expected => Ok(()),
            Some(token) => Err(malformed(format!(
                "expected {context} ({expected:?}), found {token:?}"
            ))),
            None => Err(malformed(format!(
                "expected {context} ({expected:?}), found end of input"
            ))),
        }
    }

    fn consume_word(&mut self, expected: &str) -> bool {
        if matches!(self.peek(), Some(Token::Word(word)) if word == expected) {
            self.cursor += 1;
            true
        } else {
            false
        }
    }

    fn consume(&mut self, expected: &Token) -> bool {
        if self.peek() == Some(expected) {
            self.cursor += 1;
            true
        } else {
            false
        }
    }

    fn peek(&self) -> Option<&Token> {
        self.tokens.get(self.cursor)
    }

    fn next(&mut self) -> Option<Token> {
        let token = self.tokens.get(self.cursor).cloned();
        self.cursor += usize::from(token.is_some());
        token
    }
}

fn split_coordinate_indices(indices: Vec<i64>) -> Result<Vec<Vec<usize>>, IoError> {
    let mut polygons = Vec::new();
    let mut polygon = Vec::new();
    for index in indices {
        if index == -1 {
            if polygon.is_empty() {
                return Err(malformed("coordIndex contains an empty polygon"));
            }
            polygons.push(std::mem::take(&mut polygon));
        } else {
            polygon.push(usize::try_from(index).map_err(|_| {
                malformed(format!("coordIndex contains negative index {index}"))
            })?);
        }
    }
    if !polygon.is_empty() {
        polygons.push(polygon);
    }
    if polygons.is_empty() {
        return Err(malformed("coordIndex contains no polygons"));
    }
    Ok(polygons)
}

#[derive(Default)]
struct Flattened {
    positions: Vec<Point3>,
    triangles: Vec<[usize; 3]>,
    shape_count: usize,
    indexed_face_set_count: usize,
    ignored_non_mesh_geometry_count: usize,
    ignored_degenerate_polygon_count: usize,
    ignored_degenerate_triangle_count: usize,
}

fn flatten_node(
    node: &Node,
    parent: &Matrix4,
    flattened: &mut Flattened,
) -> Result<(), IoError> {
    match node {
        Node::Transform(transform) => {
            let local = transform_matrix(
                &transform.center,
                &transform.rotation,
                &transform.scale,
                &transform.scale_orientation,
                &transform.translation,
            )?;
            let world = parent * &local;
            for child in &transform.children {
                flatten_node(child, &world, flattened)?;
            }
        },
        Node::Group(children) => {
            for child in children {
                flatten_node(child, parent, flattened)?;
            }
        },
        Node::Shape(geometry) => {
            flattened.shape_count =
                flattened
                    .shape_count
                    .checked_add(1)
                    .ok_or(IoError::SizeOverflow {
                        format: "VRML",
                        limit: "shape count",
                    })?;
            if let Some(geometry) = geometry {
                flatten_node(geometry, parent, flattened)?;
            }
        },
        Node::IndexedFaceSet {
            positions,
            polygons,
            ccw,
        } => {
            flattened.indexed_face_set_count = flattened
                .indexed_face_set_count
                .checked_add(1)
                .ok_or(IoError::SizeOverflow {
                    format: "VRML",
                    limit: "IndexedFaceSet count",
                })?;
            let base = flattened.positions.len();
            for position in positions {
                flattened
                    .positions
                    .push(parent.transform_point3(position).map_err(|error| {
                        IoError::Geometry {
                            format: "VRML",
                            detail: format!("scene transform failed: {error}"),
                        }
                    })?);
            }
            let mut face_indices = Vec::new();
            let mut face_triangles = Vec::new();
            let mut projected = Vec::new();
            for polygon in polygons {
                if polygon.len() < 3 {
                    return Err(malformed(
                        "IndexedFaceSet polygon has fewer than three vertices",
                    ));
                }
                face_indices.clear();
                for &index in polygon {
                    if index >= positions.len() {
                        return Err(malformed(format!(
                            "IndexedFaceSet references vertex {index} but has {} coordinates",
                            positions.len()
                        )));
                    }
                    face_indices.push(base.checked_add(index).ok_or(
                        IoError::SizeOverflow {
                            format: "VRML",
                            limit: "vertex index",
                        },
                    )?);
                }
                if !ccw {
                    face_indices.reverse();
                }
                triangulate_indexed_positions_into(
                    &flattened.positions,
                    &face_indices,
                    &mut face_triangles,
                    &mut projected,
                );
                if face_triangles.is_empty() {
                    flattened.ignored_degenerate_polygon_count = flattened
                        .ignored_degenerate_polygon_count
                        .checked_add(1)
                        .ok_or(IoError::SizeOverflow {
                            format: "VRML",
                            limit: "ignored degenerate polygon count",
                        })?;
                    continue;
                }
                flattened
                    .triangles
                    .extend(face_triangles.iter().map(|triangle| {
                        [
                            face_indices[triangle[0]],
                            face_indices[triangle[1]],
                            face_indices[triangle[2]],
                        ]
                    }));
            }
        },
        Node::NonMeshGeometry => {
            flattened.ignored_non_mesh_geometry_count = flattened
                .ignored_non_mesh_geometry_count
                .checked_add(1)
                .ok_or(IoError::SizeOverflow {
                    format: "VRML",
                    limit: "ignored non-mesh geometry count",
                })?;
        },
        Node::Coordinate(_) | Node::Ignored => {},
    }
    Ok(())
}

fn transform_matrix(
    center: &[Real; 3],
    rotation: &Rotation,
    scale: &[Real; 3],
    scale_orientation: &Rotation,
    translation: &[Real; 3],
) -> Result<Matrix4, IoError> {
    let translation = Matrix4::affine_translation(translation.clone());
    let center_translation = Matrix4::affine_translation(center.clone());
    let rotation = rotation_matrix(rotation)?;
    let scale_orientation_matrix = rotation_matrix(scale_orientation)?;
    let scale_matrix = Matrix4::affine_nonuniform_scale(scale.clone());
    let inverse_scale_orientation = rotation_matrix(&Rotation {
        axis: scale_orientation.axis.clone(),
        angle: -scale_orientation.angle.clone(),
    })?;
    let inverse_center =
        Matrix4::affine_translation(center.clone().map(|coordinate| -coordinate));
    Ok(&(&(&(&(&(&translation * &center_translation) * &rotation)
        * &scale_orientation_matrix)
        * &scale_matrix)
        * &inverse_scale_orientation)
        * &inverse_center)
}

fn rotation_matrix(rotation: &Rotation) -> Result<Matrix4, IoError> {
    Matrix4::rotation_axis_angle(
        &Vector3::from_xyz(
            rotation.axis[0].clone(),
            rotation.axis[1].clone(),
            rotation.axis[2].clone(),
        ),
        rotation.angle.clone(),
    )
    .map_err(|error| IoError::Geometry {
        format: "VRML",
        detail: format!("invalid axis-angle transform: {error}"),
    })
}

fn zero3() -> [Real; 3] {
    [Real::zero(), Real::zero(), Real::zero()]
}

fn one3() -> [Real; 3] {
    [Real::one(), Real::one(), Real::one()]
}

fn malformed(message: impl Into<String>) -> IoError {
    IoError::MalformedInput(format!("VRML: {}", message.into()))
}

fn parse_real(text: &str) -> Result<Real, hyperlattice::Problem> {
    const MAX_EXPANDED_DECIMAL_LEN: usize = 1_000_000;

    if !text.contains(['e', 'E']) {
        return text.parse();
    }
    let (mantissa, exponent) = text
        .split_once(['e', 'E'])
        .ok_or(hyperlattice::Problem::BadDecimal)?;
    let exponent = exponent
        .parse::<i64>()
        .map_err(|_| hyperlattice::Problem::BadDecimal)?;
    let (sign, magnitude) = if let Some(magnitude) = mantissa.strip_prefix('-') {
        ("-", magnitude)
    } else if let Some(magnitude) = mantissa.strip_prefix('+') {
        ("", magnitude)
    } else {
        ("", mantissa)
    };
    let (whole, fraction) = magnitude
        .split_once('.')
        .map_or((magnitude, ""), |parts| parts);
    if (whole.is_empty() && fraction.is_empty())
        || !whole.bytes().all(|byte| byte.is_ascii_digit())
        || !fraction.bytes().all(|byte| byte.is_ascii_digit())
    {
        return Err(hyperlattice::Problem::BadDecimal);
    }

    let mut digits = String::with_capacity(whole.len().saturating_add(fraction.len()));
    digits.push_str(whole);
    digits.push_str(fraction);
    let decimal_position = i64::try_from(whole.len())
        .ok()
        .and_then(|whole_len| whole_len.checked_add(exponent))
        .ok_or(hyperlattice::Problem::OutOfRange)?;
    let digits_len =
        i64::try_from(digits.len()).map_err(|_| hyperlattice::Problem::OutOfRange)?;
    let mut normalized = String::from(sign);
    if decimal_position <= 0 {
        normalized.push_str("0.");
        let zero_count = usize::try_from(
            decimal_position
                .checked_neg()
                .ok_or(hyperlattice::Problem::OutOfRange)?,
        )
        .map_err(|_| hyperlattice::Problem::OutOfRange)?;
        if zero_count.saturating_add(digits.len()) > MAX_EXPANDED_DECIMAL_LEN {
            return Err(hyperlattice::Problem::OutOfRange);
        }
        normalized.extend(std::iter::repeat_n('0', zero_count));
        normalized.push_str(&digits);
    } else if decimal_position >= digits_len {
        normalized.push_str(&digits);
        let zero_count = usize::try_from(decimal_position - digits_len)
            .map_err(|_| hyperlattice::Problem::OutOfRange)?;
        if zero_count.saturating_add(digits.len()) > MAX_EXPANDED_DECIMAL_LEN {
            return Err(hyperlattice::Problem::OutOfRange);
        }
        normalized.extend(std::iter::repeat_n('0', zero_count));
    } else {
        let decimal_position = usize::try_from(decimal_position)
            .map_err(|_| hyperlattice::Problem::OutOfRange)?;
        normalized.push_str(&digits[..decimal_position]);
        normalized.push('.');
        normalized.push_str(&digits[decimal_position..]);
    }
    normalized.parse()
}

#[cfg(test)]
mod tests {
    use super::from_vrml;
    use crate::triangulated::IndexedTriangulated3D;
    use hyperlattice::Real;

    #[test]
    fn imports_transformed_polygon_with_render_attributes() {
        let source = br#"#VRML V2.0 utf8
Transform {
  translation 10 20 30
  scale 2 3 4
  children [
    Shape {
      appearance Appearance {
        material DEF copper Material { diffuseColor 0.8 0.4 0.1 }
      }
      geometry IndexedFaceSet {
        ccw TRUE
        solid TRUE
        coord Coordinate {
          point [ 0 0 0, 1e0 0 0, 1 1 0, 0 1 0 ]
        }
        coordIndex [ 0, 1, 2, 3, -1 ]
        normalPerVertex TRUE
      }
    }
  ]
}
"#;
        let imported = from_vrml(source, "package").unwrap();
        assert_eq!(imported.shape_count, 1);
        assert_eq!(imported.indexed_face_set_count, 1);
        assert_eq!(imported.ignored_non_mesh_geometry_count, 0);
        assert_eq!(imported.ignored_degenerate_polygon_count, 0);
        assert_eq!(imported.ignored_degenerate_triangle_count, 0);
        let indexed = imported.mesh.indexed_triangles();
        assert_eq!(indexed.faces.len(), 2);
        assert!(indexed.positions.iter().any(|point| {
            point.x == Real::from(12) && point.y == Real::from(23) && point.z == Real::from(30)
        }));
    }

    #[test]
    fn resolves_coordinate_def_use_and_reverses_clockwise_faces() {
        let source = br#"#VRML V2.0 utf8
DEF points Coordinate { point [ 0 0 0, 1 0 0, 0 1 0 ] }
Shape {
  appearance NULL
  geometry IndexedFaceSet {
    ccw FALSE
    coord USE points
    coordIndex [ 0 1 2 -1 ]
  }
}
"#;
        let imported = from_vrml(source, ()).unwrap();
        let indexed = imported.mesh.indexed_triangles();
        assert_eq!(indexed.faces.len(), 1);
        let normal = &indexed.normals[indexed.faces[0][0].1];
        assert_eq!(normal.0[2], Real::from(-1));
    }

    #[test]
    fn rejects_non_indexed_face_geometry() {
        let source = br#"#VRML V2.0 utf8
Shape { geometry Box { size 1 1 1 } }
"#;
        let error = from_vrml(source, ()).unwrap_err();
        assert!(error.to_string().contains("geometry node Box"));
    }

    #[test]
    fn counts_ignored_line_geometry_beside_surface_geometry() {
        let source = br#"#VRML V2.0 utf8
Group { children [
  Shape { geometry IndexedLineSet {
    coord Coordinate { point [ 0 0 0, 1 1 0 ] }
    coordIndex [ 0 1 -1 ]
  } }
  Shape { geometry IndexedFaceSet {
    coord Coordinate { point [ 0 0 0, 1 0 0, 0 1 0 ] }
    coordIndex [ 0 1 2 -1 ]
  } }
] }
"#;
        let imported = from_vrml(source, ()).unwrap();
        assert_eq!(imported.ignored_non_mesh_geometry_count, 1);
        assert_eq!(imported.indexed_face_set_count, 1);
    }

    #[test]
    fn rejects_vrml_one() {
        let error = from_vrml(b"#VRML V1.0 ascii\nSeparator { }", ()).unwrap_err();
        assert!(error.to_string().contains("only VRML V2.0"));
    }
}
