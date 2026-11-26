//! Geometry-only VRML 2.0 (`.wrl`) mesh import.
//!
//! The importer targets the polygonal subset used by KiCad package models:
//! transform/group/shape scene graphs and `IndexedFaceSet` geometry. Rendering
//! attributes are intentionally ignored. Line/point nodes and decided-degenerate
//! surface pieces are omitted with counts; other unsupported geometry fails.
//! A scene with no surviving triangles is an error, not canonical empty output.

use crate::context::GeometryDecisions;
use crate::io::{IoError, triangulate_planar_face};
use crate::{GeometryContext, GeometryOutcome};
use hashbrown::HashMap;
use hyperlattice::{Matrix4, Point3, Real, Vector3};
use hypermesh::{Triangle, TriangleMesh};

/// Geometry evidence produced while flattening a VRML scene.
/// Counts describe traversed instances and selected-policy omissions, not closed
/// topology or fidelity to ignored rendering attributes. Geometry retains missing
/// source history and any consumed predicate qualification.
#[derive(Clone, Debug)]
pub struct VrmlMeshImport {
    /// Flattened native triangle mesh.
    pub mesh: hypermesh::TriangleMesh,
    /// Number of `Shape` nodes visited.
    pub shape_count: usize,
    /// Number of `IndexedFaceSet` nodes consumed.
    pub indexed_face_set_count: usize,
    /// Number of line or point geometry nodes intentionally omitted.
    pub ignored_non_mesh_geometry_count: usize,
    /// Number of polygons omitted after all first-vertex fan triples were decided
    /// degenerate, or shared triangulation returned no triangles.
    pub ignored_degenerate_polygon_count: usize,
    /// Number of emitted triangles omitted after degeneracy checks under the selected policy.
    pub ignored_degenerate_triangle_count: usize,
}

/// Imports polygonal geometry from a VRML 2.0 scene.
///
/// Decimal coordinates and transforms are parsed into [`Real`] before scene
/// flattening. `DEF`/`USE` reuse is supported for geometry and scene nodes.
/// Appearance, material, normal, color, and texture data are accepted but do
/// not cross this geometry-only boundary.
/// Uses [`from_vrml_with_context`] with strict decisions, consuming the separate
/// outcome while retaining embedded mesh qualification. Omission and nonempty
/// requirements are the same as that entry point.
pub fn from_vrml(bytes: &[u8]) -> Result<VrmlMeshImport, IoError> {
    from_vrml_with_context(bytes, GeometryContext::STRICT).map(GeometryOutcome::into_value)
}

/// Import a VRML scene with the selected mesh and triangulation policy.
/// Undecided degeneracy checks return an error instead of omitting geometry.
///
/// Requires a UTF-8 VRML V2.0 header. Geometry nodes are `IndexedFaceSet` with
/// `Coordinate` data, under recursive `Transform`/`Group`/`Shape` nodes. Numeric
/// text is parsed directly into exact scalars; axis-angle trigonometry remains
/// in native scalar expressions. Transform order is `T C R SR S SR⁻¹ C⁻¹`.
/// Each face-set instance appends all its transformed positions without welding;
/// `ccw FALSE` reverses face indices, but reflections do not trigger another reversal.
///
/// `DEF` stores an owned parsed subtree and `USE` clones an already defined one;
/// reuse is not shared scene-graph storage. Definitions inside skipped appearance
/// fields are not registered and skipped `USE` names are not resolved. Recognized
/// rendering fields are discarded rather than fully semantically validated.
/// Line/point geometry is counted and skipped. Larger polygons with no decided
/// nondegenerate fan triple are omitted; emitted triangles receive another native
/// degeneracy check. A completely empty/omitted scene returns `IoError::Geometry`.
///
/// Shared triangulation can fan nonplanar polygons. Success does not establish
/// closed topology, manifoldness or faithful rendering. Local predicate certainty
/// is returned separately from missing source history and inherited qualification.
/// Parser/transform/native geometry failures here retain text diagnostics rather
/// than typed nested causes. Parsing and flattening recurse, and subtree clones
/// can expand storage; no total depth, allocation or work limit is declared.
pub fn from_vrml_with_context(
    bytes: &[u8],
    context: GeometryContext,
) -> Result<GeometryOutcome<VrmlMeshImport>, IoError> {
    let decisions = GeometryDecisions::new(context);
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
        flatten_node(root, &identity, &mut flattened, &decisions)?;
    }
    if flattened.triangles.is_empty() {
        return Err(IoError::Geometry {
            format: "VRML",
            detail: "scene contains no triangulatable IndexedFaceSet geometry".into(),
        });
    }

    let mut native_triangles = Vec::with_capacity(flattened.triangles.len());
    let triangles = std::mem::take(&mut flattened.triangles);
    for [a, b, c] in triangles {
        let nondegenerate = hypermesh::Plane::points_are_nondegenerate(
            decisions.mesh_context(),
            &flattened.positions[a],
            &flattened.positions[b],
            &flattened.positions[c],
        )
        .map(|outcome| decisions.consume_mesh(outcome))
        .map_err(|error| IoError::Geometry {
            format: "VRML",
            detail: error.to_string(),
        })?;
        if !nondegenerate {
            flattened.ignored_degenerate_triangle_count = flattened
                .ignored_degenerate_triangle_count
                .checked_add(1)
                .ok_or(IoError::SizeOverflow {
                    format: "VRML",
                    limit: "ignored degenerate triangle count",
                })?;
            continue;
        }
        native_triangles.push(Triangle::new(a, b, c));
    }
    if native_triangles.is_empty() {
        return Err(IoError::Geometry {
            format: "VRML",
            detail: "scene contains no nondegenerate triangles under the selected policy"
                .into(),
        });
    }
    let mesh =
        decisions.qualify_mesh(TriangleMesh::new(flattened.positions, native_triangles));

    Ok(decisions.finish(VrmlMeshImport {
        mesh,
        shape_count: flattened.shape_count,
        indexed_face_set_count: flattened.indexed_face_set_count,
        ignored_non_mesh_geometry_count: flattened.ignored_non_mesh_geometry_count,
        ignored_degenerate_polygon_count: flattened.ignored_degenerate_polygon_count,
        ignored_degenerate_triangle_count: flattened.ignored_degenerate_triangle_count,
    }))
}

/// Lexical tokens retained for the geometry subset's cursor parser.
#[derive(Clone, Debug, PartialEq)]
enum Token {
    /// Unquoted node/field/name/number token, interpreted by its parser context.
    Word(
        /// Original token text, without separators.
        String,
    ),
    /// Quoted text retained so rendering bodies can be skipped safely.
    String(
        /// Text with each backslash removed before the following character.
        String,
    ),
    /// Opening node/body delimiter.
    LeftBrace,
    /// Closing node/body delimiter.
    RightBrace,
    /// Opening list delimiter.
    LeftBracket,
    /// Closing list delimiter.
    RightBracket,
}

/// Tokenizes whitespace/comma-separated words, quoted strings, delimiters and `#` comments.
/// Backslash quotes the next string character verbatim; no escape-code decoding
/// is performed. Unterminated strings/escapes fail. This does not validate full
/// VRML grammar or impose a token/depth/work bound.
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

/// Parsed geometry subset, including placeholders for deliberately omitted data.
/// Owned subtrees are cloned for DEF/USE; this is staging, not a geometry kernel.
#[derive(Clone, Debug)]
enum Node {
    /// Local transform and recursively instantiated children.
    Transform(
        /// Owned transform fields and child nodes.
        Box<TransformNode>,
    ),
    /// Children inheriting their parent's transform unchanged.
    Group(
        /// Child traversal order as parsed.
        Vec<Node>,
    ),
    /// Counted shape with an optional geometry child; appearance is discarded.
    Shape(
        /// Retained geometry node, or none when absent/initially NULL.
        Option<Box<Node>>,
    ),
    /// Polygon index rings over exact local coordinates awaiting triangulation.
    IndexedFaceSet {
        /// Complete authored position array, including unused coordinates.
        positions: Vec<Point3>,
        /// Nonnegative index rings split on -1; bounds/lengths are checked while flattening.
        polygons: Vec<Vec<usize>>,
        /// Authored winding flag; false reverses each index ring before triangulation.
        ccw: bool,
    },
    /// Reusable coordinate node, with no geometry emitted when visited alone.
    Coordinate(
        /// Exact local points grouped from scalar triples.
        Vec<Point3>,
    ),
    /// Skipped IndexedLineSet/PointSet body, counted when the node is flattened.
    NonMeshGeometry,
    /// Recognized rendering/scene metadata whose body was skipped.
    Ignored,
}

/// Exact VRML transform fields composed according to the authored center/orientation order.
#[derive(Clone, Debug)]
struct TransformNode {
    /// Pivot used by rotation and oriented scaling; defaults to the origin.
    center: [Real; 3],
    /// Axis-angle rotation applied after oriented scaling about the pivot.
    rotation: Rotation,
    /// Component scales in the scale-orientation frame; default one.
    scale: [Real; 3],
    /// Rotation selecting the scale frame, inverted on the right of scaling.
    scale_orientation: Rotation,
    /// Final local translation, applied after the pivot operations.
    translation: [Real; 3],
    /// Ordered child instances inheriting the composed transform.
    children: Vec<Node>,
}

/// Exact axis-angle fields awaiting checked native rotation construction.
#[derive(Clone, Debug)]
struct Rotation {
    /// Authored axis, normalized/validated by the matrix layer rather than the parser.
    axis: [Real; 3],
    /// Rotation angle in radians, retained as an exact scalar expression.
    angle: Real,
}

impl Default for Rotation {
    /// Supplies the zero-angle rotation about positive Z used by absent fields.
    fn default() -> Self {
        Self {
            axis: [Real::zero(), Real::zero(), Real::one()],
            angle: Real::zero(),
        }
    }
}

/// Recursive geometry-subset parser with one document-wide DEF name table.
struct Parser {
    /// Entire token stream, retained until parsing completes.
    tokens: Vec<Token>,
    /// Index of the next unconsumed token.
    cursor: usize,
    /// Fully parsed named subtrees; later definitions replace earlier entries.
    definitions: HashMap<String, Node>,
}

impl Parser {
    /// Starts before the first token with no registered definitions.
    fn new(tokens: Vec<Token>) -> Self {
        Self {
            tokens,
            cursor: 0,
            definitions: HashMap::new(),
        }
    }

    /// Parses ordered roots until token exhaustion; there is no separate document terminator.
    fn parse_document(&mut self) -> Result<Vec<Node>, IoError> {
        let mut roots = Vec::new();
        while self.peek().is_some() {
            roots.push(self.parse_node()?);
        }
        Ok(roots)
    }

    /// Parses a supported node or resolves DEF/USE with owned subtree copies.
    /// DEF registers only after its body succeeds; forward/self references cannot
    /// resolve unless an older definition already supplies that name. Recognized
    /// rendering bodies are skipped by brace balance, not validated by their grammar.
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

    /// Parses transform fields with identity defaults and last-value replacement.
    /// Bounding-box triples are parsed then discarded; unknown fields fail.
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

    /// Parses children and discarded bounding-box triples; repeated children replace the list.
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

    /// Parses optional geometry while skipping appearance without registering its definitions.
    /// Repeated non-NULL geometry replaces the prior child; NULL leaves it unchanged.
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

    /// Requires Coordinate data and coordIndex rings, retaining only winding and geometry.
    /// Rendering node fields are skipped; known scalar/list/boolean fields are
    /// parsed then discarded. Bounds and polygon lengths are deferred to flattening.
    /// Repeated values replace prior ones, except coord NULL does not clear a prior node.
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

    /// Parses the required bracketed point field into exact triples, allowing an empty array.
    /// Rejects incomplete triples/unknown fields; a repeated point field replaces its predecessor.
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
                            .as_chunks::<3>()
                            .0
                            .iter()
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

    /// Accepts NULL, a single node or a bracketed ordered list of parsed nodes.
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

    /// Skips a NULL, USE name or optionally DEF-prefixed braced node value.
    /// Does not resolve USE or register DEF, and checks neither node kind nor fields.
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

    /// Consumes through the closing brace after an already-consumed opening brace.
    /// Counts nested braces iteratively; brackets and field semantics are ignored.
    /// Quoted delimiters are string tokens. End of input before closure fails.
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

    /// Parses a bracketed list of signed i64 tokens without interpreting index separators.
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

    /// Accepts exactly TRUE/true or FALSE/false, rejecting mixed-case spellings.
    fn boolean(&mut self) -> Result<bool, IoError> {
        let text = self.word("boolean")?;
        match text.as_str() {
            "TRUE" | "true" => Ok(true),
            "FALSE" | "false" => Ok(false),
            _ => Err(malformed(format!("invalid boolean {text:?}"))),
        }
    }

    /// Reads three axis components and a radian angle, leaving axis validation to construction.
    fn rotation(&mut self) -> Result<Rotation, IoError> {
        let [x, y, z] = self.real3()?;
        Ok(Rotation {
            axis: [x, y, z],
            angle: self.real()?,
        })
    }

    /// Parses exactly three scalar tokens in component order.
    fn real3(&mut self) -> Result<[Real; 3], IoError> {
        Ok([self.real()?, self.real()?, self.real()?])
    }

    /// Parses a word through Real's exact text parser, retaining parse failures as context text.
    fn real(&mut self) -> Result<Real, IoError> {
        let text = self.word("number")?;
        text.parse()
            .map_err(|error| malformed(format!("invalid number {text:?}: {error}")))
    }

    /// Consumes one unquoted word or reports the encountered token/end with caller context.
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

    /// Consumes one token and requires exact token equality; mismatches are not rolled back.
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

    /// Advances only when the next token is the exact case-sensitive word requested.
    fn consume_word(&mut self, expected: &str) -> bool {
        if matches!(self.peek(), Some(Token::Word(word)) if word == expected) {
            self.cursor += 1;
            true
        } else {
            false
        }
    }

    /// Advances only on exact token equality, leaving nonmatches untouched.
    fn consume(&mut self, expected: &Token) -> bool {
        if self.peek() == Some(expected) {
            self.cursor += 1;
            true
        } else {
            false
        }
    }

    /// Borrows the next token without advancing, or returns none at end of input.
    fn peek(&self) -> Option<&Token> {
        self.tokens.get(self.cursor)
    }

    /// Clones and advances past one available token; exhaustion leaves the cursor unchanged.
    fn next(&mut self) -> Option<Token> {
        let token = self.tokens.get(self.cursor).cloned();
        self.cursor += usize::from(token.is_some());
        token
    }
}

/// Splits -1-delimited coordIndex rings, accepting a final ring without its terminator.
/// Empty rings/no rings and non-usize indices fail; minimum ring length and
/// coordinate-array bounds are checked later by flattening.
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

/// Accumulated transformed geometry and traversal/omission counts before final admission.
#[derive(Default)]
struct Flattened {
    /// Per-instance transformed positions, including unused or later-degenerate vertices.
    positions: Vec<Point3>,
    /// Shared-triangulator index triples pending the final per-triangle degeneracy pass.
    triangles: Vec<[usize; 3]>,
    /// Shape instances visited, including shapes without retained geometry.
    shape_count: usize,
    /// Face-set instances visited, including those whose polygons are all omitted.
    indexed_face_set_count: usize,
    /// Line/point placeholder instances deliberately omitted.
    ignored_non_mesh_geometry_count: usize,
    /// Rings omitted by fan nondegeneracy screening or empty triangulation output.
    ignored_degenerate_polygon_count: usize,
    /// Emitted triples omitted in the final native degeneracy pass.
    ignored_degenerate_triangle_count: usize,
}

/// Recursively instantiates one parsed node under `parent`, accumulating geometry and counts.
///
/// Checks face sizes/indices and applies ccw reversal after exact transformation.
/// Larger faces search first-vertex consecutive triples for decided nondegeneracy;
/// the first unresolved/native failure aborts, while all-degenerate rings are counted
/// and skipped. Shared triangulation supplies the surviving triples, with final
/// triangle degeneracy checked by the outer importer. Scratch output may be partial
/// on failure and is then discarded. Neither recursion nor clone expansion has a budget.
fn flatten_node(
    node: &Node,
    parent: &Matrix4,
    flattened: &mut Flattened,
    decisions: &GeometryDecisions,
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
                flatten_node(child, &world, flattened, decisions)?;
            }
        },
        Node::Group(children) => {
            for child in children {
                flatten_node(child, parent, flattened, decisions)?;
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
                flatten_node(geometry, parent, flattened, decisions)?;
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
                if face_indices.len() > 3 {
                    let points = face_indices
                        .iter()
                        .map(|&index| &flattened.positions[index])
                        .collect::<Vec<_>>();
                    let mut nondegenerate = false;
                    for index in 1..points.len() - 1 {
                        if hypermesh::Plane::points_are_nondegenerate(
                            decisions.mesh_context(),
                            points[0],
                            points[index],
                            points[index + 1],
                        )
                        .map_err(|error| IoError::Geometry {
                            format: "VRML",
                            detail: error.to_string(),
                        })
                        .map(|outcome| decisions.consume_mesh(outcome))?
                        {
                            nondegenerate = true;
                            break;
                        }
                    }
                    if !nondegenerate {
                        flattened.ignored_degenerate_polygon_count = flattened
                            .ignored_degenerate_polygon_count
                            .checked_add(1)
                            .ok_or(IoError::SizeOverflow {
                                format: "VRML",
                                limit: "ignored degenerate polygon count",
                            })?;
                        continue;
                    }
                }
                let face_triangles = triangulate_planar_face(
                    &flattened.positions,
                    &face_indices,
                    "VRML",
                    decisions,
                )?;
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
                    .extend(face_triangles.iter().map(|triangle| (*triangle).indices()));
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

/// Composes the exact affine VRML transform `T C R SR S SR⁻¹ C⁻¹`.
/// Rotations are checked by the matrix layer. Zero/negative scales are not
/// independently rejected here; transformed faces undergo later geometry checks.
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

/// Delegates exact axis-angle construction, retaining native failures as VRML geometry text.
/// Uses checked scalar/matrix rules rather than the selected triangulation policy.
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

/// Supplies the exact zero triple for absent center and translation fields.
fn zero3() -> [Real; 3] {
    [Real::zero(), Real::zero(), Real::zero()]
}

/// Supplies the exact identity scale triple.
fn one3() -> [Real; 3] {
    [Real::one(), Real::one(), Real::one()]
}

/// Adds the VRML label to a text-only malformed-input diagnostic.
fn malformed(message: impl Into<String>) -> IoError {
    IoError::MalformedInput(format!("VRML: {}", message.into()))
}

#[cfg(test)]
/// Parser, transform, omission and triangulation-policy regression witnesses.
mod tests {
    use super::from_vrml;
    use hyperlattice::Real;

    #[test]
    /// Checks parsed render-data omission, counts and one independently expected transformed corner.
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
        let imported = from_vrml(source).unwrap();
        assert_eq!(imported.shape_count, 1);
        assert_eq!(imported.indexed_face_set_count, 1);
        assert_eq!(imported.ignored_non_mesh_geometry_count, 0);
        assert_eq!(imported.ignored_degenerate_polygon_count, 0);
        assert_eq!(imported.ignored_degenerate_triangle_count, 0);
        assert_eq!(imported.mesh.triangles.len(), 2);
        assert!(imported.mesh.positions.iter().any(|point| {
            point.x == Real::from(12) && point.y == Real::from(23) && point.z == Real::from(30)
        }));
    }

    #[test]
    /// Checks Coordinate DEF/USE and the exact negative-Z normal induced by ccw FALSE.
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
        let imported = from_vrml(source).unwrap();
        assert_eq!(imported.mesh.triangles.len(), 1);
        let [a, b, c] = imported.mesh.triangles[0].indices();
        let ab = &imported.mesh.positions[b] - &imported.mesh.positions[a];
        let ac = &imported.mesh.positions[c] - &imported.mesh.positions[a];
        let normal = ab.cross(&ac);
        assert_eq!(normal.0[2], Real::from(-1));
    }

    #[test]
    /// Rejects a Box geometry node with an explicit unsupported-geometry diagnostic.
    fn rejects_non_indexed_face_geometry() {
        let source = br#"#VRML V2.0 utf8
Shape { geometry Box { size 1 1 1 } }
"#;
        let error = from_vrml(source).unwrap_err();
        assert!(error.to_string().contains("geometry node Box"));
    }

    #[test]
    /// Counts a skipped line-set instance beside a retained face-set instance.
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
        let imported = from_vrml(source).unwrap();
        assert_eq!(imported.ignored_non_mesh_geometry_count, 1);
        assert_eq!(imported.indexed_face_set_count, 1);
    }

    #[test]
    /// Separates skipped lines, degenerate polygon rings and final degenerate triangles in counts.
    fn retains_valid_faces_while_counting_degenerate_geometry() {
        let source = br#"#VRML V2.0 utf8
Group { children [
  Shape { geometry IndexedLineSet {
    coord Coordinate { point [ 0 0 0, 1 1 0 ] }
    coordIndex [ 0 1 -1 ]
  } }
  Shape { geometry IndexedFaceSet {
    coord Coordinate { point [ 0 0 0, 1 0 0, 0 1 0, 2 0 0, 3 0 0 ] }
    coordIndex [ 0 1 2 -1, 0 0 0 -1, 0 1 3 4 -1 ]
  } }
] }
"#;
        let imported = from_vrml(source).unwrap();

        assert_eq!(imported.mesh.triangles.len(), 1);
        assert_eq!(imported.ignored_non_mesh_geometry_count, 1);
        assert_eq!(imported.ignored_degenerate_polygon_count, 1);
        assert_eq!(imported.ignored_degenerate_triangle_count, 1);
    }

    /// Exercises near-tied normal components under X/Z rotations near pi/4.
    /// Checks certified triangulation in both policies and exact aggregate vector
    /// area. This does not independently assert full triangle coverage/nonoverlap
    /// or that the component-order predicate itself remains unresolved.
    #[test]
    fn unordered_normal_components_still_project_with_certified_nonzero_axis() {
        use crate::{GeometryCertainty, GeometryContext};
        use hyperlattice::{Point3, Vector3};

        /// Long exact decimal near pi/4, used to make normal-component ordering difficult.
        const NEAR_QUARTER_TURN: &str = "0.785398163397448309615660845819875721049292349843776455243736148076954101571552249657008706335529266995537021628320576661773461152387645557931339852032120279362571025675484630276389911155737238732595";
        // A concave pentagon of area 3, so ear clipping must choose diagonals.
        let points = [[0, 0], [2, 0], [1, 1], [2, 2], [0, 2]];
        for (axis, plane_points) in [
            ("1 0 0", points.map(|[x, y]| format!("{x} {y} 0"))),
            ("0 0 1", points.map(|[x, z]| format!("{x} 0 {z}"))),
        ] {
            let source = format!(
                "#VRML V2.0 utf8\nTransform {{ rotation {axis} {NEAR_QUARTER_TURN} children [\n\
                 Shape {{ geometry IndexedFaceSet {{\n\
                 coord Coordinate {{ point [ {} ] }}\n\
                 coordIndex [ 0 1 2 3 4 -1 ] }} }} ] }}\n",
                plane_points.join(", ")
            );
            for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
                let outcome =
                    super::from_vrml_with_context(source.as_bytes(), context).unwrap();
                assert_eq!(outcome.certainty, GeometryCertainty::Certified);
                let mesh = outcome.value.mesh;
                assert_eq!(mesh.triangles.len(), 3);
                // Compare aggregate oriented vector area. Equality alone does
                // not establish complete coverage or absence of overlaps.
                let cross = |a: &Point3, b: &Point3| a.to_vector().cross(&b.to_vector());
                let mut boundary = Vector3::zero();
                for index in 0..mesh.positions.len() {
                    let next = (index + 1) % mesh.positions.len();
                    boundary = boundary + cross(&mesh.positions[index], &mesh.positions[next]);
                }
                let mut covered = Vector3::zero();
                for triangle in mesh.triangles.iter() {
                    let [a, b, c] = triangle.indices().map(|index| &mesh.positions[index]);
                    covered = covered + cross(a, b) + cross(b, c) + cross(c, a);
                }
                for (covered, boundary) in covered.0.iter().zip(&boundary.0) {
                    assert_eq!(
                        hyperlimit::compare_reals(
                            covered,
                            boundary,
                            hyperlimit::PredicatePolicy::STRICT
                        )
                        .value(),
                        Some(std::cmp::Ordering::Equal)
                    );
                }
            }
        }
    }

    #[test]
    /// Rejects a VRML 1.0 header before attempting to parse its scene grammar.
    fn rejects_vrml_one() {
        let error = from_vrml(b"#VRML V1.0 ascii\nSeparator { }").unwrap_err();
        assert!(error.to_string().contains("only VRML V2.0"));
    }
}
