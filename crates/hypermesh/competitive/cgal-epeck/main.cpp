#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/copy_face_graph.h>
#include <CGAL/boost/graph/helpers.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;
using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using Point = Kernel::Point_3;
using Mesh = CGAL::Surface_mesh<Point>;

enum class Operation {
  Union,
  Intersection,
  Difference,
  ReverseDifference,
  Xor,
  All,
};

struct BooleanOutputs {
  std::array<Mesh, 4> meshes;
  std::array<bool, 4> valid{};
  std::array<bool, 4> requested{};
};

static Kernel::FT exact_scalar(const std::string &token) {
  Kernel::FT value;
  std::istringstream input(token);
  input >> value;
  if (!input || !input.eof()) {
    throw std::runtime_error("invalid exact scalar: " + token);
  }
  return value;
}

static Mesh read_exact_off(const std::string &path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open exact OFF input: " + path);
  }

  std::string magic;
  input >> magic;
  if (magic != "OFF") {
    throw std::runtime_error("expected OFF header in " + path);
  }

  std::size_t vertex_count = 0;
  std::size_t face_count = 0;
  std::size_t edge_count = 0;
  input >> vertex_count >> face_count >> edge_count;
  static_cast<void>(edge_count);
  if (!input) {
    throw std::runtime_error("invalid OFF counts in " + path);
  }

  Mesh mesh;
  std::vector<Mesh::Vertex_index> vertices;
  vertices.reserve(vertex_count);
  for (std::size_t index = 0; index < vertex_count; ++index) {
    std::array<std::string, 3> coordinate;
    input >> coordinate[0] >> coordinate[1] >> coordinate[2];
    if (!input) {
      throw std::runtime_error("truncated OFF vertex list in " + path);
    }
    vertices.push_back(mesh.add_vertex(
        Point(exact_scalar(coordinate[0]), exact_scalar(coordinate[1]),
              exact_scalar(coordinate[2]))));
  }

  for (std::size_t face_index = 0; face_index < face_count; ++face_index) {
    std::size_t degree = 0;
    std::array<std::size_t, 3> triangle{};
    input >> degree;
    if (degree != triangle.size()) {
      throw std::runtime_error("CGAL adapter accepts triangular OFF only: " +
                               path);
    }
    input >> triangle[0] >> triangle[1] >> triangle[2];
    if (!input || triangle[0] >= vertex_count ||
        triangle[1] >= vertex_count || triangle[2] >= vertex_count) {
      throw std::runtime_error("invalid OFF triangle in " + path);
    }
    if (mesh.add_face(vertices[triangle[0]], vertices[triangle[1]],
                      vertices[triangle[2]]) == Mesh::null_face()) {
      throw std::runtime_error("OFF triangle violates Surface_mesh incidence: " +
                               path);
    }
  }

  if (!CGAL::is_triangle_mesh(mesh) || !CGAL::is_closed(mesh) ||
      !CGAL::is_valid_polygon_mesh(mesh) || !PMP::does_bound_a_volume(mesh)) {
    throw std::runtime_error("input does not satisfy the CGAL common contract: " +
                             path);
  }
  return mesh;
}

static Operation parse_operation(std::string_view name) {
  if (name == "union") {
    return Operation::Union;
  }
  if (name == "intersection") {
    return Operation::Intersection;
  }
  if (name == "difference") {
    return Operation::Difference;
  }
  if (name == "reverse-difference") {
    return Operation::ReverseDifference;
  }
  if (name == "xor") {
    return Operation::Xor;
  }
  if (name == "all") {
    return Operation::All;
  }
  throw std::runtime_error(
      "operation must be union, intersection, difference, reverse-difference, "
      "xor, or all");
}

static const char *operation_name(Operation operation) {
  switch (operation) {
    case Operation::Union:
      return "union";
    case Operation::Intersection:
      return "intersection";
    case Operation::Difference:
      return "difference";
    case Operation::ReverseDifference:
      return "reverse-difference";
    case Operation::Xor:
      return "xor";
    case Operation::All:
      return "all";
  }
  throw std::runtime_error("unreachable operation");
}

static BooleanOutputs run_boolean(Mesh &left, Mesh &right,
                                  Operation operation) {
  BooleanOutputs result;
  switch (operation) {
    case Operation::Union:
      result.requested[0] = true;
      break;
    case Operation::Intersection:
      result.requested[1] = true;
      break;
    case Operation::Difference:
      result.requested[2] = true;
      break;
    case Operation::ReverseDifference:
      result.requested[3] = true;
      break;
    case Operation::Xor:
      result.requested[2] = true;
      result.requested[3] = true;
      break;
    case Operation::All:
      result.requested.fill(true);
      break;
  }

  std::array<std::optional<Mesh *>, 4> output{};
  for (std::size_t index = 0; index < output.size(); ++index) {
    if (result.requested[index]) {
      output[index] = &result.meshes[index];
    }
  }
  result.valid = PMP::corefine_and_compute_boolean_operations(left, right, output);
  return result;
}

static void write_output_summary(const Mesh &mesh, bool valid) {
  const bool empty = mesh.is_empty();
  const bool closed = empty || CGAL::is_closed(mesh);
  const bool structurally_valid = CGAL::is_valid_polygon_mesh(mesh);
  double volume = 0.0;
  if (valid && !empty && closed) {
    volume = CGAL::to_double(PMP::volume(mesh));
  }
  std::cout << "{\"valid\":" << (valid ? "true" : "false")
            << ",\"closed\":" << (closed ? "true" : "false")
            << ",\"structurally_valid\":"
            << (structurally_valid ? "true" : "false")
            << ",\"vertices\":" << mesh.number_of_vertices()
            << ",\"triangles\":" << mesh.number_of_faces()
            << ",\"volume_f64\":" << volume << "}";
}

static void write_sample(const BooleanOutputs &outputs, Operation operation,
                         std::size_t repetition, std::int64_t elapsed_ns,
                         bool copy_inside) {
  static constexpr std::array<const char *, 4> names = {
      "union", "intersection", "difference", "reverse-difference"};
  std::cout << "{\"engine\":\"CGAL EPECK\",\"cgal_version\":\"6.0.3\""
            << ",\"operation\":\"" << operation_name(operation) << "\""
            << ",\"repetition\":" << repetition
            << ",\"copy_timing\":\"" << (copy_inside ? "inside" : "outside")
            << "\",\"elapsed_ns\":" << elapsed_ns << ",\"outputs\":{";
  bool first = true;
  for (std::size_t index = 0; index < outputs.requested.size(); ++index) {
    if (!outputs.requested[index]) {
      continue;
    }
    if (!first) {
      std::cout << ',';
    }
    first = false;
    std::cout << '\"' << names[index] << "\":";
    write_output_summary(outputs.meshes[index], outputs.valid[index]);
  }
  std::cout << "}}\n";
}

int main(int argc, char **argv) {
  try {
    if (argc != 6) {
      throw std::runtime_error(
          "usage: hypermesh_cgal_epeck <left.off> <right.off> <operation> "
          "<repetitions> <inside|outside>");
    }
    const Mesh left_input = read_exact_off(argv[1]);
    const Mesh right_input = read_exact_off(argv[2]);
    const Operation operation = parse_operation(argv[3]);
    const std::size_t repetitions = std::stoull(argv[4]);
    if (repetitions == 0) {
      throw std::runtime_error("repetitions must be positive");
    }
    const std::string_view copy_timing = argv[5];
    if (copy_timing != "inside" && copy_timing != "outside") {
      throw std::runtime_error("copy timing must be inside or outside");
    }
    const bool copy_inside = copy_timing == "inside";

    for (std::size_t repetition = 0; repetition < repetitions; ++repetition) {
      Mesh left;
      Mesh right;
      if (!copy_inside) {
        left = left_input;
        right = right_input;
      }
      const auto start = std::chrono::steady_clock::now();
      if (copy_inside) {
        left = left_input;
        right = right_input;
      }
      BooleanOutputs outputs = run_boolean(left, right, operation);
      const auto end = std::chrono::steady_clock::now();
      const auto elapsed =
          std::chrono::duration_cast<std::chrono::nanoseconds>(end - start)
              .count();
      write_sample(outputs, operation, repetition, elapsed, copy_inside);
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "hypermesh_cgal_epeck: " << error.what() << '\n';
    return 1;
  }
}
