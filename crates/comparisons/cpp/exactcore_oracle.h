#ifndef EXACTCORE_HYPER_COMPARISON_ORACLE_H
#define EXACTCORE_HYPER_COMPARISON_ORACLE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* All string-returning functions return the byte count, or a negative status. */
int ec_rational_binary(int operation, const char *left, const char *right, char *out, size_t out_len);
int ec_rational_unary(int operation, const char *value, char *out, size_t out_len);
int ec_rational_compare(const char *left, const char *right);
int ec_rational_parts(const char *value, char *numerator, size_t numerator_len, char *denominator, size_t denominator_len);

double ec_expr_unary(int operation, const char *value, uint32_t root_degree);
double ec_expr_binary(int operation, const char *left, const char *right, int64_t integer_parameter);
double ec_expr_constant(int operation);
int ec_expr_sign_unary(int operation, const char *value, uint32_t root_degree);
int ec_expr_floor_ceil(int operation, const char *value, char *out, size_t out_len);

int ec_complex_binary(int operation, const char *ar, const char *ai, const char *br, const char *bi,
                      char *real_out, size_t real_len, char *imag_out, size_t imag_len);
int ec_complex_norm_squared(const char *real, const char *imag, char *out, size_t out_len);

double ec_vector_dot(uint32_t dimension, const int64_t *left, const int64_t *right);
int ec_vector_binary(int operation, uint32_t dimension, const int64_t *left, const int64_t *right, double *out);
int ec_vector_cross3(const int64_t *left, const int64_t *right, double *out);
double ec_vector_norm(uint32_t dimension, const int64_t *values);
double ec_matrix_determinant(uint32_t dimension, const int64_t *row_major);
int ec_matrix_binary(int operation, uint32_t dimension, const int64_t *left, const int64_t *right, double *out);
int ec_matrix_transpose(uint32_t dimension, const int64_t *values, double *out);
int ec_matrix_adjugate(uint32_t dimension, const int64_t *values, double *determinant, double *out);

int ec_orientation2(const int64_t *coordinates);
double ec_area2(const int64_t *coordinates);
int ec_between2(const int64_t *coordinates);
int ec_line2_relation(int operation, const int64_t *coordinates);
int ec_line2_intersection(const int64_t *coordinates, double *output);
int ec_segment2_relation(int operation, const int64_t *coordinates);
int ec_incircle2(const int64_t *coordinates);
int ec_circle2_relation(int operation, int64_t radius, const int64_t *coordinates);
double ec_circle2_distance(int operation, int64_t first_radius, int64_t second_radius,
                           const int64_t *coordinates);
double ec_point2_distance(const int64_t *coordinates);
double ec_line2_point_distance(const int64_t *coordinates);
double ec_segment2_point_distance(const int64_t *coordinates);

int ec_orientation3(const int64_t *coordinates);
double ec_volume3(const int64_t *coordinates);
int ec_line3_relation(int operation, const int64_t *coordinates);
int ec_segment3_relation(int operation, const int64_t *coordinates);
int ec_plane3_relation(int operation, const int64_t *coordinates);
double ec_point3_distance(const int64_t *coordinates);
double ec_line3_point_distance(const int64_t *coordinates);
double ec_segment3_point_distance(const int64_t *coordinates);
double ec_plane3_point_distance(const int64_t *coordinates);
int ec_triangle3_relation(int operation, const int64_t *coordinates);

int ec_polynomial_eval(const int64_t *coefficients, size_t coefficient_count, int64_t x, char *out, size_t out_len);
int ec_polynomial_binary_eval(int operation, const int64_t *left, size_t left_count,
                              const int64_t *right, size_t right_count, int64_t x,
                              char *out, size_t out_len);
int ec_polynomial_derivative_eval(const int64_t *coefficients, size_t coefficient_count,
                                  uint32_t order, int64_t x, char *out, size_t out_len);
int ec_polynomial_resultant(const int64_t *left, size_t left_count,
                            const int64_t *right, size_t right_count, char *out, size_t out_len);
int ec_polynomial_discriminant(const int64_t *coefficients, size_t coefficient_count,
                               char *out, size_t out_len);
int ec_polynomial_gcd_degree(const int64_t *left, size_t left_count,
                             const int64_t *right, size_t right_count);
int ec_polynomial_square_free_degree(const int64_t *coefficients, size_t coefficient_count);
int ec_polynomial_root_count(const int64_t *coefficients, size_t coefficient_count);
int ec_polynomial_root_count_interval(const int64_t *coefficients, size_t coefficient_count,
                                      int64_t lower, int64_t upper);
int ec_polynomial_isolate_roots(const int64_t *coefficients, size_t coefficient_count,
                                double *interval_pairs, size_t pair_capacity);

int ec_bivariate_eval(const int64_t *coefficients, size_t x_count, size_t y_count,
                      int64_t x, int64_t y, char *out, size_t out_len);
int ec_bivariate_resultant(int eliminate_y, const int64_t *left, const int64_t *right,
                           size_t x_count, size_t y_count, int64_t retained_value,
                           char *out, size_t out_len);

/* Independent O(n^4) empty-circumcircle enumeration using exactCore scalars. */
int ec_delaunay_dt4(const int64_t *xy, size_t point_count, uint32_t *triangle_indices,
                    size_t triangle_capacity);

#ifdef __cplusplus
}
#endif

#endif
