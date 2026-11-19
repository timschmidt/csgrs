;; This module exercises the disassembly check. It is separate from the client.
(module
  (func (export "scalar") (param f32 f32) (result f32)
    (f32.add (local.get 0) (local.get 1)))
  (func (export "packed") (param v128 v128) (result v128)
    (f32x4.add (local.get 0) (local.get 1)))
  (func (export "convert") (param v128) (result v128)
    (i32x4.trunc_sat_f64x2_s_zero (f64x2.convert_low_i32x4_s (local.get 0))))
  (func (export "integer") (param v128 v128) (result v128)
    (i32x4.add (local.get 0) (local.get 1)))
  ;; The export name contains an instruction spelling. Its body only copies bits.
  (func (export "(f64x2.div)") (param i64) (result i64)
    (i64.reinterpret_f64
      (f64x2.extract_lane 0 (f64x2.splat (f64.reinterpret_i64 (local.get 0)))))))
