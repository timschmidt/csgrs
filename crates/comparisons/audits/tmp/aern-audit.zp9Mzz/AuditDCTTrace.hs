module AuditDCTTrace (trace) where
-- Benchmark-only omission of diagnostic string construction, not arithmetic.
trace :: String -> a -> a
trace _ x = x
