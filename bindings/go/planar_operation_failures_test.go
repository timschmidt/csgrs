package csgrs

import (
	"errors"
	"math"
	"strings"
	"testing"
)

func TestPlanarTranslationAdmissionAndRecovery(t *testing.T) {
	for _, width := range []float64{0, 1} {
		source, err := CurveRegionF64Square(width)
		if err != nil {
			t.Fatal(err)
		}
		defer source.Free()
		for _, rejected := range []struct {
			z      float64
			status int
		}{{1, 7}, {math.NaN(), 3}} {
			result, err := source.TranslateF64(0, 0, rejected.z)
			var status *StatusError
			if result != nil || !errors.As(err, &status) || int(status.Status) != rejected.status || status.Message == "" {
				t.Fatalf("lost planar transform error: %v, %v", result, err)
			}
			if rejected.status == 7 && !strings.Contains(status.Message, "XY plane") {
				t.Fatal(status.Message)
			}
		}
		moved, err := source.TranslateF64(2, -3, 0)
		if err != nil {
			t.Fatal(err)
		}
		defer moved.Free()
		bounds, err := moved.BoundingBoxF64()
		if err != nil {
			t.Fatal(err)
		}
		if width == 0 {
			if bounds != nil {
				t.Fatal("empty transform acquired bounds")
			}
		} else if bounds == nil || bounds.mins.x != 2 || bounds.maxs.x != 3 || bounds.mins.y != -3 || bounds.maxs.y != -2 {
			t.Fatalf("wrong translated set: %v", bounds)
		}
	}
}
