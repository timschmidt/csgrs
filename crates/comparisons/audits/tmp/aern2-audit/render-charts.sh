#!/bin/bash
set -euo pipefail
audit_repo=/home/tim/Documents/GitHub/workspace/exact-real-references/aern2
audit_out=$(mktemp -d /tmp/aern2-chart-review.XXXXXX)
audit_count=0
audit_page=0
audit_tiles=()
while IFS= read -r audit_file; do
    audit_name=$(basename "$audit_file" .pdf)
    audit_number=$(printf '%03d' "$audit_count")
    pdftoppm -f 1 -singlefile -scale-to 650 -png "$audit_repo/$audit_file" "$audit_out/$audit_number"
    audit_tiles+=(-label "$audit_file" "$audit_out/$audit_number.png")
    audit_count=$((audit_count+1))
    if ((audit_count % 12 == 0)); then
        montage -font DejaVu-Sans -pointsize 9 "${audit_tiles[@]}" -tile 3x4 -geometry 650x380+3+8 -background white "$audit_out/sheet-$audit_page.png"
        audit_tiles=()
        audit_page=$((audit_page+1))
    fi
done < <(jq -r '.validations[]|select(.kind=="PDF inventory" and .detail.pages==1)|.file' /tmp/aern2-artifact-inventory.json)
if ((${#audit_tiles[@]})); then
    montage -font DejaVu-Sans -pointsize 9 "${audit_tiles[@]}" -tile 3x4 -geometry 650x380+3+8 -background white "$audit_out/sheet-$audit_page.png"
fi
printf '%s\n' "$audit_out"
