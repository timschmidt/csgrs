"""Read every nonempty cell/formula from the two audited ODS artifacts.

Archive extraction is already complete; no spreadsheet formulas are executed.
"""
import pathlib
import sys
import xml.etree.ElementTree as ET

ns = {
    'o': 'urn:oasis:names:tc:opendocument:xmlns:office:1.0',
    't': 'urn:oasis:names:tc:opendocument:xmlns:table:1.0',
    'p': 'urn:oasis:names:tc:opendocument:xmlns:text:1.0',
}
def attr(node, space, name, default=None):
    return node.get('{'+ns[space]+'}'+name, default)

for dirname in sys.argv[1:]:
    root = pathlib.Path(dirname)
    print('ARTIFACT', root.name)
    print('META', (root/'meta.xml').read_text())
    doc = ET.parse(root/'content.xml')
    for sheet in doc.findall('.//t:table', ns):
        print('SHEET', attr(sheet,'t','name'))
        r = 0
        for row in sheet.findall('t:table-row', ns):
            nrows = int(attr(row,'t','number-rows-repeated','1'))
            r += 1
            c = 0
            for cell in row:
                count = int(attr(cell,'t','number-columns-repeated','1'))
                c += 1
                paras = [''.join(p.itertext()) for p in cell.findall('p:p',ns)]
                value = attr(cell,'o','value')
                formula = attr(cell,'t','formula')
                if paras or value is not None or formula is not None:
                    print('R'+str(r)+'C'+str(c), 'rows='+str(nrows),
                          'cols='+str(count), repr(paras),
                          'value='+str(value), 'formula='+str(formula))
                c += count-1
            r += nrows-1
