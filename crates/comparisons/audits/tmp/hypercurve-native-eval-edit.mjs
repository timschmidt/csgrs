import fs from 'node:fs';

const edits = [
  ['hypersolve/src/algebraic_fiber.rs', 'evaluate_real_polynomial', 'Real::eval_poly'],
  ['hypersolve/src/algebraic_rational_image.rs', 'evaluate_real_polynomial', 'Real::eval_poly'],
  ['hypersolve/src/root_isolation.rs', 'refinement_sign_at', 'sign_at'],
  ['hypercurve/src/bezier_algebraic_image.rs', 'evaluate_power_polynomial', 'Real::eval_poly'],
  ['hypercurve/src/rational_bezier_general.rs', 'evaluate_power_polynomial', 'Real::eval_poly'],
  ['hypercurve/src/bezier_moment.rs', 'evaluate_polynomial', 'Real::eval_poly'],
];

let patch = '*** Begin Patch\n';
for (const [file, name, replacement] of edits) {
  const source = fs.readFileSync(file, 'utf8');
  const definition = new RegExp(`^fn ${name}\\([\\s\\S]*?^}\\n\\n`, 'm');
  const match = definition.exec(source);
  if (!match) throw new Error(`Missing definition: ${file}: ${name}`);
  const first = source.slice(0, match.index).split('\n').length - 1;
  const count = match[0].split('\n').length - 1;
  const lines = source.split('\n');
  const call = new RegExp(`\\b${name}(?=\\()`, 'g');
  patch += `*** Update File: ${file}\n`;
  let replacements = 0;
  for (let index = 0; index < lines.length; ++index) {
    if (index === first) {
      patch += '@@\n' + lines.slice(index, index + count).map(line => '-' + line + '\n').join('');
      index += count - 1;
      continue;
    }
    const updated = lines[index].replace(call, replacement);
    if (updated !== lines[index]) {
      patch += `@@\n-${lines[index]}\n+${updated}\n`;
      ++replacements;
    }
  }
  if (replacements === 0) throw new Error(`No consumers: ${file}`);
}
process.stdout.write(patch + '*** End Patch\n');
