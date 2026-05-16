#!/usr/bin/env python3
"""
gen_cli_docs.py — Generate docs/CLI/parameters-reference.md from settings.c

Parses valueTable[], lookupTables[], and section comments from settings.c/h
to produce a complete, always-current CLI parameter reference in Markdown.

No external dependencies (Python 3.6+ stdlib only).

Usage (from repo root):
    python3 docs/gen_cli_docs.py
    python3 docs/gen_cli_docs.py --settings src/main/interface/settings.c
    python3 docs/gen_cli_docs.py --output docs/CLI/parameters-reference.md
"""

import re
import sys
import argparse
import subprocess
from pathlib import Path
from datetime import date
from collections import OrderedDict


# ---------------------------------------------------------------------------
# PG group → human-readable section name (derived, not hardcoded)
# ---------------------------------------------------------------------------

# Known tokens to preserve in uppercase when converting PG_xxx names.
# Words containing digits are kept as-is automatically (e.g. 3D, MAX7456).
_ACRONYMS = {
    'ADC', 'CRSF', 'DMA', 'ESC', 'GPS', 'IMU', 'LED', 'LPF', 'MSP',
    'OSD', 'PID', 'PWM', 'RC', 'RPM', 'RTC', 'RX', 'SDCARD', 'SDIO',
    'SPI', 'TX', 'USB', 'VCD', 'VTX',
}


def pg_to_human(pg_name):
    """Derive a human-readable section name from a PG_xxx constant name.

    Examples:
        PG_GYRO_CONFIG           -> 'Gyro Config'
        PG_PID_PROFILE           -> 'PID Profile'
        PG_GPS_RESCUE            -> 'GPS Rescue'
        PG_CONTROL_RATE_PROFILES -> 'Control Rate Profiles'
        PG_MOTOR_3D_CONFIG       -> 'Motor 3D Config'
        PG_SDCARD_CONFIG         -> 'SDCARD Config'
    """
    name = pg_name[3:] if pg_name.startswith('PG_') else pg_name  # strip PG_
    parts = name.split('_')

    def _word(p):
        if p in _ACRONYMS:
            return p
        if any(c.isdigit() for c in p):  # e.g. 3D, MAX7456 — keep as-is
            return p
        return p.capitalize()

    return ' '.join(_word(p) for p in parts)


# ---------------------------------------------------------------------------
# Lookup table parsing
# ---------------------------------------------------------------------------

def strip_block_comments(text):
    return re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)


def parse_lookup_defs(text):
    """Returns dict: variable_name -> [string_values] from all const char* const arrays.

    Handles both unsized (name[]) and sized (name[COUNT]) array declarations.
    """
    tables = {}
    pattern = re.compile(
        r'(?:static\s+)?const\s+char\s*\*\s*const\s+(\w+)\s*\[[^\]]*\]\s*=\s*\{([^}]+)\}',
        re.DOTALL
    )
    for m in pattern.finditer(text):
        values = re.findall(r'"([^"]*)"', m.group(2))
        if values:
            tables[m.group(1)] = values
    return tables


def parse_enum_members(text, enum_name):
    """Returns ordered list of enum member names, stripping #ifdef guard lines."""
    m = re.search(
        r'typedef\s+enum\s*\{([^}]+)\}\s*' + enum_name + r'\s*;',
        text, re.DOTALL
    )
    if not m:
        return []
    members = []
    for line in m.group(1).splitlines():
        s = line.strip()
        if not s or s.startswith(('#', '//', '*')):
            continue
        em = re.match(r'([A-Z_][A-Z0-9_]+)', s)
        if em and em.group(1) != 'LOOKUP_TABLE_COUNT':
            members.append(em.group(1))
    return members


def parse_lookup_table_array(text):
    """Returns ordered list of variable names from lookupTables[] = { LOOKUP_TABLE_ENTRY(x)... }"""
    m = re.search(
        r'const lookupTableEntry_t lookupTables\[\]\s*=\s*\{(.+?)^\};',
        text, re.DOTALL | re.MULTILINE
    )
    if not m:
        return []
    entries = []
    for line in m.group(1).splitlines():
        s = line.strip()
        if s.startswith(('#', '//')):
            continue
        em = re.search(r'LOOKUP_TABLE_ENTRY\((\w+)\)', s)
        if em:
            entries.append(em.group(1))
    return entries


def build_table_map(h_text, c_text, lookup_defs):
    """Returns dict: TABLE_xxx -> [string_values]."""
    enum_members = parse_enum_members(h_text, 'lookupTableIndex_e')
    lookup_array = parse_lookup_table_array(c_text)
    result = {}
    for i, name in enumerate(enum_members):
        if i < len(lookup_array):
            var = lookup_array[i]
            result[name] = lookup_defs.get(var, [])
        else:
            result[name] = []
    return result


# ---------------------------------------------------------------------------
# valueTable[] parsing
# ---------------------------------------------------------------------------

def parse_value_table(c_text):
    """
    Returns list of entry dicts, tracking PG section comments and #ifdef conditions.
    Each dict: name, var_type, scope, mode, table_name, min, max, array_len, bitpos, pg, ifdef_conds
    """
    start = c_text.find('const clivalue_t valueTable[] = {')
    if start == -1:
        sys.exit("ERROR: cannot find 'const clivalue_t valueTable[]' in settings.c")

    end_marker = '\nconst uint16_t valueTableEntryCount'
    end = c_text.find(end_marker, start)
    block = c_text[start: end if end != -1 else len(c_text)]

    entries = []
    current_pg = 'UNKNOWN'
    ifdef_stack = []
    pending = ''
    pending_pg = 'UNKNOWN'
    pending_conds = []

    for raw_line in block.splitlines():
        line = raw_line.strip()

        # Track PG section comments
        pg_m = re.search(r'//\s*(PG_\w+)', line)
        if pg_m:
            current_pg = pg_m.group(1)
            continue

        # Preprocessor directives
        if re.match(r'#ifdef\s+(\w+)', line):
            ifdef_stack.append(re.match(r'#ifdef\s+(\w+)', line).group(1))
            continue
        if re.match(r'#ifndef\s+(\w+)', line):
            ifdef_stack.append('!' + re.match(r'#ifndef\s+(\w+)', line).group(1))
            continue
        if re.match(r'#if\s+', line):
            ifdef_stack.append(line[3:].strip())
            continue
        if line.startswith('#endif'):
            if ifdef_stack:
                ifdef_stack.pop()
            continue
        if line.startswith('#else'):
            if ifdef_stack:
                top = ifdef_stack[-1]
                ifdef_stack[-1] = top[1:] if top.startswith('!') else ('!' + top)
            continue
        if re.match(r'#elif\s', line):
            if ifdef_stack:
                ifdef_stack.pop()
            ifdef_stack.append(line[5:].strip())
            continue

        # Accumulate entry lines
        if line.startswith('{ "') and not pending:
            pending = line
            pending_pg = current_pg
            pending_conds = list(ifdef_stack)
        elif pending:
            pending += ' ' + line

        # Detect complete entry: has offsetof( and ends with }
        if pending and 'offsetof(' in pending:
            stripped = pending.rstrip().rstrip(',').rstrip()
            if stripped.endswith('}'):
                entry = _parse_entry(pending, pending_pg, pending_conds)
                if entry:
                    entries.append(entry)
                pending = ''
                pending_pg = 'UNKNOWN'
                pending_conds = []

    return entries


def _parse_entry(text, pg, ifdef_conds):
    """Parse a single valueTable entry string into a dict. Returns None on failure."""
    m = re.match(r'\{\s*"(\w+)"\s*,\s*(.+)', text, re.DOTALL)
    if not m:
        return None
    name = m.group(1)
    rest = m.group(2)

    config_idx = rest.find('.config')
    if config_idx == -1:
        return None

    flags_str = rest[:config_idx].rstrip(',').strip()
    config_part = rest[config_idx:]

    # Override pg from inline PG_xxx token if present
    pg_m = re.search(r'\b(PG_\w+)\b', config_part)
    if pg_m:
        pg = pg_m.group(1)

    var_type = _decode_type(flags_str)
    scope = _decode_scope(flags_str)
    mode, table_name, min_val, max_val, array_len, bitpos = _decode_config(flags_str, config_part)

    return {
        'name':        name,
        'var_type':    var_type,
        'scope':       scope,
        'mode':        mode,
        'table_name':  table_name,
        'min':         min_val,
        'max':         max_val,
        'array_len':   array_len,
        'bitpos':      bitpos,
        'pg':          pg,
        'ifdef_conds': ifdef_conds,
    }


def _decode_type(flags):
    for token, label in [
        ('VAR_UINT32', 'uint32'), ('VAR_INT16', 'int16'), ('VAR_UINT16', 'uint16'),
        ('VAR_INT8', 'int8'), ('VAR_UINT8', 'uint8'),
    ]:
        if token in flags:
            return label
    return 'uint8'


def _decode_scope(flags):
    if 'PROFILE_RATE_VALUE' in flags:
        return 'rate'
    if 'PROFILE_VALUE' in flags:
        return 'profile'
    return 'master'


def _decode_config(flags_str, config_part):
    """Returns (mode, table_name, min, max, array_len, bitpos)."""
    mode = 'direct'
    table_name = min_val = max_val = array_len = bitpos = None

    if 'MODE_LOOKUP' in flags_str:
        mode = 'lookup'
        m = re.search(r'\.config\.lookup\s*=\s*\{\s*(\w+)\s*\}', config_part)
        if m:
            table_name = m.group(1)

    elif 'MODE_ARRAY' in flags_str:
        mode = 'array'
        m = re.search(r'\.config\.array\.length\s*=\s*(\w+)', config_part)
        if m:
            array_len = m.group(1)

    elif 'MODE_BITSET' in flags_str:
        mode = 'bitset'
        m = re.search(r'\.config\.bitpos\s*=\s*(\w+)', config_part)
        if m:
            bitpos = m.group(1)

    else:
        m = re.search(r'\.config\.minmax\s*=\s*\{\s*([^,}]+),\s*([^}]+)\}', config_part)
        if m:
            min_val = m.group(1).strip()
            max_val = m.group(2).strip()

    return mode, table_name, min_val, max_val, array_len, bitpos


# ---------------------------------------------------------------------------
# Markdown generation
# ---------------------------------------------------------------------------

def _anchor(text):
    """Convert section heading to GitHub-flavored markdown anchor."""
    return re.sub(r'[^a-z0-9-]', '', text.lower().replace(' ', '-'))


def _format_range(entry, table_map):
    mode = entry['mode']
    if mode == 'lookup':
        tn = entry['table_name'] or ''
        values = table_map.get(tn, [])
        if values:
            return ', '.join(f'`{v}`' for v in values)
        return f'*{tn}*'
    if mode == 'direct':
        mn, mx = entry['min'], entry['max']
        if mn is not None and mx is not None:
            return f'`{mn}` – `{mx}`'
        return ''
    if mode == 'array':
        return f'array\\[{entry["array_len"]}\\]'
    if mode == 'bitset':
        return 'bitflag'
    return ''


def _format_requires(ifdef_conds):
    """Return only the simple USE_xxx conditions as a readable string."""
    simple = [c for c in ifdef_conds if re.match(r'USE_\w+$', c)]
    if not simple:
        return ''
    return ', '.join(f'`{c}`' for c in dict.fromkeys(simple))  # deduplicated, ordered


def generate_markdown(entries, table_map, settings_c_path, git_hash=None):
    today = date.today().isoformat()
    ref = git_hash or 'unknown'

    # Group by PG, preserving first-seen order
    sections = OrderedDict()
    for e in entries:
        sections.setdefault(e['pg'], []).append(e)

    lines = [
        '# CLI Parameters Reference',
        '',
        '> **Auto-generated** — do not edit manually.',
        f'> Source: `{settings_c_path}` | Generated: {today} | Commit: `{ref}`',
        '',
        '---',
        '',
        '## Table of Contents',
        '',
    ]

    for pg, pg_entries in sections.items():
        human = pg_to_human(pg)
        lines.append(f'- [{human}](#{_anchor(human)})')

    lines += ['', '---', '']

    for pg, pg_entries in sections.items():
        human = pg_to_human(pg)
        lines.append(f'## {human}')
        lines.append('')

        has_requires = any(_format_requires(e['ifdef_conds']) for e in pg_entries)

        if has_requires:
            lines.append('| Parameter | Type | Scope | Range / Values | Requires |')
            lines.append('|-----------|------|-------|----------------|----------|')
        else:
            lines.append('| Parameter | Type | Scope | Range / Values |')
            lines.append('|-----------|------|-------|----------------|')

        for e in pg_entries:
            name  = f'`{e["name"]}`'
            vtype = e['var_type']
            scope = e['scope']
            rng   = _format_range(e, table_map)
            if has_requires:
                req = _format_requires(e['ifdef_conds'])
                lines.append(f'| {name} | {vtype} | {scope} | {rng} | {req} |')
            else:
                lines.append(f'| {name} | {vtype} | {scope} | {rng} |')

        lines.append('')

    lines += [
        '---',
        f'*Generated by `docs/gen_cli_docs.py` from `{settings_c_path}`*',
        '',
    ]
    return '\n'.join(lines)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _git_hash(path):
    try:
        r = subprocess.run(
            ['git', 'rev-parse', '--short', 'HEAD'],
            cwd=path, capture_output=True, text=True, timeout=5
        )
        return r.stdout.strip() if r.returncode == 0 else None
    except Exception:
        return None


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        '--settings', default='src/main/interface/settings.c',
        help='Path to settings.c (default: src/main/interface/settings.c)'
    )
    parser.add_argument(
        '--header', default='src/main/interface/settings.h',
        help='Path to settings.h (default: src/main/interface/settings.h)'
    )
    parser.add_argument(
        '--output', default='docs/CLI/parameters-reference.md',
        help='Output path (default: docs/CLI/parameters-reference.md)'
    )
    args = parser.parse_args()

    c_path  = Path(args.settings)
    h_path  = Path(args.header)
    out_path = Path(args.output)

    for p in (c_path, h_path):
        if not p.exists():
            sys.exit(f"ERROR: {p} not found. Run from repo root.")

    c_text = strip_block_comments(c_path.read_text())
    h_text = strip_block_comments(h_path.read_text())

    # Also parse extern lookup arrays from sibling source files
    extra_sources = [
        c_path.parent.parent / 'sensors' / 'current.c',
        c_path.parent.parent / 'sensors' / 'voltage.c',
        c_path.parent.parent / 'build'   / 'debug.c',
    ]

    print(f"Parsing lookup table definitions...")
    lookup_defs = parse_lookup_defs(c_text)
    for extra in extra_sources:
        if extra.exists():
            lookup_defs.update(parse_lookup_defs(strip_block_comments(extra.read_text())))
        else:
            print(f"  WARNING: {extra} not found — some lookup values may be unresolved")
    print(f"  {len(lookup_defs)} arrays found")

    print("Building TABLE_xxx -> values map...")
    table_map = build_table_map(h_text, c_text, lookup_defs)
    print(f"  {len(table_map)} TABLE_xxx entries mapped")

    print("Parsing valueTable[]...")
    entries = parse_value_table(c_text)
    print(f"  {len(entries)} parameters found")

    if not entries:
        sys.exit("ERROR: no entries parsed — check settings.c path and format")

    git_hash = _git_hash(c_path.parent)

    print(f"Generating {out_path}...")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    md = generate_markdown(entries, table_map, str(c_path), git_hash)
    out_path.write_text(md)

    n_sections = len({e['pg'] for e in entries})
    print(f"Done: {len(entries)} parameters across {n_sections} sections -> {out_path}")


if __name__ == '__main__':
    main()
