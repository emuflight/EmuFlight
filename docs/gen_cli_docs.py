#!/usr/bin/env python3
"""
gen_cli_docs.py — Generate docs/CLI/parameters-reference.md

Parses valueTable[], lookupTables[], and section comments from settings.c/h,
plus cmdTable[] in cli.c, to produce a single CLI reference: one section per
settings PG group, and a CLI Commands (PG-Backed) section for commands whose
config persists through their own dedicated table/handler instead of
valueTable[].

No external dependencies (Python 3.6+ stdlib only).

Usage (from repo root):
    python3 docs/gen_cli_docs.py
    python3 docs/gen_cli_docs.py --settings src/main/interface/settings.c
    python3 docs/gen_cli_docs.py --cli src/main/interface/cli.c
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
    'OSD', 'PID', 'PWM', 'RC', 'RCDEVICE', 'RPM', 'RTC', 'RX', 'SDCARD', 'SDIO',
    'SPI', 'TX', 'USB', 'VCD', 'VTX',
}

# Commands whose settings persist through their own dedicated table + handler
# resolving into PG storage (pgFind()), not through settings.c's valueTable[] --
# invisible to the settings-based parser above. Names must match cmdTable[]
# entries in cli.c exactly.
PG_BACKED_COMMANDS = [
    'adjrange', 'aux', 'color', 'dma', 'led', 'mmix',
    'mode_color', 'rxfail', 'rxrange', 'serial', 'smix', 'vtx',
]


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
# cmdTable[] parsing (CLI_COMMAND_DEF entries)
# ---------------------------------------------------------------------------

def _split_top_level_args(s):
    """Split a CLI_COMMAND_DEF(...) argument list on commas outside quotes."""
    fields = []
    depth = 0
    in_str = False
    cur = ''
    i = 0
    while i < len(s):
        c = s[i]
        if in_str:
            cur += c
            if c == '\\' and i + 1 < len(s):
                i += 1
                cur += s[i]
            elif c == '"':
                in_str = False
        elif c == '"':
            in_str = True
            cur += c
        elif c == '(':
            depth += 1
            cur += c
        elif c == ')':
            depth -= 1
            cur += c
        elif c == ',' and depth == 0:
            fields.append(cur.strip())
            cur = ''
        else:
            cur += c
        i += 1
    if cur.strip():
        fields.append(cur.strip())
    return fields


def _extract_c_string(field):
    """Join adjacent string literals in a field; None for a bare NULL."""
    if field.strip() == 'NULL':
        return None
    parts = re.findall(r'"((?:[^"\\]|\\.)*)"', field)
    joined = ''.join(parts)
    return joined.replace('\\r\\n', '\n').replace('\\t', '\t').replace('\\"', '"')


def parse_cmd_table(c_text):
    """Returns list of entry dicts (name, description, args, ifdef_conds) from cmdTable[]."""
    start = c_text.find('const clicmd_t cmdTable[] = {')
    if start == -1:
        sys.exit("ERROR: cannot find 'const clicmd_t cmdTable[]' in cli.c")

    end = c_text.find('\n};', start)
    block = c_text[start: end if end != -1 else len(c_text)]

    entries = []
    ifdef_stack = []
    lines = block.splitlines()
    idx = 0
    while idx < len(lines):
        line = lines[idx].strip()

        m = re.match(r'#ifdef\s+(\w+)', line)
        if m:
            ifdef_stack.append(m.group(1))
            idx += 1
            continue
        m = re.match(r'#ifndef\s+(\w+)', line)
        if m:
            ifdef_stack.append('!' + m.group(1))
            idx += 1
            continue
        if re.match(r'#if\s+', line):
            ifdef_stack.append(line[3:].strip())
            idx += 1
            continue
        if line.startswith('#endif'):
            if ifdef_stack:
                ifdef_stack.pop()
            idx += 1
            continue
        if line.startswith('#else'):
            if ifdef_stack:
                top = ifdef_stack[-1]
                ifdef_stack[-1] = top[1:] if top.startswith('!') else ('!' + top)
            idx += 1
            continue
        if re.match(r'#elif\s', line):
            if ifdef_stack:
                ifdef_stack.pop()
            ifdef_stack.append(line[5:].strip())
            idx += 1
            continue

        if line.startswith('CLI_COMMAND_DEF('):
            text = line
            depth = text.count('(') - text.count(')')
            while depth > 0 and idx + 1 < len(lines):
                idx += 1
                text += ' ' + lines[idx].strip()
                depth += lines[idx].count('(') - lines[idx].count(')')

            inner = text[text.index('(') + 1: text.rindex(')')]
            fields = _split_top_level_args(inner)
            if len(fields) == 4:
                entries.append({
                    'name':        fields[0].strip().strip('"'),
                    'description': _extract_c_string(fields[1]),
                    'args':        _extract_c_string(fields[2]),
                    'ifdef_conds': list(ifdef_stack),
                })
        idx += 1

    return entries


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


def _extract_use_conditions(cond):
    """Extract all USE_* symbols from a condition, preserving negation."""
    out = []
    # Match negated or plain USE_* tokens, including defined() forms
    for m in re.finditer(r'(!)?\s*(?:defined\()?\s*(USE_[A-Z0-9_]+)\s*\)?', cond):
        neg, sym = m.groups()
        out.append(f'!{sym}' if neg else sym)
    return out


def _format_requires(ifdef_conds):
    """Return USE_xxx requirements, including negated/defined forms."""
    simple = []
    for c in ifdef_conds:
        simple.extend(_extract_use_conditions(c))
        # Also catch standalone simple forms
        if re.match(r'!?USE_\w+$', c):
            simple.append(c)
    if not simple:
        return ''
    return ', '.join(f'`{c}`' for c in dict.fromkeys(simple))  # deduplicated, ordered


def generate_markdown(entries, table_map, settings_c_path, cmd_entries, cli_c_path,
                      git_hash=None, fw_version=None, msp_version=None):
    today = date.today().isoformat()
    ref = git_hash or 'unknown'
    fw_str  = f' | Firmware: `{fw_version}`'  if fw_version  else ''
    msp_str = f' | MSP: `{msp_version}`'      if msp_version else ''

    # Group by PG, preserving first-seen order
    sections = OrderedDict()
    for e in entries:
        sections.setdefault(e['pg'], []).append(e)

    lines = [
        '# CLI Reference',
        '',
        '> **Auto-generated** — do not edit manually.',
        f'> Source: `{settings_c_path}`, `{cli_c_path}` | Generated: {today} | Commit: `{ref}`{fw_str}{msp_str}',
        '',
        'Settings parameters (`valueTable[]`) and CLI commands that persist config through',
        'their own dedicated table instead (`cmdTable[]`, PG-backed) are both covered here.',
        '',
        '---',
        '',
        '## Table of Contents',
        '',
    ]

    for pg, pg_entries in sections.items():
        human = pg_to_human(pg)
        lines.append(f'- [{human}](#{_anchor(human)})')
    lines.append('- [CLI Commands (PG-Backed)](#cli-commands-pg-backed)')

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

    lines.append('## CLI Commands (PG-Backed)')
    lines.append('')
    lines.append('Commands whose settings persist through their own dedicated table and')
    lines.append('handler instead of `valueTable[]`.')
    lines.append('')

    by_name = {e['name']: e for e in cmd_entries}
    missing = [n for n in PG_BACKED_COMMANDS if n not in by_name]
    if missing:
        print(f"  WARNING: PG-backed commands not found in cmdTable[]: {', '.join(missing)}")
    rows = [by_name[n] for n in PG_BACKED_COMMANDS if n in by_name]

    lines.append('| Command | Description | Args | Requires |')
    lines.append('|---------|-------------|------|----------|')
    for e in rows:
        desc = (e['description'] or '').replace('|', '\\|')
        args = _format_cmd_args(e['args'])
        req  = _format_requires(e['ifdef_conds'])
        lines.append(f'| `{e["name"]}` | {desc} | {args} | {req} |')
    lines.append('')

    lines += [
        '---',
        f'*Generated by `docs/gen_cli_docs.py` from `{settings_c_path}`, `{cli_c_path}`*',
        '',
    ]
    return '\n'.join(lines)


def _format_cmd_args(args):
    if not args:
        return ''
    flat = ' '.join(line.strip() for line in args.splitlines() if line.strip())
    escaped = flat.replace('|', '\\|')
    return f'`{escaped}`'


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


def _firmware_version(repo_root: Path):
    """Return 'MAJOR.MINOR.PATCH' from src/main/build/version.h, or None."""
    vh = repo_root / 'src' / 'main' / 'build' / 'version.h'
    if not vh.exists():
        return None
    text = vh.read_text()
    def _def(name):
        m = re.search(rf'#define\s+{name}\s+(\d+)', text)
        return m.group(1) if m else None
    major = _def('FC_VERSION_MAJOR')
    minor = _def('FC_VERSION_MINOR')
    patch = _def('FC_VERSION_PATCH_LEVEL')
    if major and minor and patch:
        return f'{major}.{minor}.{patch}'
    return None


def _msp_version(repo_root: Path):
    """Return 'PROTO.MAJOR.MINOR' MSP version from msp_protocol.h, or None."""
    mh = repo_root / 'src' / 'main' / 'interface' / 'msp_protocol.h'
    if not mh.exists():
        return None
    text = mh.read_text()
    def _def(name):
        m = re.search(rf'#define\s+{name}\s+(\d+)', text)
        return m.group(1) if m else None
    proto = _def('MSP_PROTOCOL_VERSION')
    major = _def('API_VERSION_MAJOR')
    minor = _def('API_VERSION_MINOR')
    if major and minor:
        proto = proto or '0'
        return f'{proto}.{major}.{minor}'
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
    parser.add_argument(
        '--cli', default='src/main/interface/cli.c',
        help='Path to cli.c (default: src/main/interface/cli.c)'
    )
    args = parser.parse_args()

    c_path  = Path(args.settings)
    h_path  = Path(args.header)
    out_path = Path(args.output)
    cli_path = Path(args.cli)

    for p in (c_path, h_path, cli_path):
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

    # Resolve repo root: walk up from settings.c until we find version.h
    repo_root = c_path.resolve().parent
    while repo_root != repo_root.parent:
        if (repo_root / 'src' / 'main' / 'build' / 'version.h').exists():
            break
        repo_root = repo_root.parent

    git_hash   = _git_hash(repo_root)
    fw_version = _firmware_version(repo_root)
    msp_version = _msp_version(repo_root)

    if fw_version:
        print(f"Firmware version: {fw_version}")
    if msp_version:
        print(f"MSP version:      {msp_version}")

    print("Parsing cmdTable[]...")
    cli_text = strip_block_comments(cli_path.read_text())
    cmd_entries = parse_cmd_table(cli_text)
    print(f"  {len(cmd_entries)} commands found")

    if not cmd_entries:
        sys.exit("ERROR: no entries parsed — check cli.c path and format")

    print(f"Generating {out_path}...")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    md = generate_markdown(entries, table_map, str(c_path), cmd_entries, str(cli_path),
                           git_hash, fw_version=fw_version, msp_version=msp_version)
    out_path.write_text(md)

    n_sections = len({e['pg'] for e in entries})
    n_pg_commands = len([n for n in PG_BACKED_COMMANDS if n in {e['name'] for e in cmd_entries}])
    print(f"Done: {len(entries)} parameters across {n_sections} sections, "
          f"{n_pg_commands} PG-backed commands -> {out_path}")


if __name__ == '__main__':
    main()
