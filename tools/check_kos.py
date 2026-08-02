#!/usr/bin/env python3
"""Static checks for kOS (.ks) scripts, run before the game gets a chance.

kOS compiles a script only when you RUN it, in the middle of a flight, and it
stops at the *first* error.  A typo therefore costs a reload and a rollout, and
you find the next one the same way.  These checks catch the mistakes that are
mechanical enough to spot from the text alone:

  1. Built-in shadowing.  kOS refuses to compile a script declaring a variable
     that would hide one of its built-in functions - the CLOBBERBUILTINS check.
     It is easy to think this only concerns R(), V() and Q(), but QUEUE, STACK,
     LIST, RANGE, NODE, PATH and a long tail of ordinary words are built-ins
     too, and `LOCAL queue IS LIST().` will not compile.
  2. Unbalanced braces and parentheses.
  3. Calls to functions that are never defined, and calls with the wrong
     number of arguments.

Usage:  python3 tools/check_kos.py scripts/*.ks
Exit status is 1 if anything was reported, so it drops into a git hook or CI.

The built-in list below is a best-effort snapshot of kOS 1.x.  A name missing
from it means a missed error, never a false one, so err on the side of adding.
"""

import re
import sys

# Built-in FUNCTIONS.  Declaring any of these as a variable is a compile error.
BUILTIN_FUNCTIONS = """
abs addons anglebetween angleaxis arccos arcsin arctan arctan2 body
bodyatmosphere buildlist career ceiling char clearguis clearvecdraws constant
copypath cos create createdir createorbit debugdump debugfreezegame deletepath
exists floor getvoice gui hasnode heading highlight hsv hsva hudtext latlng lex
lexicon list ln log10 lookdirup makebuiltindelegate max min mod movepath node
note open orbitat path pidloop positionat processor profileresult q queue r
random randomseed range readjson rgb rgba rotatefromto round scriptpath sin
slidenote sqrt stack stopallvoices tan timespan timestamp transfer transferall
unchar uniqueset v vang vcrs vdot vecdraw vecdrawargs vectorangle
vectorcrossproduct vectordotproduct vectorexclude velocityat vessel volume vxcl
warpto waypoint writejson
"""

# Bound variables.  Shadowing these is at best confusing and often an error too,
# but it is a softer call than the function list, so it is reported as a warning.
BOUND_VARIABLES = """
addons airspeed altitude angularvelocity apoapsis archive body config core
encounter eta facing geoposition groundspeed homeconnection kuniverse latitude
longitude mapview missiontime nextnode north obt periapsis prograde retrograde
sas sessionversion ship solarprimevector status steering target terminal
throttle time up velocity version warp warpmode
"""

FUNCS = set(BUILTIN_FUNCTIONS.split())
BOUND = set(BOUND_VARIABLES.split())

# Words that introduce a name.  `SET foo:bar TO x` assigns through a suffix and
# introduces nothing, so the pattern requires the name not be followed by ':'.
#
# The third field says whether the form is a *hard* declaration.  It matters for
# bound variables: `SET WARP TO 0.` is the documented way to drop out of time
# warp, an assignment to something that already exists, whereas `LOCAL warp IS
# ...` really would shadow it.  Built-in function names are rejected either way.
DECL_RE = [
    (re.compile(r"\bSET\s+([A-Za-z_]\w*)\s*(?![:\w])", re.I), "variable", False),
    (re.compile(r"\b(?:LOCAL|GLOBAL|DECLARE)\s+([A-Za-z_]\w*)\s*(?![:\w])", re.I), "variable", True),
    (re.compile(r"\bFOR\s+([A-Za-z_]\w*)\s+IN\b", re.I), "loop variable", True),
    (re.compile(r"\bFUNCTION\s+([A-Za-z_]\w*)", re.I), "function", True),
    (re.compile(r"\bLIST\s+\w+\s+IN\s+([A-Za-z_]\w*)", re.I), "list target", True),
]
PARAM_RE = re.compile(r"\bPARAMETER\s+([\w,\s]+?)\.", re.I)


def strip_noise(src):
    """Replace string literals and // comments, preserving line structure.

    Each string literal collapses to a single `0` rather than to whitespace:
    it still has to count as one argument when checking call arity, and
    `resAmt("LiquidFuel")` must not read as a zero-argument call.
    """
    out, in_str, i = [], False, 0
    while i < len(src):
        ch = src[i]
        if ch == '"':
            if not in_str:
                out.append("0")
            in_str = not in_str
            i += 1
            continue
        if in_str:
            out.append("\n" if ch == "\n" else "")
            i += 1
            continue
        if src[i:i + 2] == "//":
            while i < len(src) and src[i] != "\n":
                i += 1
            continue
        out.append(ch)
        i += 1
    return "".join(out)


def declarations(body):
    """Yield (line_number, name, kind, is_hard_declaration)."""
    for lineno, line in enumerate(body.split("\n"), 1):
        for regex, kind, hard in DECL_RE:
            for m in regex.finditer(line):
                yield lineno, m.group(1), kind, hard
        for m in PARAM_RE.finditer(line):
            for param in m.group(1).split(","):
                # Optional parameters carry a default: `PARAMETER ap IS 100000.`
                # Only the name before IS is the declaration.
                param = re.split(r"\s+IS\s+", param.strip(), maxsplit=1, flags=re.I)[0]
                param = param.strip()
                if param:
                    yield lineno, param, "parameter", True


def check_balance(body, problems):
    for name, (opener, closer) in {"brace": ("{", "}"), "paren": ("(", ")")}.items():
        depth = 0
        for lineno, line in enumerate(body.split("\n"), 1):
            for ch in line:
                if ch == opener:
                    depth += 1
                elif ch == closer:
                    depth -= 1
                    if depth < 0:
                        problems.append((lineno, f"unbalanced {name}: '{closer}' with nothing open"))
                        depth = 0
        if depth:
            problems.append((0, f"unbalanced {name}: {depth} left open at end of file"))


def check_functions(body, problems):
    defined = {}
    for m in re.finditer(r"FUNCTION\s+(\w+)\s*\{(.*?)(?=\bFUNCTION\b|\Z)", body, re.I | re.S):
        params = PARAM_RE.search(m.group(2))
        arity = len([p for p in params.group(1).split(",") if p.strip()]) if params else 0
        defined[m.group(1).lower()] = arity

    for lineno, line in enumerate(body.split("\n"), 1):
        if re.match(r"\s*FUNCTION\b", line, re.I):
            continue
        for m in re.finditer(r"\b([A-Za-z_]\w*)\s*\(([^()]*)\)", line):
            name, args = m.group(1).lower(), m.group(2).strip()
            if name in FUNCS or name in BOUND:
                continue
            if name not in defined:
                # Unknown names are usually built-ins missing from the list
                # above, so only flag ones that look like local helpers.
                if re.match(r"^[a-z][a-zA-Z0-9]*$", m.group(1)) and any(
                        c.isupper() for c in m.group(1)):
                    problems.append((lineno, f"call to undefined function '{m.group(1)}'"))
                continue
            got = 0 if not args else len(args.split(","))
            if got != defined[name]:
                problems.append((lineno,
                                 f"'{m.group(1)}' called with {got} argument(s), defined with {defined[name]}"))


def check_file(path):
    with open(path) as handle:
        body = strip_noise(handle.read())

    problems = []
    for lineno, name, kind, hard in declarations(body):
        low = name.lower()
        if low in FUNCS:
            problems.append((lineno,
                             f"{kind} '{name}' shadows the built-in function {name.upper()}() "
                             f"- kOS will refuse to compile this"))
        elif low in BOUND and hard:
            problems.append((lineno, f"{kind} '{name}' shadows the bound variable {name.upper()}"))
    check_balance(body, problems)
    check_functions(body, problems)

    if problems:
        print(f"{path}:")
        for lineno, message in sorted(problems):
            where = f"line {lineno}" if lineno else "end of file"
            print(f"  {where}: {message}")
    else:
        print(f"{path}: ok")
    return len(problems)


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    total = sum(check_file(path) for path in argv)
    if total:
        print(f"\n{total} problem(s) found.")
    return 1 if total else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
