"""
Mini preprocesseur C (sous-ensemble) pour verifier les descriptions de carte
sans compilateur : #define (objet et fonction), #undef, #if/#ifdef/#ifndef/
#elif/#else/#endif, defined(), #include "...", #error, #warning.

Limites assumees : pas de # / ##, pas de macros variadiques, arithmetique
entiere evaluee en Python. Suffisant pour hw_types.h / hw_check.h / cartes.
"""
import os
import re

IDENT = re.compile(r"[A-Za-z_]\w*")


def _strip(text):
    text = re.sub(r"/\*.*?\*/", lambda m: " " * 0 + "\n" * m.group(0).count("\n"), text, flags=re.S)
    text = re.sub(r"//[^\n]*", "", text)
    return re.sub(r"\\\s*\n", " ", text)


class MiniPP:
    def __init__(self, include_dirs=(), predefined=None):
        self.include_dirs = list(include_dirs)
        self.macros = {}          # nom -> (params | None, corps)
        self.errors = []          # (fichier, ligne, message)
        self.warnings = []
        for k, v in (predefined or {}).items():
            self.macros[k] = (None, str(v))

    # ------------------------------------------------------------ expansion
    def _split_args(self, s, i):
        """s[i] == '(' ; renvoie (liste d'arguments, index apres ')')."""
        depth, args, cur = 0, [], ""
        while i < len(s):
            c = s[i]
            if c == "(":
                depth += 1
                if depth > 1:
                    cur += c
            elif c == ")":
                depth -= 1
                if depth == 0:
                    args.append(cur.strip())
                    return args, i + 1
                cur += c
            elif c == "," and depth == 1:
                args.append(cur.strip()); cur = ""
            else:
                cur += c
            i += 1
        raise ValueError("parenthese non fermee")

    def expand(self, s, hide=frozenset(), depth=0):
        if depth > 40:
            return s
        out, i = "", 0
        while i < len(s):
            m = IDENT.match(s, i)
            if not m:
                out += s[i]; i += 1; continue
            name = m.group(0); j = m.end()
            if name in self.macros and name not in hide:
                params, body = self.macros[name]
                if params is None:
                    out += "(" + self.expand(body, hide | {name}, depth + 1) + ")" if body.strip() else ""
                    i = j; continue
                k = j
                while k < len(s) and s[k].isspace():
                    k += 1
                if k < len(s) and s[k] == "(":
                    args, end = self._split_args(s, k)
                    args = [self.expand(a, hide, depth + 1) for a in args]
                    rep = body
                    for p, a in zip(params, args):
                        rep = re.sub(r"\b%s\b" % re.escape(p), lambda _m, a=a: "(" + a + ")", rep)
                    out += "(" + self.expand(rep, hide | {name}, depth + 1) + ")"
                    i = end; continue
            out += name; i = j
        return out

    def eval_cond(self, expr):
        e = re.sub(r"defined\s*\(\s*(\w+)\s*\)", lambda m: "1" if m.group(1) in self.macros else "0", expr)
        e = re.sub(r"defined\s+(\w+)", lambda m: "1" if m.group(1) in self.macros else "0", e)
        e = self.expand(e)
        e = re.sub(r"\b(0[xX][0-9a-fA-F]+|\d+)[uUlL]+\b", r"\1", e)
        e = re.sub(r"\b0[xX][0-9a-fA-F]+\b", lambda m: str(int(m.group(0), 16)), e)
        e = IDENT.sub(lambda m: m.group(0) if m.group(0) in ("and", "or", "not") else "0", e)
        e = e.replace("&&", " and ").replace("||", " or ")
        e = re.sub(r"!(?!=)", " not ", e)
        e = e.replace("/", "//")
        return bool(eval(e, {"__builtins__": {}}, {}))

    def value(self, name):
        """Valeur numerique d'une macro objet, ou None."""
        if name not in self.macros:
            return None
        try:
            v = self.expand(name)
            v = re.sub(r"\b(\d+\.\d*|\.\d+|\d+)[fF]\b", r"\1", v)
            v = re.sub(r"\b(0[xX][0-9a-fA-F]+|\d+)[uUlL]+\b", r"\1", v)
            v = re.sub(r"\(\s*float\s*\)", "", v)
            return eval(v, {"__builtins__": {}}, {})
        except Exception:
            return None

    # ------------------------------------------------------------- fichiers
    def _find(self, name, cur_dir):
        for d in [cur_dir] + self.include_dirs:
            p = os.path.join(d, name)
            if os.path.exists(p):
                return p
        return None

    def run_file(self, path):
        with open(path, encoding="utf-8", errors="replace") as fh:
            lines = _strip(fh.read()).split("\n")
        stack = []
        cur_dir = os.path.dirname(path)
        for ln, raw in enumerate(lines, 1):
            line = raw.strip()
            m = re.match(r"#\s*(\w+)\s*(.*)", line)
            if not m:
                continue
            d, rest = m.group(1), m.group(2).strip()
            active = all(s[0] for s in stack)
            if d in ("if", "ifdef", "ifndef"):
                if not active:
                    stack.append((False, True)); continue
                if d == "ifdef": c = rest.split()[0] in self.macros
                elif d == "ifndef": c = rest.split()[0] not in self.macros
                else: c = self.eval_cond(rest)
                stack.append((c, c)); continue
            if d == "elif":
                _, taken = stack.pop()
                parent = all(s[0] for s in stack)
                c = parent and not taken and self.eval_cond(rest)
                stack.append((c, taken or c)); continue
            if d == "else":
                _, taken = stack.pop()
                stack.append((all(s[0] for s in stack) and not taken, True)); continue
            if d == "endif":
                stack.pop(); continue
            if not active:
                continue
            if d == "define":
                mm = re.match(r"(\w+)(\(([^)]*)\))?\s*(.*)", rest)
                params = [p.strip() for p in mm.group(3).split(",")] if mm.group(2) else None
                self.macros[mm.group(1)] = (params, mm.group(4).strip())
            elif d == "undef":
                self.macros.pop(rest.split()[0], None)
            elif d == "include":
                inc = re.match(r'"([^"]+)"', rest)
                if not inc:
                    tok = rest.split()[0]
                    if tok in self.macros:
                        inc = re.match(r'"([^"]+)"', self.macros[tok][1])
                if inc:
                    p = self._find(inc.group(1), cur_dir)
                    if p is None:
                        self.errors.append((path, ln, f"include introuvable : {inc.group(1)}"))
                    else:
                        self.run_file(p)
            elif d == "error":
                self.errors.append((path, ln, rest))
            elif d == "warning":
                self.warnings.append((path, ln, rest))
        if stack:
            self.errors.append((path, len(lines), "#if non ferme"))
