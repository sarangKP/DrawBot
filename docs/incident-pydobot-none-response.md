# Incident: `AttributeError: 'NoneType' object has no attribute 'params'`

## What went wrong

On a second laptop, running `main_cp.py` crashed mid-draw:

```
File ".../robo_draw/venv/lib/python3.10/site-packages/pydobot/dobot.py", line 95
    expected_idx = struct.unpack_from('L', response.params, 0)[0]
AttributeError: 'NoneType' object has no attribute 'params'
```

## Why it happened

- This repo's `pydobot/` is a **patched local copy** (checksum fix `%255` → `%256`, plus a `response is None` guard in `_send_command`).
- The Dobot_v2 repo was copied to the second laptop, but its `robo_draw` venv installed `pydobot` from **PyPI** instead of the local patched package.
- The PyPI version lacks the `None` guard. When the arm didn't reply within the 1s read window (`_read_message()` returns `None`), the unpatched code crashed dereferencing `.params` instead of raising a clear error.
- Root trigger for the missing reply itself is likely the same class of issue the checksum fix addressed — arm silently drops malformed/unacknowledged commands.

## How we fixed it

1. Added `-e ./pydobot` to `Dobot_v2/requirements.txt` so `pip install -r requirements.txt` always installs the local patched package instead of pulling from PyPI.
2. On the second laptop: `pip uninstall pydobot -y` then reinstall via `pip install -r requirements.txt` from the repo root (or `pip install -e ../Dobot_v2/pydobot` if `robo_draw` is a separate repo).
3. Verify the right package is loaded:
   ```bash
   python -c "import pydobot, inspect; print(inspect.getfile(pydobot))"
   ```
   Path must point into `Dobot_v2/pydobot/...`, not `site-packages/pydobot`.

## Takeaway

Any machine that clones this repo must install `pydobot` via the local path, not pip. `requirements.txt` / `pyproject.toml` now enforce this by default.

## Follow-up incident (2026-07-06): editable install self-shadowed by the outer `pydobot/` folder

The `-e ./pydobot` fix above turned out to be broken on its own terms.

### What went wrong

Running `main_cp_v2.py -v` after an editable reinstall:

```
AttributeError: module 'pydobot' has no attribute 'Dobot'
```

### Why it happened

- This repo's outer project directory is itself named `pydobot/` (containing `setup.py`, the real package at `pydobot/pydobot/`, and a stale `build/lib/pydobot/`).
- Scripts are run from the repo root (`/home/user/Dobot_v2`), which puts `''` (cwd) first on `sys.path`.
- Python's default path-based finder sees the bare `pydobot/` directory sitting in cwd, has no `__init__.py` at that level, and treats it as an implicit **namespace package** for `pydobot` — before pip's editable meta-path finder is consulted.
- An editable install (`pip install -e`/`uv pip install -e`) only registers a meta-path finder + `.pth` file; it does **not** place a physical `pydobot/` directory in `site-packages`. With nothing in `site-packages` to make PathFinder resolve a *regular* package first, the namespace shadow wins, `__init__.py` (which does `from .dobot import Dobot`) never runs, and `pydobot.Dobot` doesn't exist.
- A plain (non-editable) install doesn't have this problem: it copies a real `pydobot/` package (with `__init__.py`) into `site-packages`, and a later path entry containing a *regular* package overrides an earlier namespace portion.
- This also explains why an earlier hardware-debugging session showed **zero `GET_ALARMS_STATE` polls** in a verbose log during a 60s command-queue stall — at the time, `import pydobot` was silently resolving to an old pip-installed 1.3.2 build in `~/.local/lib/...` (predating the alarm-poll code), not the patched local copy at all, despite `requirements.txt` supposedly pinning it.

### How we fixed it

1. `requirements.txt`: changed `-e ./pydobot` → `./pydobot` (plain install).
2. `pyproject.toml`: replaced the machine-specific absolute path (`pydobot @ file:///home/user/Dobot_v2/pydobot` — would not even resolve on another laptop) with a portable relative source:
   ```toml
   dependencies = [
       ...
       "pydobot",
   ]

   [tool.uv.sources]
   pydobot = { path = "pydobot", editable = false }
   ```
3. Verified the fix resolves the actual class, not just the module:
   ```bash
   python -c "import pydobot, inspect; print(pydobot.Dobot); print(inspect.getfile(pydobot.Dobot))"
   ```
   Must print a class (not raise `AttributeError`) and a file path under this repo's `pydobot/pydobot/dobot.py`.

### Setting this up on a new laptop

```bash
git clone <repo> Dobot_v2
cd Dobot_v2
uv sync                 # or: python -m venv .venv && ./venv/bin/pip install -r requirements.txt
.venv/bin/python -c "import pydobot, inspect; print(pydobot.Dobot); print(inspect.getfile(pydobot.Dobot))"
```
Always run install commands from the repo root — a relative `./pydobot` path resolves against the current working directory, not the location of `requirements.txt`/`pyproject.toml`.

### Takeaway

Never install this repo's `pydobot` in editable mode while the outer project folder is itself named `pydobot/` — cwd-based namespace shadowing silently wins over the editable meta-path finder. Use a plain install (or rename the outer folder) instead. After any future edit to `pydobot/pydobot/*.py`, re-run `uv sync` (or `pip install --force-reinstall ./pydobot`) to pick up the change, since it's no longer a live link.
