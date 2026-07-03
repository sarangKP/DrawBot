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

Any machine that clones this repo must install `pydobot` via the local editable path, not pip. `requirements.txt` now enforces this by default.
