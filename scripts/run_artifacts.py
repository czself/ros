"""Persist complete run summaries without exposing a partially written JSON file."""
import json
import os
import stat
import tempfile


def write_run_summary(directory, report):
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, 'run_summary.json')
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                                         dir=directory, prefix='.run_summary.',
                                         suffix='.tmp', delete=False) as stream:
            temporary = stream.name
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        try:
            mode = stat.S_IMODE(os.stat(path).st_mode)
        except FileNotFoundError:
            mode = 0o644
        os.chmod(temporary, mode)
        os.replace(temporary, path)
        temporary = None
        return path
    finally:
        if temporary is not None:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass
