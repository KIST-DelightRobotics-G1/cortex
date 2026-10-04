"""Exercise the real runtime script against a recording Docker CLI."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]


def run_script(tmp_path, gpus='all', existing=False, old_gpus=None, old_image='image-new'):
    models = tmp_path/'model directory'
    models.mkdir(exist_ok=True)
    log = tmp_path/'calls.jsonl'
    fake = tmp_path/'docker'
    fake.write_text('#!' + sys.executable + '\n' + """
import json, os, sys
a = sys.argv[1:]
with open(os.environ['DOCKER_TEST_LOG'], 'a') as f:
    f.write(json.dumps(a) + '\\n')
if a[0] == 'ps':
    if os.environ['DOCKER_TEST_EXISTS'] == 'yes':
        print('container-id')
elif a[0] == 'inspect':
    fmt = a[2]
    if 'runtime.gpus' in fmt:
        print(os.environ['DOCKER_TEST_OLD_GPUS'])
    elif 'runtime.models' in fmt:
        print(os.environ['CORTEX_MODEL_DIR'])
    else:
        print(os.environ['DOCKER_TEST_OLD_IMAGE'])
elif a[:2] == ['image', 'inspect']:
    print('image-new')
""")
    fake.chmod(0o755)
    env = {**os.environ, 'PATH': str(tmp_path)+os.pathsep+os.environ['PATH'],
           'CORTEX_MODEL_DIR': str(models), 'CORTEX_GPUS': gpus,
           'TTS_CACHE_DIR': str(tmp_path/'tts'), 'CORTEX_ENV_FILE': str(tmp_path/'no.env'),
           'DOCKER_TEST_LOG': str(log), 'DOCKER_TEST_EXISTS': 'yes' if existing else 'no',
           'DOCKER_TEST_OLD_GPUS': old_gpus if old_gpus is not None else gpus,
           'DOCKER_TEST_OLD_IMAGE': old_image}
    result = subprocess.run(['bash', str(ROOT/'docker/run.sh'), 'echo', 'argument with spaces'],
                            env=env, capture_output=True, text=True)
    calls = [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []
    return result, calls, models


@pytest.mark.parametrize('gpus,expected', [('all', 'all'), ('0', 'device=0'), ('none', None)])
def test_new_container_gpu_and_readonly_model_mount(tmp_path, gpus, expected):
    result, calls, models = run_script(tmp_path, gpus)
    assert result.returncode == 0, result.stderr
    args = calls[-1]
    assert args[0] == 'run'
    if expected:
        assert args[args.index('--gpus')+1] == expected
    else:
        assert '--gpus' not in args
    assert args[args.index('--mount')+1] == f'type=bind,src={models},dst=/models/cortex,readonly'
    assert args[-2:] == ['echo', 'argument with spaces']


def test_matching_container_is_reused_without_recreation(tmp_path):
    result, calls, _ = run_script(tmp_path, existing=True)
    assert result.returncode == 0
    assert calls[-1][0] == 'exec'
    assert not any(c[0] in ('run', 'rm') for c in calls)


@pytest.mark.parametrize('old_gpus,old_image', [('', 'image-new'), ('none', 'image-new'),
                                               ('all', 'image-old')])
def test_incompatible_existing_container_is_left_untouched(tmp_path, old_gpus, old_image):
    result, calls, _ = run_script(tmp_path, existing=True, old_gpus=old_gpus, old_image=old_image)
    assert result.returncode != 0 and 'NOT been removed' in result.stderr
    assert not any(c[0] in ('run', 'rm', 'start', 'exec') for c in calls)


def test_invalid_gpu_option_fails_before_docker(tmp_path):
    result, calls, _ = run_script(tmp_path, 'invalid')
    assert result.returncode != 0 and not calls
