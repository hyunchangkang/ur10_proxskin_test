#!/usr/bin/env python3
"""Local browser controller for the existing interactive experiment launch."""

import argparse
from collections import deque
import codecs
import csv
import fcntl
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import importlib.util
import json
import math
import os
from pathlib import Path
import pty
import re
import secrets
import select
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import unquote, urlsplit
import webbrowser


ANSI = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
PROMPTS = re.compile(
    r'(?P<menu>Choose mode:)'
    r'|(?P<choose>Use current position as start\? \(y/n, q to cancel\):)'
    r'|(?P<distance>Start distance \(mm\):)'
    r'|(?P<adjust>Z adjustment:)'
    r'|(?P<repeat>Round trips(?: per X position)? \(1-100\):)'
    r'|(?P<dwell>Hold time at each stop \(s\)(?: \[[^\]]*\])?:)'
    r'|(?P<running>Press q to cancel this experiment and return to the (?:menu|saved start) \(no Enter needed\)\.)'
    r'|(?P<returning>Returning to saved experiment start after cancellation\.)')


def number(value, label, minimum=0, integer=False, maximum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label}: 숫자를 입력하세요.')
    if not math.isfinite(value) or value <= minimum:
        raise ValueError(f'{label}: {minimum}보다 큰 값을 입력하세요.')
    if integer and int(value) != value:
        raise ValueError(f'{label}: 정수를 입력하세요.')
    if maximum is not None and value > maximum:
        raise ValueError(f'{label}: 최대 {maximum}입니다.')
    return int(value) if integer else float(value)


def terminal_child():
    fcntl.ioctl(0, 0x540E, 0)  # TIOCSCTTY: let the existing nodes open /dev/tty.


def open_browser(url):
    try:
        if webbrowser.open(url, new=2):
            return
    except Exception as exc:
        print(f'Could not open the browser: {exc}', flush=True)
    print(f'Open this address in your browser: {url}', flush=True)


def experiment_files(directory):
    files = []
    directory = Path(directory).resolve()
    for path in directory.glob('experiment_*.csv'):
        try:
            if (re.fullmatch(r'experiment_[A-Za-z0-9_.-]+\.csv', path.name)
                    and path.resolve().parent == directory and path.is_file()):
                files.append((path, path.stat()))
        except OSError:
            continue
    return sorted(files, key=lambda item: (item[1].st_mtime_ns, item[0].name), reverse=True)


class CsvPlotReader:
    """Read every complete, valid CSV row at its original time resolution."""
    def __init__(self, directory):
        self.directory = Path(directory)
        self.lock = threading.Lock()
        self.cached_key = None
        self.cached = None

    @staticmethod
    def rows(path, size):
        # Use the size at request time, ignoring a partially written last row.
        with path.open('rb') as file:
            header = file.readline().decode('utf-8-sig').strip().split(',')
            if header[:3] != ['time_s', 'robot_distance_mm', 'sensor_value']:
                raise ValueError('CSV의 시간·로봇 거리·센서값 열을 확인하세요.')
            while file.tell() < size:
                line = file.readline()
                if file.tell() > size or not line.endswith(b'\n'):
                    break
                try:
                    row = next(csv.reader([line.decode('utf-8')]))
                    stamp, distance = float(row[0]), float(row[1])
                    if not math.isfinite(stamp) or not math.isfinite(distance):
                        continue
                    sensor = float(row[2]) if row[2].strip() else None
                    if sensor is not None and not math.isfinite(sensor):
                        sensor = None
                    yield [stamp, distance, sensor]
                except (ValueError, IndexError, UnicodeError, csv.Error):
                    continue

    def latest(self):
        return self.read()

    def read(self, name=None):
        with self.lock:
            files = experiment_files(self.directory)
            if name is not None:
                files = [(path, stat) for path, stat in files if path.name == name]
                if not files:
                    raise FileNotFoundError('선택한 CSV 파일을 찾을 수 없습니다.')
            if not files:
                return {'name': None, 'bytes': 0, 'rows': 0, 'sensor_rows': 0, 'points': []}
            path, stat = files[0]
            key = (path.name, stat.st_ino, stat.st_size, stat.st_mtime_ns)
            if key == self.cached_key:
                return self.cached
            points = list(self.rows(path, stat.st_size))
            sensor_count = sum(point[2] is not None for point in points)
            result = {'name': path.name, 'bytes': stat.st_size, 'mtime_ns': stat.st_mtime_ns,
                      'rows': len(points), 'sensor_rows': sensor_count, 'points': points}
            self.cached_key, self.cached = key, result
            return result


class ExperimentController:
    def __init__(self, start_file, feedback, start_loader, launch_args=(), command=None, driver_command=None):
        self.start_file = str(start_file)
        self.feedback = feedback
        self.start_loader = start_loader
        self.launch_args = list(launch_args)
        self.command = command or ['ros2', 'launch', 'ur10_proxskin_test', 'proxskin_soc_test.launch.py']
        self.driver_command = driver_command or ['ros2', 'run', 'urcb2_driver', 'single_arm']
        self.lock = threading.RLock()
        self.changed = threading.Condition(self.lock)
        self.phase = 'menu'
        self.mode = None
        self.process = None
        self.master = None
        self.reader = None
        self.options = {}
        self.logs = deque(maxlen=160)
        self.sequence = 0
        self.last_error = ''
        self.closed = False
        self.driver_process = None
        self.driver_reader = None

    def _log(self, message):
        self.sequence += 1
        self.logs.append({'id': self.sequence, 'text': message})

    def _write(self, text):
        if self.master is None or self.process is None or self.process.poll() is not None:
            raise ValueError('실험 프로세스가 종료되었습니다. 모드를 다시 선택하세요.')
        os.write(self.master, text.encode())

    def _prompt(self, kind):
        if self.phase == 'stopping':
            return
        if kind == 'menu':
            self.phase = 'menu'
            self.mode = None
            self.options = {}
        elif kind in ('choose', 'distance', 'adjust'):
            self.phase = {'choose': 'choose_start', 'distance': 'distance', 'adjust': 'adjust'}[kind]
        elif kind == 'repeat':
            self._write(str(self.options['repeats']) + '\n')
        elif kind == 'dwell':
            self._write(str(self.options['dwell']) + '\n')
        elif kind == 'running':
            self.phase = 'running'
        elif kind == 'returning':
            self.phase = 'returning'
        self.changed.notify_all()

    def _read(self, process, master):
        decoder = codecs.getincrementaldecoder('utf-8')(errors='replace')
        pending, line = '', ''
        try:
            while True:
                ready, _, _ = select.select([master], [], [], .1)
                if ready:
                    try:
                        chunk = os.read(master, 8192)
                    except OSError:
                        break
                    if not chunk:
                        break
                    text = decoder.decode(chunk).replace('\r', '')
                    with self.changed:
                        pending += text
                        line += text
                        while '\n' in line:
                            item, line = line.split('\n', 1)
                            item = ANSI.sub('', item).strip()
                            if item:
                                self._log(item)
                                if '[ERROR]' in item:
                                    self.last_error = item
                        while True:
                            match = PROMPTS.search(ANSI.sub('', pending))
                            if not match:
                                pending = pending[-16384:]
                                break
                            clean = ANSI.sub('', pending)
                            pending = clean[match.end():]
                            self._prompt(match.lastgroup)
                elif process.poll() is not None:
                    break
        except Exception as exc:
            with self.lock:
                self.last_error = f'실험 통신 오류: {exc}'
                self._log(self.last_error)
            self._signal_owned(process, signal.SIGINT)
        finally:
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._signal_owned(process, signal.SIGKILL)
                process.wait(timeout=3)
            os.close(master)
            with self.changed:
                if self.process is process:
                    if line.strip():
                        self._log(ANSI.sub('', line).strip())
                    self._log('실험 프로세스 종료. 모드를 다시 선택할 수 있습니다.')
                    self.process = None
                    self.master = None
                    self.mode = None
                    self.phase = 'menu'
                    self.changed.notify_all()

    @staticmethod
    def _signal_owned(process, sig):
        if process.poll() is None:
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                pass

    def _ensure_menu(self):
        if self.process is not None:
            return
        master, slave = pty.openpty()
        try:
            command = self.command + ['start_file:=' + self.start_file] + self.launch_args
            child = [sys.executable, str(Path(__file__).resolve()), '--terminal-child'] + command
            process = subprocess.Popen(child, stdin=slave, stdout=slave, stderr=slave,
                                       start_new_session=True, close_fds=True)
        except Exception:
            os.close(master)
            raise
        finally:
            os.close(slave)
        self.process, self.master, self.phase = process, master, 'connecting'
        self.reader = threading.Thread(target=self._read, args=(process, master), daemon=True)
        self.reader.start()
        deadline = time.monotonic() + 12
        while self.phase == 'connecting' and self.process is not None:
            left = deadline - time.monotonic()
            if left <= 0:
                self.phase = 'stopping'
                self._signal_owned(process, signal.SIGINT)
                raise ValueError('실험 메뉴를 열지 못했습니다. 아래 로그를 확인하세요.')
            self.changed.wait(min(left, .2))
        if self.process is None or self.phase != 'menu':
            raise ValueError('실험 메뉴 실행에 실패했습니다. ROS 환경과 로그를 확인하세요.')

    def action(self, data):
        with self.changed:
            if self.closed:
                raise ValueError('서버가 종료 중입니다.')
            action = data.get('action')
            if action == 'driver_start':
                feedback = self.feedback()
                if self.driver_process is not None or feedback['connected'] or feedback.get('joint_publishers', 0):
                    raise ValueError('드라이버가 이미 실행 중입니다.')
                process = subprocess.Popen(self.driver_command,
                                           stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                           stderr=subprocess.STDOUT, text=True, errors='replace',
                                           start_new_session=True)
                self.driver_process = process
                self.driver_reader = threading.Thread(target=self._read_driver, args=(process,), daemon=True)
                self.driver_reader.start()
                self._log('드라이버 실행: ros2 run urcb2_driver single_arm')
            elif action == 'driver_stop':
                if self.phase != 'menu':
                    raise ValueError('실험을 먼저 중단하고 드라이버를 종료하세요.')
                if self.driver_process is None:
                    raise ValueError('외부 터미널에서 실행한 드라이버는 해당 터미널에서 종료하세요.')
                process = self.driver_process
                self._signal_owned(process, signal.SIGINT)
                threading.Thread(target=self._finish_driver, args=(process,), daemon=True).start()
            elif action == 'connect':
                if self.phase != 'menu':
                    raise ValueError('진행 중인 작업을 먼저 마치거나 중단하세요.')
                if self.feedback()['publishers']:
                    raise ValueError('다른 제어 노드가 실행 중입니다. 기존 실험 launch를 종료하세요.')
                self._ensure_menu()
            elif action == 'mode':
                mode = data.get('mode')
                if type(mode) is not int or mode not in range(5):
                    raise ValueError('0~4번 모드를 선택하세요.')
                if self.phase != 'menu':
                    raise ValueError('실행 중인 작업을 먼저 마치거나 중단하세요.')
                feedback = self.feedback()
                if not feedback['connected']:
                    raise ValueError('로봇 관절 피드백이 없습니다. 같은 ROS_DOMAIN_ID로 드라이버를 켜세요.')
                if feedback['publishers']:
                    raise ValueError('다른 제어 노드가 실행 중입니다. 기존 실험 launch를 종료하세요.')
                options = {}
                if mode >= 2:
                    self.start_loader(self.start_file)
                    options['repeats'] = number(data.get('repeats', 1), '왕복 횟수', integer=True, maximum=100)
                    options['dwell'] = number(data.get('dwell', 2), '정지 시간') if mode == 4 else 0
                self._ensure_menu()
                if self.closed or not self.feedback()['connected']:
                    raise ValueError('서버가 종료 중이거나 로봇 피드백이 끊겼습니다.')
                self.options = options
                self.mode = mode
                self.phase = 'running' if mode == 0 else 'starting'
                self.last_error = ''
                self._write(str(mode) + '\n')
            elif action == 'choose':
                self._require('choose_start')
                if data.get('value') not in ('y', 'n'):
                    raise ValueError('현재 위치 사용 또는 위치 조정을 선택하세요.')
                self.phase = 'starting'
                self._write(data['value'] + '\n')
            elif action == 'save':
                self._require('distance')
                distance = number(data.get('distance'), 'stroke')
                self.phase = 'aligning'
                self.last_error = ''
                self._write(str(distance) + '\n')
            elif action in ('jog', 'align', 'back'):
                self._require('adjust')
                self.last_error = ''
                if action == 'jog':
                    delta = data.get('delta')
                    if isinstance(delta, bool) or not isinstance(delta, (int, float)) or not math.isfinite(delta):
                        raise ValueError('Z 이동량은 방향을 포함한 정수 mm입니다.')
                    if delta == 0 or abs(delta) > 300 or int(delta) != delta:
                        raise ValueError('Z 이동량은 ±1~300 정수 mm입니다.')
                    self.phase = 'jogging'
                    self._write(f'{int(delta):+d}\n')
                elif action == 'align':
                    self.phase = 'aligning'
                    self._write('a\n')
                else:
                    self.phase = 'starting'
                    self._write('e\n')
            elif action == 'cancel':
                if self.process is None or self.phase == 'menu':
                    return
                previous = self.phase
                self.phase = 'stopping'
                self._log('사용자가 중단을 요청했습니다.')
                if self.mode in (2, 3, 4) and previous in ('running', 'returning'):
                    self._write('q')
                    # Menu prompt must still be accepted after graceful q cancellation.
                    self.phase = 'cancelling'
                elif self.mode == 1 and previous in ('choose_start', 'distance', 'adjust'):
                    self._write('q\n')
                    self.phase = 'cancelling'
                else:
                    self._signal_owned(self.process, signal.SIGINT)
                threading.Thread(target=self._finish_cancel, args=(self.process,), daemon=True).start()
            else:
                raise ValueError('지원하지 않는 동작입니다.')

    def _read_driver(self, process):
        try:
            for line in process.stdout:
                with self.lock:
                    self._log('[Driver] ' + ANSI.sub('', line).strip())
        finally:
            process.wait()
            process.stdout.close()
            with self.lock:
                if self.driver_process is process:
                    self.driver_process = None
                    self._log(f'웹에서 실행한 드라이버 종료 (code {process.returncode}).')

    def _finish_driver(self, process):
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self._signal_owned(process, signal.SIGTERM)
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._signal_owned(process, signal.SIGKILL)

    def _finish_cancel(self, process):
        time.sleep(3)
        with self.lock:
            if self.process is not process or self.phase not in ('stopping', 'cancelling'):
                return
            self._signal_owned(process, signal.SIGTERM)
        time.sleep(2)
        with self.lock:
            if self.process is process and self.phase in ('stopping', 'cancelling'):
                self._signal_owned(process, signal.SIGKILL)

    def _require(self, phase):
        if self.phase != phase:
            raise ValueError('현재 단계에서는 이 버튼을 사용할 수 없습니다.')

    def snapshot(self):
        with self.lock:
            try:
                saved = self.start_loader(self.start_file)
                start = {'distance': saved['start_distance_mm'], 'pose': saved['tcp_pose']}
            except (OSError, ValueError):
                start = None
            return {'phase': self.phase, 'mode': self.mode, 'start': start,
                    'feedback': self.feedback(), 'logs': list(self.logs), 'error': self.last_error,
                    'experiment_connected': self.process is not None,
                    'driver_owned': self.driver_process is not None}

    def close(self):
        with self.lock:
            self.closed = True
            process, reader = self.process, self.reader
            if process:
                self._signal_owned(process, signal.SIGINT)
        if reader:
            reader.join(timeout=3)
        if process and process.poll() is None:
            self._signal_owned(process, signal.SIGKILL)
        if reader and reader.is_alive():
            reader.join(timeout=4)
        with self.lock:
            driver, driver_reader = self.driver_process, self.driver_reader
        if driver:
            self._signal_owned(driver, signal.SIGINT)
            self._finish_driver(driver)
        if driver_reader:
            driver_reader.join(timeout=2)


class WebServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, port, controller, share, data_dir):
        super().__init__(('127.0.0.1', port), Handler)
        self.controller, self.share, self.data_dir = controller, Path(share), Path(data_dir).resolve()
        self.plot_reader = CsvPlotReader(self.data_dir)
        self.token = secrets.token_urlsafe(32)
        self.hosts = {f'127.0.0.1:{self.server_port}', f'localhost:{self.server_port}'}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def _json(self, value, status=200):
        body = json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(body)

    def _local(self):
        return self.headers.get('Host') in self.server.hosts

    def do_GET(self):
        if not self._local():
            return self._json({'error': 'Local access only.'}, 403)
        path = urlsplit(self.path).path
        if path == '/api/state':
            state = self.server.controller.snapshot()
            state['token'] = self.server.token
            state['domain'] = os.environ.get('ROS_DOMAIN_ID', '0')
            files = experiment_files(self.server.data_dir)
            state['files'] = [{'name': file.name, 'bytes': stat.st_size, 'mtime_ns': stat.st_mtime_ns}
                              for file, stat in files]
            return self._json(state)
        if path == '/api/plot/latest' or path.startswith('/api/plot/file/'):
            try:
                name = unquote(path[len('/api/plot/file/'):]) if path.startswith('/api/plot/file/') else None
                return self._json(self.server.plot_reader.read(name))
            except FileNotFoundError as exc:
                return self._json({'error': str(exc)}, 404)
            except (OSError, ValueError) as exc:
                return self._json({'error': f'CSV 그래프를 불러오지 못했습니다: {exc}'}, 409)
        static = {'/': 'control.html', '/control.html': 'control.html',
                  '/guide': 'how_to_use.html', '/how_to_use.html': 'how_to_use.html',
                  '/how_to_use.txt': 'how_to_use.txt'}
        if path in static:
            file = self.server.share / static[path]
            content_type = 'text/plain; charset=utf-8' if file.suffix == '.txt' else 'text/html; charset=utf-8'
        elif path.startswith('/data/'):
            name = unquote(path[len('/data/'):])
            file = (self.server.data_dir / name).resolve()
            if not re.fullmatch(r'experiment_[A-Za-z0-9_.-]+\.csv', name) or file.parent != self.server.data_dir:
                return self._json({'error': '파일을 찾을 수 없습니다.'}, 404)
            content_type = 'text/csv; charset=utf-8'
        else:
            return self._json({'error': '파일을 찾을 수 없습니다.'}, 404)
        try:
            body = file.read_bytes()
        except OSError:
            return self._json({'error': '파일을 찾을 수 없습니다.'}, 404)
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        if file.suffix == '.csv':
            self.send_header('Content-Disposition', f'attachment; filename="{file.name}"')
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        origin = self.headers.get('Origin', '')
        if (not self._local() or origin not in {'http://' + host for host in self.server.hosts}
                or not secrets.compare_digest(self.headers.get('X-Control-Token', ''), self.server.token)):
            return self._json({'error': '제어 페이지를 이 서버 주소에서 다시 여세요.'}, 403)
        if self.path != '/api/action':
            return self._json({'error': '지원하지 않는 주소입니다.'}, 404)
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if length <= 0 or length > 4096:
                raise ValueError('잘못된 요청입니다.')
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict):
                raise ValueError('잘못된 요청입니다.')
            self.server.controller.action(data)
            self._json({'ok': True})
        except (ValueError, OSError, KeyError) as exc:
            self._json({'error': str(exc)}, 409)


def main():
    from ament_index_python.packages import get_package_share_directory
    import rclpy
    from sensor_msgs.msg import JointState

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--no-browser', action='store_true', help='Start the server without opening a browser')
    parser.add_argument('--start-file', default='/home/ur/.ros/ur10_proxskin_test/experiment_start.json')
    parser.add_argument('--config-file', default='')
    args = parser.parse_args()
    share = Path(get_package_share_directory('ur10_proxskin_test'))
    spec = importlib.util.spec_from_file_location('experiment_menu', share / 'launch/proxskin_soc_test.launch.py')
    menu = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(menu)
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    lock_file = open(f'/tmp/ur10_proxskin_web_{os.getuid()}_{domain}.lock', 'w')
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('이 ROS_DOMAIN_ID의 웹 제어 서버가 이미 실행 중입니다.')
    rclpy.init(args=[])
    node = rclpy.create_node('proxskin_web_status')
    latest = {'stamp': 0.0}
    def receive(message):
        if len(message.position) >= 6 and all(math.isfinite(value) for value in message.position[:6]):
            latest['stamp'] = time.monotonic()
    node.create_subscription(JointState, '/UR10_right/joint_states', receive, 10)
    def feedback():
        return {'connected': time.monotonic() - latest['stamp'] < .5,
                'publishers': node.count_publishers('/UR10_right/targetJ'),
                'joint_publishers': node.count_publishers('/UR10_right/joint_states')}
    launch_args = ['config_file:=' + args.config_file] if args.config_file else []
    controller = ExperimentController(args.start_file, feedback, menu._load_experiment_start, launch_args)
    server = WebServer(args.port, controller, share, '/home/ur/ros2_ws/src/ur10_proxskin_test/data')
    stopping = threading.Event()
    def spin():
        while not stopping.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
    monitor = threading.Thread(target=spin, daemon=True)
    monitor.start()
    def stop_server(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGINT, stop_server)
    signal.signal(signal.SIGTERM, stop_server)
    url = f'http://127.0.0.1:{server.server_port}'
    print(f'UR10 web control: {url}  (ROS_DOMAIN_ID={domain})', flush=True)
    print('Starting the server does not move the robot. Select a mode in the browser.', flush=True)
    try:
        if not args.no_browser:
            threading.Thread(target=open_browser, args=(url,), daemon=True).start()
        server.serve_forever(poll_interval=.1)
    finally:
        controller.close()
        server.server_close()
        stopping.set()
        monitor.join(timeout=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock_file.close()


if __name__ == '__main__':
    if len(sys.argv) > 2 and sys.argv[1] == '--terminal-child':
        terminal_child()
        os.execvp(sys.argv[2], sys.argv[2:])
    else:
        main()
