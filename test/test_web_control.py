#!/usr/bin/env python3
"""Exercise the web bridge with fake terminal programs; never start ROS or a robot."""

import http.client
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('web_control', PACKAGE / 'scripts/web_control.py')
web = importlib.util.module_from_spec(spec)
spec.loader.exec_module(web)

FAKE = r'''
import json, os, signal, subprocess, sys, termios, time
from pathlib import Path
record = Path(sys.argv[1])
def log(text):
    with record.open('a') as file: file.write(text + '\n')
def ask(prompt):
    print(prompt, end='', flush=True)
    with open('/dev/tty') as tty: return tty.readline().strip()
if '--driver' in sys.argv:
    print('FAKE DRIVER READY', flush=True)
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    while True: time.sleep(.1)
if '--setup' in sys.argv:
    while True:
        answer = ask('Use current position as start? (y/n, q to cancel):\n')
        log('choose:' + answer)
        if answer == 'q': break
        if answer == 'y':
            distance = ask('Start distance (mm):\n')
            if distance == 'q': break
            path = next(arg.split(':=', 1)[1] for arg in sys.argv if arg.startswith('start_file:='))
            Path(path).write_text(json.dumps({'start_distance_mm': float(distance), 'tcp_pose': [0]*6}))
            log('save:' + distance)
            break
        while True:
            adjustment = ask('Z adjustment:\n')
            log('adjust:' + adjustment)
            if adjustment in ('e', 'q'): break
            if adjustment == 'a' and '--slow-align' in sys.argv: time.sleep(30)
        if adjustment == 'q': break
    sys.exit(0)
while True:
    mode = ask('Choose mode: ')
    log('mode:' + mode)
    if mode == '1':
        # Match launch: child stdin need not be the controlling terminal.
        subprocess.run([sys.executable, __file__] + sys.argv[1:] + ['--setup'], stdin=subprocess.DEVNULL)
    elif mode in ('2', '3', '4'):
        label = 'Round trips per X position (1-100):' if mode == '3' else 'Round trips (1-100):'
        log('repeats:' + ask(label))
        if mode == '4': log('dwell:' + ask('Hold time at each stop (s) [2]:'))
        fd = os.open('/dev/tty', os.O_RDONLY)
        previous = termios.tcgetattr(fd)
        raw = termios.tcgetattr(fd)
        raw[3] &= ~(termios.ICANON | termios.ECHO)
        termios.tcsetattr(fd, termios.TCSANOW, raw)
        print('Press q to cancel this experiment and return to the saved start (no Enter needed).', flush=True)
        log('cancel:' + os.read(fd, 1).decode())
        print('Returning to saved experiment start after cancellation.', flush=True)
        if '--hold-return' in sys.argv:
            log('return-cancel:' + os.read(fd, 1).decode())
        else:
            time.sleep(4.1 if '--slow-return' in sys.argv else .1)
            log('returned')
        termios.tcsetattr(fd, termios.TCSANOW, previous)
        os.close(fd)
'''


class CsvPlotTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.addCleanup(self.temp.cleanup)

    def test_latest_missing_sensor_zero_and_incomplete_row(self):
        old = self.root / 'experiment_old.csv'
        old.write_text('time_s,robot_distance_mm,sensor_value\n0,100,99\n')
        latest = self.root / 'experiment_new.csv'
        latest.write_text('time_s,robot_distance_mm,sensor_value\n0,300,\n0.01,299.5,0\n0.02,299,8590941\n0.03,298.5,8')
        os.utime(old, ns=(1, 1))
        reader = web.CsvPlotReader(self.root)
        result = reader.latest()
        self.assertEqual(result['name'], latest.name)
        self.assertEqual(result['points'], [[0, 300, None], [.01, 299.5, 0], [.02, 299, 8590941]])
        self.assertEqual(result['sensor_rows'], 2)
        self.assertIs(result, reader.latest())
        with latest.open('a') as file:
            file.write('590942\n')
        updated = reader.latest()
        self.assertEqual(updated['rows'], 4)
        self.assertEqual(updated['points'][-1][2], 8590942)

    def test_long_record_keeps_every_original_row(self):
        file = self.root / 'experiment_long.csv'
        expected = []
        with file.open('w') as output:
            output.write('time_s,robot_distance_mm,sensor_value\n')
            for i in range(20000):
                robot = -17 if i == 3456 else (345 if i == 6789 else 100)
                sensor = '' if i == 10001 else (40000001 if i == 12345 else 8590000)
                output.write(f'{i / 100},{robot},{sensor}\n')
                expected.append([i / 100, robot, None if sensor == '' else sensor])
        result = web.CsvPlotReader(self.root).latest()
        self.assertEqual(result['rows'], 20000)
        self.assertEqual(result['points'], expected)
        self.assertEqual(result['points'][0][0], 0)
        self.assertEqual(result['points'][-1][0], 199.99)
        self.assertEqual(min(point[1] for point in result['points']), -17)
        self.assertEqual(max(point[1] for point in result['points']), 345)
        self.assertIn([123.45, 100, 40000001], result['points'])
        self.assertIn([100.01, 100, None], result['points'])

    def test_selected_old_csv_and_restricted_paths(self):
        old = self.root / 'experiment_old.csv'
        old.write_text('time_s,robot_distance_mm,sensor_value\n0,100,99\n')
        for i in range(8):
            (self.root / f'experiment_new_{i}.csv').write_text(old.read_text())
        os.utime(old, ns=(1, 1))
        reader = web.CsvPlotReader(self.root)
        self.assertNotEqual(reader.latest()['name'], old.name)
        self.assertEqual(reader.read(old.name)['points'], [[0, 100, 99]])
        with old.open('a') as output:
            output.write('0.01,99,98\n')
        self.assertEqual(reader.read(old.name)['rows'], 2)
        for name in ('../experiment_old.csv', 'missing.csv', '/etc/passwd'):
            with self.assertRaises(FileNotFoundError):
                reader.read(name)

    def test_no_files_invalid_csv_and_outside_symlink(self):
        reader = web.CsvPlotReader(self.root)
        self.assertIsNone(reader.latest()['name'])
        outside = self.root / 'outside'
        outside.mkdir()
        target = outside / 'other.csv'
        target.write_text('private data')
        (self.root / 'experiment_link.csv').symlink_to(target)
        self.assertIsNone(reader.latest()['name'])
        file = self.root / 'experiment_invalid.csv'
        file.write_text('a,b,c\n1,2,3\n')
        with self.assertRaises(ValueError):
            reader.latest()


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        self.script, self.record, self.start = root / 'fake.py', root / 'record.txt', root / 'start.json'
        self.script.write_text(FAKE)
        self.status = {'connected': True, 'publishers': 0, 'joint_publishers': 1}
        command = [sys.executable, str(self.script), str(self.record)]
        self.controller = web.ExperimentController(
            self.start, lambda: dict(self.status), lambda path: json.loads(Path(path).read_text()),
            command=command, driver_command=command + ['--driver'])

    def tearDown(self):
        self.controller.close()
        self.temp.cleanup()

    def wait(self, predicate):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            state = self.controller.snapshot()
            if predicate(state): return state
            time.sleep(.02)
        self.fail('Timed out: ' + repr(self.controller.snapshot()))

    def phase(self, phase):
        return self.wait(lambda state: state['phase'] == phase)

    def test_setup_and_nested_terminal_child(self):
        self.controller.action({'action': 'connect'})
        self.assertTrue(self.controller.snapshot()['experiment_connected'])
        self.controller.action({'action': 'mode', 'mode': 1})
        self.phase('choose_start')
        self.controller.action({'action': 'choose', 'value': 'n'})
        self.phase('adjust')
        with self.assertRaises(ValueError): self.controller.action({'action': 'jog', 'delta': .5})
        self.controller.action({'action': 'jog', 'delta': -10})
        self.phase('adjust')
        self.controller.action({'action': 'align'})
        self.phase('adjust')
        self.controller.action({'action': 'back'})
        self.phase('choose_start')
        self.controller.action({'action': 'choose', 'value': 'y'})
        self.phase('distance')
        with self.assertRaises(ValueError): self.controller.action({'action': 'save', 'distance': 0})
        self.controller.action({'action': 'save', 'distance': 125.5})
        state = self.phase('menu')
        self.assertEqual(state['start']['distance'], 125.5)
        self.assertIn('adjust:-10\nadjust:a\nadjust:e', self.record.read_text())

    def test_all_experiments_and_raw_q_return_to_menu(self):
        self.start.write_text(json.dumps({'start_distance_mm': 200, 'tcp_pose': [0]*6}))
        for mode in (2, 3, 4):
            with self.subTest(mode=mode):
                self.controller.action({'action': 'mode', 'mode': mode, 'repeats': 3, 'dwell': .25})
                self.phase('running')
                with self.assertRaises(ValueError): self.controller.action({'action': 'mode', 'mode': 0})
                self.controller.action({'action': 'cancel'})
                state = self.phase('menu')
                self.assertTrue(state['experiment_connected'])
        record = self.record.read_text()
        self.assertEqual(record.count('repeats:3'), 3)
        self.assertEqual(record.count('cancel:q'), 3)
        self.assertIn('dwell:0.25', record)

    def test_cancel_alignment_terminates_owned_launch(self):
        self.controller.command.append('--slow-align')
        self.controller.action({'action': 'mode', 'mode': 1})
        self.phase('choose_start')
        self.controller.action({'action': 'choose', 'value': 'n'})
        self.phase('adjust')
        self.controller.action({'action': 'align'})
        self.controller.action({'action': 'cancel'})
        self.wait(lambda state: state['phase'] == 'menu' and not state['experiment_connected'])

    def test_return_longer_than_cancel_watchdog_is_not_killed(self):
        self.start.write_text(json.dumps({'start_distance_mm': 200, 'tcp_pose': [0]*6}))
        self.controller.command.append('--slow-return')
        self.controller.action({'action': 'mode', 'mode': 3})
        self.phase('running')
        self.controller.action({'action': 'cancel'})
        self.phase('returning')
        state = self.phase('menu')
        self.assertTrue(state['experiment_connected'])
        self.assertIn('returned', self.record.read_text())

    def test_second_cancel_stops_return(self):
        self.start.write_text(json.dumps({'start_distance_mm': 200, 'tcp_pose': [0]*6}))
        self.controller.command.append('--hold-return')
        self.controller.action({'action': 'mode', 'mode': 2})
        self.phase('running')
        self.controller.action({'action': 'cancel'})
        self.phase('returning')
        self.controller.action({'action': 'cancel'})
        self.phase('menu')
        self.assertIn('return-cancel:q', self.record.read_text())

    def test_driver_buttons_and_external_driver_guard(self):
        with self.assertRaises(ValueError): self.controller.action({'action': 'driver_start'})
        self.status.update(connected=False, joint_publishers=0)
        self.controller.action({'action': 'driver_start'})
        self.wait(lambda state: any('FAKE DRIVER READY' in item['text'] for item in state['logs']))
        with self.assertRaises(ValueError): self.controller.action({'action': 'driver_start'})
        self.controller.action({'action': 'driver_stop'})
        self.wait(lambda state: not state['driver_owned'])
        with self.assertRaises(ValueError): self.controller.action({'action': 'driver_stop'})

    def test_motion_requires_feedback_and_exclusive_publisher(self):
        self.status['connected'] = False
        with self.assertRaises(ValueError): self.controller.action({'action': 'mode', 'mode': 0})
        self.status.update(connected=True, publishers=1)
        with self.assertRaises(ValueError): self.controller.action({'action': 'mode', 'mode': 0})
        with self.assertRaises(ValueError): self.controller.action({'action': 'connect'})
        self.assertIsNone(self.controller.process)

    def test_http_origin_token_routes_and_download(self):
        server = web.WebServer(0, self.controller, PACKAGE, self.temp.name)
        thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .05})
        thread.start()
        connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=3)
        try:
            connection.request('GET', '/')
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            self.assertIn('driver-start', response.read().decode())
            connection.request('GET', '/api/state')
            response = connection.getresponse()
            token = json.loads(response.read())['token']
            headers = {'Content-Type': 'application/json', 'Origin': 'https://example.com', 'X-Control-Token': token}
            connection.request('POST', '/api/action', json.dumps({'action': 'connect'}), headers)
            response = connection.getresponse()
            self.assertEqual(response.status, 403)
            response.read()
            self.assertIsNone(self.controller.process)
            headers['Origin'] = f'http://127.0.0.1:{server.server_port}'
            headers['X-Control-Token'] = 'wrong'
            connection.request('POST', '/api/action', json.dumps({'action': 'connect'}), headers)
            response = connection.getresponse()
            self.assertEqual(response.status, 403)
            response.read()
            headers['X-Control-Token'] = token
            connection.request('POST', '/api/action', json.dumps({'action': 'connect'}), headers)
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            response.read()
            csv = Path(self.temp.name) / 'experiment_mock.csv'
            csv.write_text('time_s,robot_distance_mm,sensor_value\n0.0,125.50,\n')
            connection.request('GET', '/data/' + csv.name)
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            self.assertEqual(response.read().decode(), csv.read_text())
            connection.request('GET', '/api/plot/latest')
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            plot = json.loads(response.read())
            self.assertEqual(plot['name'], csv.name)
            self.assertEqual(plot['points'], [[0, 125.5, None]])
            old = csv.parent / 'experiment_old.csv'
            old.write_text('time_s,robot_distance_mm,sensor_value\n0,200,321\n')
            os.utime(old, ns=(1, 1))
            for i in range(8):
                (csv.parent / f'experiment_new_{i}.csv').write_text(csv.read_text())
            connection.request('GET', '/api/state')
            response = connection.getresponse()
            self.assertIn(old.name, [file['name'] for file in json.loads(response.read())['files']])
            connection.request('GET', '/api/plot/file/' + old.name)
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            self.assertEqual(json.loads(response.read())['points'], [[0, 200, 321]])
            for name in ('..%2Fstart.json', '%2Fetc%2Fpasswd', 'experiment_missing.csv'):
                connection.request('GET', '/api/plot/file/' + name)
                response = connection.getresponse()
                self.assertEqual(response.status, 404)
                response.read()
            connection.request('GET', '/data/../start.json')
            response = connection.getresponse()
            self.assertEqual(response.status, 404)
            response.read()
        finally:
            connection.close()
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


if __name__ == '__main__':
    unittest.main()
