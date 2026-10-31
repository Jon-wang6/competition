#!/usr/bin/env python3
"""公开练习包控制器；裁判规则由随包二进制执行，不包含正式赛事配置。"""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

import requests
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from internal_ros import internal_context

ROOT = Path(__file__).resolve().parent


def write_json(path, value):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2))
    tmp.replace(path)


def integer(value, low, high, name):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f'{name} 须为 {low}–{high} 的整数')
    return value


def public_board(value):
    # Keep practice results without exporting full state or structured event effects.
    fields = ('team', 'team_name', 'score', 'resource_value', 'effective_damage',
              'mission_completed', 'invalid_actions', 'hp', 'alive')
    board = {**{key: value[key] for key in ('tick', 'action_tick', 'stage', 'session_id') if key in value},
            'teams': [{key: row[key] for key in fields if key in row}
                      for row in value.get('teams', []) if isinstance(row, dict)]}
    view = value.get('map_view', {})
    board['map_view'] = {key: view[key] for key in ('map', 'obstacles', 'resources') if key in view}
    board['map_view']['robots'] = [{key: row[key] for key in ('team', 'position', 'hp', 'alive') if key in row}
                                  for row in view.get('robots', [])]
    return board


class Practice:
    def __init__(self, args):
        self.args = args
        self.lock = threading.RLock()
        self.command_lock = threading.Lock()
        self.children = []
        self.session = ''
        self.starting = False
        self.stopped = True
        self.status, self.board, self.checks = {}, {}, {}
        self.fault = ''
        self.ai = {}
        self.settings = None
        self.ai_slots = {seat: threading.BoundedSemaphore(1) for seat in 'ABCD'}
        self.last_status = time.monotonic()
        self.data = Path(args.data_dir).resolve()
        self.data.mkdir(parents=True, exist_ok=True)
        self.private_context = internal_context()
        self.private = Node('practice_control', context=self.private_context)
        self.public = Node('practice_check')
        self.control = self.private.create_publisher(String, '/organizer/control', 10)
        self.private.create_subscription(String, '/organizer/status', self.on_status, 10)
        self.private.create_subscription(String, '/referee/scoreboard', self.on_board,
            QoSProfile(depth=2048, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for seat in 'ABCD':
            self.public.create_subscription(String, f'/team_{seat}/perception',
                lambda msg, seat=seat: self.on_public(seat, 'perception', msg), 1)
            self.public.create_subscription(String, f'/team_{seat}/action',
                lambda msg, seat=seat: self.on_public(seat, 'action', msg), 1)
        self.executors = []
        self.threads = []
        for node in (self.private, self.public):
            executor = MultiThreadedExecutor(num_threads=2, context=node.context)
            executor.add_node(node)
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            self.executors.append(executor)
            self.threads.append(thread)

    def on_status(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                return
        except (ValueError, TypeError):
            return
        with self.lock:
            if not self.stopped and value.get('session_id') == self.session:
                self.status = value
                self.last_status = time.monotonic()

    def on_board(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict) or not isinstance(value.get('actions', []), list):
                return
        except (ValueError, TypeError):
            return
        with self.lock:
            if self.stopped or value.get('session_id') != self.session:
                return
            self.board = public_board(value)
            try:
                with (self.run / 'frames.jsonl').open('a') as stream:
                    stream.write(json.dumps(self.board, ensure_ascii=False) + '\n')
                for row in value.get('actions', []):
                    if isinstance(row, dict) and isinstance(row.get('team'), str) and 'action_tick' in value and row.get('source') == 'submitted' and row.get('status') != 'rejected':
                        self.checks.setdefault(row['team'], {})['executed_tick'] = value['action_tick']
            except OSError:
                self.fault = '练习日志写入失败，已暂停；检查磁盘后重开'
                self.emit('pause')

    def on_public(self, seat, kind, msg):
        try:
            value = json.loads(msg.data)
            with self.lock:
                if not isinstance(value, dict):
                    raise ValueError('message must be object')
                if self.stopped or not self.session:
                    return
                if value.get('session_id') != self.session:
                    if kind == 'action':
                        self.checks.setdefault(seat, {})['session_error'] = 'session_mismatch'
                        with (self.run / 'communication.jsonl').open('a') as stream:
                            stream.write(json.dumps({'seat': seat, 'kind': kind, 'data': value,
                                'diagnostic': 'session_mismatch', 'expected_session_id': self.session}, ensure_ascii=False) + '\n')
                    return
                check = self.checks.setdefault(seat, {})
                if kind == 'action':
                    check.pop('session_error', None)
                check[kind + '_tick'] = value.get('tick')
                if kind == 'perception':
                    check['position'] = value.get('self', {}).get('position')
                    check['last_action'] = value.get('self', {}).get('last_action')
                    check['event_text'] = value.get('event_text', '')
                # Only referee settlement establishes acceptance; seeing a topic alone does not.
                with (self.run / 'communication.jsonl').open('a') as stream:
                    stream.write(json.dumps({'seat': seat, 'kind': kind, 'data': value}, ensure_ascii=False) + '\n')
        except (ValueError, TypeError, AttributeError):
            with self.lock:
                self.checks.setdefault(seat, {})['parse_error'] = '收到非 JSON 对象消息'
        except OSError:
            with self.lock:
                self.fault = '通信日志写入失败，已暂停'
                self.emit('pause')

    def emit(self, command):
        self.control.publish(String(data=json.dumps({'command': command,
            'session_id': self.session, 'request_id': uuid.uuid4().hex})))

    def snapshot(self):
        with self.lock:
            if self.children and not self.starting and not self.stopped:
                for proc, _, label in self.children:
                    if proc.poll() is not None:
                        self.fault = f'{label} 已退出（{proc.returncode}），请查看日志后重开'
                if time.monotonic() - self.last_status > 8:
                    self.fault = '裁判心跳超时，请查看日志后重开'
                if self.fault:
                    self.emit('pause')
            return copy.deepcopy(dict(session_id=self.session, starting=self.starting, stopped=self.stopped,
                status={key: self.status[key] for key in ('tick', 'max_ticks', 'ready', 'paused', 'finished', 'fault') if key in self.status}, board=self.board,
                checks=self.checks, ai=self.ai, fault=self.fault,
                settings=self.settings, ai_mode=self.args.ai_mode, run_dir=str(getattr(self, 'run', self.data)),
                gateway=f'http://127.0.0.1:{self.args.port}/v1/chat/completions',
                public_domain=int(os.environ['ROS_DOMAIN_ID'])))

    def stop(self):
        with self.lock:
            self.stopped = True
            self.starting = False
            self.status.update(stopped=True, ready=False, paused=True)
            children = list(self.children)
        # All processes share each shutdown deadline; N hung children do not cost N waits.
        for sig in (signal.SIGTERM, signal.SIGKILL):
            for proc, _, _ in children:
                if proc.poll() is None:
                    try:
                        os.killpg(proc.pid, sig)
                    except ProcessLookupError:
                        pass
            deadline = time.monotonic() + 5
            for proc, _, _ in children:
                try:
                    proc.wait(timeout=max(0, deadline - time.monotonic()))
                except subprocess.TimeoutExpired:
                    pass
            if all(proc.poll() is not None for proc, _, _ in children):
                break
        self.children = [(p, f, label) for p, f, label in children if p.poll() is None]
        for proc, stream, _ in children:
            if proc.poll() is not None:
                stream.close()
        if self.children:
            raise RuntimeError('子进程未能在停止期限内退出，请检查系统状态')

    def start(self, settings):
        stage = integer(settings.get('stage', 2), 1, 2, 'stage')
        tick_ms = integer(settings.get('tick_ms', 100 if stage == 1 else 500), 50, 5000, 'tick_ms')
        max_ticks = integer(settings.get('max_ticks', 100 if stage == 1 else 500), 1, 5000, 'max_ticks')
        bots = settings.get('bots', [])
        if not isinstance(bots, list) or len(bots) > 3 or any(s not in list('ABCD') for s in bots) or len(set(bots)) != len(bots):
            raise ValueError('陪练席位须是最多三个不重复的 A/B/C/D')
        config = copy.deepcopy(settings['config'] if 'config' in settings else json.loads((ROOT / f'config/stage{stage}.json').read_text()))
        if not isinstance(config, dict) or 'event_params_file' in config:
            raise ValueError('配置须为完整 JSON 对象，事件配置请直接写在 events 内')
        config['stage'] = stage
        if stage == 1:
            config['missions'] = []
            config['mission_text'] = '练习采集：自主收集资源。'
            config['events']['enabled'] = False
        seats = [r['team'] for r in config['robots']]
        if not seats or len(set(seats)) != len(seats) or any(s not in list('ABCD') for s in seats) or any(s not in seats for s in bots):
            raise ValueError('车辆席位须为不重复的 A/B/C/D，陪练须在场内')
        validation = subprocess.run([str(ROOT / 'bin/validate_config')], input=json.dumps(config),
            capture_output=True, text=True, timeout=5)
        if validation.returncode:
            raise ValueError('练习配置无效：' + validation.stderr.strip())
        session = 'practice-' + uuid.uuid4().hex
        run = self.data / session
        run.mkdir()
        write_json(run / 'config.json', config)
        # Validate and persist the replacement before stopping a playable match.
        with self.lock:
            self.stop()
            self.starting, self.stopped = True, False
            self.session, self.run = session, run
            self.status, self.board, self.checks, self.ai = {}, {}, {}, {}
            self.fault = ''
            self.settings = dict(stage=stage, tick_ms=tick_ms, max_ticks=max_ticks, bots=list(bots), config=config)
            initial = json.loads(validation.stdout)
            self.board = public_board(dict(tick=0, stage=stage,
                teams=[dict(team=r['team'], team_name=r['team'], score=0, hp=r['hp']) for r in initial['robots']],
                map_view={key: initial[key] for key in ('map', 'obstacles', 'resources', 'robots')}))
            self.last_status = time.monotonic()
        env = dict(os.environ, ROS_LOG_DIR=str(run / 'ros-logs'))
        def spawn(binary, params, label, extra_env=None):
            stream = (run / (label + '.log')).open('a')
            try:
                proc = subprocess.Popen([str(ROOT / 'bin' / binary), '--ros-args'] +
                    [arg for key, value in params.items() for arg in ('-p', f'{key}:={value}')],
                    env=dict(env, **(extra_env or {})), stdout=stream, stderr=subprocess.STDOUT,
                    start_new_session=True)
            except Exception:
                stream.close()
                raise
            self.children.append((proc, stream, label))
        try:
            spawn('simulator_node', dict(config=run / 'config.json', event_log='/dev/null', session_id=session), 'simulator')
            spawn('scoring_node', dict(score_file=run / 'score.json', stage=stage), 'scoring')
            spawn('referee_node', dict(teams='[' + ','.join(seats) + ']', tick_ms=tick_ms,
                max_ticks=max_ticks, session_id=session, start_paused='true'), 'referee')
            for seat in bots:
                spawn('robot_node', dict(team=seat, team_name='练习陪练' + seat,
                    ai_gateway=f'http://127.0.0.1:{self.args.port}/v1/chat/completions'), 'bot_' + seat,
                    dict(RMTC_TEAM_TOKEN='practice-' + seat))
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline:
                if any(p.poll() is not None for p, _, _ in self.children):
                    raise RuntimeError('练习节点启动失败，请查看运行日志')
                with self.lock:
                    if self.status.get('ready'):
                        self.starting = False
                        self.last_status = time.monotonic()
                        return {'ok': True, 'session_id': session}
                time.sleep(.1)
            raise RuntimeError('裁判启动超时，请检查 ROS 环境与日志')
        except Exception:
            with self.lock:
                self.fault = '部署失败，请检查日志后重开'
            try:
                self.stop()
            except Exception:
                with self.lock:
                    self.fault += '；进程清理失败，请检查残留进程'
            raise

    def close(self):
        errors = []
        cleanup = [self.stop]
        cleanup += [lambda executor=e: executor.shutdown(timeout_sec=5) for e in self.executors]
        cleanup += [lambda thread=t: thread.join(timeout=5) for t in self.threads]
        cleanup += [self.private.destroy_node, self.public.destroy_node, self.private_context.shutdown]
        for action in cleanup:
            try:
                action()
            except Exception as exc:
                errors.append(exc)
        if errors:
            raise errors[0]


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    slots = threading.BoundedSemaphore(24)

    def process_request(self, request, client_address):
        if not self.slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self.slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.slots.release()


class Handler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(35)

    def log_message(self, *args):
        pass

    @property
    def app(self):
        return self.server.app

    def send(self, value, status=200, mime='application/json'):
        data = value if isinstance(value, bytes) else json.dumps(value, ensure_ascii=False).encode()
        try:
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass

    def do_GET(self):
        try:
            path = urlsplit(self.path).path
            if path == '/':
                return self.send((ROOT / 'index.html').read_bytes(), mime='text/html; charset=utf-8')
            if path == '/api/state':
                return self.send(self.app.snapshot())
            if path in ('/api/config/1', '/api/config/2'):
                return self.send(json.loads((ROOT / ('config/stage' + path[-1] + '.json')).read_text()))
            self.send({'error': '资源不存在'}, 404)
        except OSError:
            self.send({'error': '读取失败，请检查运行日志'}, 500)

    def do_POST(self):
        try:
            if self.headers.get('Transfer-Encoding'):
                return self.send({'error': {'type': 'invalid_request_error', 'message': '不支持 Transfer-Encoding'}}, 501)
            length = int(self.headers.get('Content-Length', '0'))
            if length <= 0:
                raise ValueError('请求体不能为空，须提供 JSON 对象')
            if length > 2 * 1024 * 1024:
                return self.send({'error': {'type': 'invalid_request_error', 'message': '请求体大小无效'}}, 413)
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict):
                raise ValueError('请求体须为 JSON 对象')
            if self.path == '/v1/chat/completions':
                return self.gateway(body)
            origin = self.headers.get('Origin')
            if origin and urlsplit(origin).netloc != self.headers.get('Host'):
                return self.send({'error': '拒绝跨域控制'}, 403)
            with self.app.command_lock:
                if self.path == '/api/start':
                    return self.send(self.app.start(body))
                if self.path == '/api/reset':
                    if self.app.settings is None:
                        raise ValueError('尚未部署')
                    return self.send(self.app.start(self.app.settings))
                if self.path in ('/api/pause', '/api/resume'):
                    if self.app.stopped:
                        raise ValueError('练习已停止，请重新部署')
                    command = self.path.rsplit('/', 1)[-1]
                    if command == 'resume' and (self.app.fault or not self.app.status.get('ready')):
                        raise ValueError('裁判未就绪或已故障，请重新部署')
                    self.app.emit(command)
                    return self.send({'ok': True})
                if self.path == '/api/stop':
                    self.app.stop()
                    return self.send({'ok': True})
            self.send({'error': '资源不存在'}, 404)
        except (ValueError, KeyError, TypeError) as exc:
            self.send({'error': str(exc)}, 400)
        except (OSError, RuntimeError, subprocess.SubprocessError):
            self.send({'error': '系统操作失败，请查看运行目录日志'}, 500)

    def gateway(self, body):
        auth = self.headers.get('Authorization', '')
        seat = auth.removeprefix('Bearer practice-') if auth.startswith('Bearer practice-') else ''
        if seat not in list('ABCD'):
            return self.send({'error': {'message': '使用本地测试令牌 practice-A/B/C/D',
                'type': 'authentication_error', 'code': 'invalid_token'}}, 401)
        if not isinstance(body.get('messages'), list) or not body['messages']:
            return self.send({'error': {'message': 'messages 须为非空数组', 'type': 'invalid_request_error'}}, 400)
        if body.get('stream'):
            return self.send({'error': {'message': '不支持流式请求', 'type': 'invalid_request_error'}}, 400)
        with self.app.lock:
            session = self.app.session
        if not session or self.app.stopped:
            return self.send({'error': {'message': '尚未部署练习', 'type': 'server_error'}}, 503)
        if not self.app.ai_slots[seat].acquire(blocking=False):
            return self.send({'error': {'message': '本席位已有请求进行中', 'type': 'rate_limit_error'}}, 429)
        try:
            return self.forward_ai(body, seat, session)
        finally:
            self.app.ai_slots[seat].release()

    def forward_ai(self, body, seat, session):
        if self.app.args.ai_mode == 'mock':
            code = 200
            result = dict(id='chatcmpl-practice-mock', object='chat.completion', created=int(time.time()), model='practice-mock',
                choices=[dict(index=0, finish_reason='stop', message=dict(role='assistant', content=json.dumps(
                    {'goals': ['resource_high', 'resource_normal', 'reach_target'], 'event_plan': None,
                     'summary': '模拟响应，仅验证通信，未理解事件'}, ensure_ascii=False)))],
                usage=dict(prompt_tokens=0, completion_tokens=0, total_tokens=0))
        else:
            payload = dict(body)
            if not payload.get('model'):
                payload['model'] = os.environ.get('PRACTICE_MODEL', '')
            if not payload['model']:
                return self.send({'error': {'message': '须提供 model 或设置 PRACTICE_MODEL', 'type': 'invalid_request_error'}}, 400)
            try:
                with requests.post(os.environ['PRACTICE_LLM_URL'], json=payload, headers={
                    'Authorization': 'Bearer ' + os.environ['PRACTICE_LLM_KEY'],
                    'Accept-Encoding': 'identity'}, timeout=(5, 25), allow_redirects=False) as response:
                    code = response.status_code
                    if 300 <= code < 400:
                        code, result = 502, {'error': {'message': '上游地址发生重定向，请配置最终请求地址', 'type': 'upstream_error'}}
                    else:
                        result = response.json()
            except (requests.RequestException, ValueError):
                code, result = 502, {'error': {'message': '上游模型调用失败', 'type': 'upstream_error'}}
        success = code == 200 and isinstance(result, dict) and bool(result.get('choices'))
        record = dict(seat=seat, session_id=session, mode=self.app.args.ai_mode, success=success, status=code)
        with self.app.lock:
            if session == self.app.session and not self.app.stopped:
                previous = self.app.ai.get(seat, {})
                record['success_count'] = previous.get('success_count', 0) + int(success)
                record['attempt_count'] = previous.get('attempt_count', 0) + 1
                try:
                    with (self.app.run / 'ai.jsonl').open('a') as stream:
                        stream.write(json.dumps(record) + '\n')
                except OSError:
                    return self.send({'error': {'message': 'AI 记录写入失败', 'type': 'server_error'}}, 500)
                self.app.ai[seat] = record
        self.send(result, code)


def main():
    parser = argparse.ArgumentParser(description='公开练习裁判：仅练习场景，不产生正式赛事成绩')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--stage', type=int, choices=(1, 2), default=2)
    parser.add_argument('--tick-ms', type=int)
    parser.add_argument('--max-ticks', type=int)
    parser.add_argument('--bots', default='', help='陪练席位，例如 B,C,D；空表示只接真实选手')
    parser.add_argument('--config', help='完整练习配置 JSON')
    parser.add_argument('--data-dir', default=str(Path.home() / '.rmtc-practice'))
    parser.add_argument('--ai-mode', choices=('mock', 'real'), default='mock')
    args = parser.parse_args()
    if args.ai_mode == 'real':
        url = urlsplit(os.environ.get('PRACTICE_LLM_URL', ''))
        if url.scheme not in ('http', 'https') or not url.netloc or not os.environ.get('PRACTICE_LLM_KEY'):
            parser.error('真实 AI 模式须设置 PRACTICE_LLM_URL（完整 completions 地址）和 PRACTICE_LLM_KEY')
    os.environ.setdefault('ROS_DOMAIN_ID', '27')
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    rclpy.init()
    app = None
    server = Server(('127.0.0.1', args.port), Handler)
    stopped = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    try:
        app = Practice(args)
        server.app = app
        # Gateway must be serving before optional companion robots start.
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        settings = dict(stage=args.stage, tick_ms=args.tick_ms if args.tick_ms is not None else (100 if args.stage == 1 else 500), max_ticks=args.max_ticks if args.max_ticks is not None else (100 if args.stage == 1 else 500),
                        bots=args.bots.split(',') if args.bots else [])
        if args.config:
            settings['config'] = json.loads(Path(args.config).read_text())
        with app.command_lock:
            app.start(settings)
        print(f'练习裁判已就绪（暂停）：http://127.0.0.1:{args.port}', flush=True)
        print('测试令牌：practice-A / practice-B / practice-C / practice-D；模拟 AI 不验证事件理解。', flush=True)
        while not stopped.wait(.5):
            app.snapshot()
    finally:
        try:
            if app is not None:
                server.shutdown()
                with app.command_lock:
                    app.close()
        finally:
            try:
                server.server_close()
            finally:
                rclpy.shutdown()


if __name__ == '__main__':
    main()
