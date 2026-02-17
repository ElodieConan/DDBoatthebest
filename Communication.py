# network_ddboat2.py
# Compatible Python 3.4
# TCP swarm networking:
# - 1 server thread (accept loop)
# - 1 thread per incoming connection (recv handler)
# - N outgoing client threads (one per peer)
# Message format (simple & robust): "j=<int>;phase=<float>\n"
#
# Usage (in arduino_driver.py):
#   from network_ddboat2 import DDBoatNetwork2
#   PEERS = [("172.20.25.208", 29200), ("172.20.25.211", 29200)]
#   net = DDBoatNetwork2(ddboat_id=0, my_port=29200, peers=PEERS, bind_ip="0.0.0.0", send_hz=10.0)
#   net.start()
#   ...
#   net.update_my_data(j=last_sommet-1, phase=phase_dd_j)
#   p208 = net.get_peer_phase("172.20.25.208")
#   p211 = net.get_peer_phase("172.20.25.211")

from __future__ import print_function

import socket
import threading
import time


class DDBoatNetwork2(object):
    def __init__(self,
                 ddboat_id,
                 my_port,
                 peers,
                 bind_ip="0.0.0.0",
                 send_hz=5.0,
                 connect_retry_s=1.0,
                 recv_timeout_s=2.0,
                 tcp_nodelay=True,
                 verbose=True):
        """
        ddboat_id: int (for logs)
        my_port: int (server port)
        peers: list of (ip, port) tuples
        bind_ip: usually "0.0.0.0"
        send_hz: float, how often to send to each peer
        connect_retry_s: float, reconnect delay
        recv_timeout_s: float, recv timeout for incoming handlers
        tcp_nodelay: bool, disable Nagle for lower latency
        verbose: bool, print logs
        """
        self.ddboat_id = int(ddboat_id)
        self.bind_ip = bind_ip
        self.my_port = int(my_port)
        self.peers = list(peers)

        self.send_hz = float(send_hz) if float(send_hz) > 0.0 else 1.0
        self.send_period = 1.0 / self.send_hz
        self.connect_retry_s = float(connect_retry_s)
        self.recv_timeout_s = float(recv_timeout_s)
        self.tcp_nodelay = bool(tcp_nodelay)
        self.verbose = bool(verbose)

        # Local data to publish
        self._my_lock = threading.Lock()
        self._my_j = 0
        self._my_phase = 0.0

        # Peer data received: peer_ip -> dict
        self._peer_lock = threading.Lock()
        self._peer_data = {}  # {ip: {"j":int,"phase":float,"timestamp":float}}

        # Running control
        self._running = False

        # Server
        self._server_sock = None
        self._accept_thread = None

        # Threads
        self._out_threads = []  # one per peer
        self._in_threads = []  # one per incoming connection

        if self.verbose:
            print("[Network2] Init DDBoat #%d" % self.ddboat_id)
            print("[Network2] bind_ip=%s my_port=%d" % (self.bind_ip, self.my_port))
            print("[Network2] peers=%s" % (self.peers,))

    # ---------------------------
    # Public API
    # ---------------------------
    def start(self):
        if self._running:
            return
        self._running = True

        # Create server socket
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            if self.tcp_nodelay:
                self._server_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except Exception:
            pass

        self._server_sock.bind((self.bind_ip, self.my_port))
        self._server_sock.listen(20)
        self._server_sock.settimeout(1.0)

        self._accept_thread = threading.Thread(target=self._accept_loop)
        self._accept_thread.daemon = True
        self._accept_thread.start()

        # Start outgoing client threads (one per peer)
        for (ip, port) in self.peers:
            t = threading.Thread(target=self._outgoing_loop, args=(ip, int(port)))
            t.daemon = True
            t.start()
            self._out_threads.append(t)

        if self.verbose:
            print("[Network2] Started DDBoat #%d" % self.ddboat_id)

    def stop(self):
        self._running = False
        if self._server_sock:
            try:
                self._server_sock.close()
            except Exception:
                pass
        if self.verbose:
            print("[Network2] Stopped DDBoat #%d" % self.ddboat_id)

    def update_my_data(self, j, phase):
        with self._my_lock:
            self._my_j = int(j)
            self._my_phase = float(phase)

    def get_peer_data(self):
        # returns a shallow copy
        with self._peer_lock:
            return dict(self._peer_data)
            # return self._peer_data

    def get_peer_phase(self, peer_ip):
        with self._peer_lock:
            d = self._peer_data.get(peer_ip)

            if d is None:
                return None
            return d.get("phase", None)
            # return d

    def get_peer_j(self, peer_ip):
        with self._peer_lock:
            d = self._peer_data.get(peer_ip)
            if d is None:
                return None
            return d.get("j", None)

    def get_peer_age_s(self, peer_ip):
        with self._peer_lock:
            d = self._peer_data.get(peer_ip)
            if d is None:
                return None
            return time.time() - float(d.get("timestamp", 0.0))

    # ---------------------------
    # Internal: server accept loop
    # ---------------------------
    def _accept_loop(self):
        if self.verbose:
            print("[Server2 %d] Listening on %s:%d" % (self.ddboat_id, self.bind_ip, self.my_port))

        while self._running:
            try:
                conn, addr = self._server_sock.accept()
                peer_ip = addr[0]

                try:
                    if self.tcp_nodelay:
                        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                except Exception:
                    pass

                conn.settimeout(self.recv_timeout_s)

                if self.verbose:
                    print("[Server2 %d] Accepted from %s:%d" % (self.ddboat_id, addr[0], addr[1]))

                th = threading.Thread(target=self._incoming_handler, args=(conn, peer_ip))
                th.daemon = True
                th.start()
                self._in_threads.append(th)

            except socket.timeout:
                continue
            except Exception as e:
                # During shutdown, accept may raise because socket closed
                if self._running and self.verbose:
                    print("[Server2 %d] accept error: %s" % (self.ddboat_id, str(e)))
                time.sleep(0.1)

        if self.verbose:
            print("[Server2 %d] accept loop stopped" % self.ddboat_id)

    # ---------------------------
    # Internal: per-incoming-connection handler
    # ---------------------------
    def _incoming_handler(self, conn, peer_ip):
        buf = b""
        try:
            while self._running:
                try:
                    data = conn.recv(1024)
                    if not data:
                        break
                    buf += data

                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        msg = None
                        try:
                            msg = line.decode("utf-8", "ignore").strip()
                        except Exception:
                            continue
                        if not msg:
                            continue

                        parsed = self._parse_message(msg)
                        if parsed is None:
                            continue

                        j = parsed.get("j", None)
                        phase = parsed.get("phase", None)
                        if j is None or phase is None:
                            continue

                        with self._peer_lock:
                            self._peer_data[peer_ip] = {
                                "j": int(j),
                                "phase": float(phase),
                                "timestamp": time.time()
                            }

                        if self.verbose:
                            print("[Server2 %d] Recu de %s: j=%d phase=%.3f" %
                                  (self.ddboat_id, peer_ip, int(j), float(phase)))

                except socket.timeout:
                    continue
                except Exception:
                    break
        finally:
            try:
                conn.close()
            except Exception:
                pass
            if self.verbose:
                print("[Server2 %d] Handler closed for %s" % (self.ddboat_id, peer_ip))

    # ---------------------------
    # Internal: outgoing loop (one per peer)
    # ---------------------------
    def _outgoing_loop(self, peer_ip, peer_port):
        tag = "%s:%d" % (peer_ip, peer_port)
        if self.verbose:
            print("[Client2 %d] Thread for %s started" % (self.ddboat_id, tag))

        while self._running:
            s = None
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                try:
                    if self.tcp_nodelay:
                        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                except Exception:
                    pass

                s.settimeout(2.0)
                s.connect((peer_ip, peer_port))
                s.settimeout(None)

                if self.verbose:
                    print("[Client2 %d] Connected to %s" % (self.ddboat_id, tag))

                # Send loop
                while self._running:
                    with self._my_lock:
                        j = self._my_j
                        phase = self._my_phase

                    msg = self._format_message(j, phase)
                    try:
                        s.sendall(msg)
                    except Exception:
                        break

                    time.sleep(self.send_period)

            except Exception:
                if self.verbose:
                    print("[Client2 %d] Retry connection to %s" % (self.ddboat_id, tag))
                time.sleep(self.connect_retry_s)

            finally:
                if s is not None:
                    try:
                        s.close()
                    except Exception:
                        pass
                if self._running:
                    time.sleep(self.connect_retry_s)

        if self.verbose:
            print("[Client2 %d] Thread for %s stopped" % (self.ddboat_id, tag))

    # ---------------------------
    # Message format helpers
    # ---------------------------
    def _format_message(self, j, phase):
        # bytes ended with \n
        # Example: b"j=3;phase=1.570796\n"
        return ("%d|%.6f\n" % (14, phase)).encode("utf-8")

    def _parse_message(self, msg):
        try:
            if "|" in msg:
                parts = msg.split("|")
                if len(parts) != 2:
                    return None
                return {
                    "j": int(parts[0].strip()),
                    "phase": float(parts[1].strip())
                }
            return None
        except:
            return None